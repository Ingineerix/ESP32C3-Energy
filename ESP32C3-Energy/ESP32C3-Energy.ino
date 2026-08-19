/*
 * BL0906 6-Channel Energy Monitor for ESP32-C3 (4MB Flash)
 * Web dashboard with daily graphs and kWh totals
 * OTA update support: ArduinoOTA (LAN push) + HTTP pull from update server
 *
 * NOTE: MUST PARTITION WITH SPIFFS!
 *
 * Channel mapping:
 *   CT1 + CT2 -> House split-phase  (Graph 1, kW, can go negative for solar export)
 *   CT3 + CT4 -> Lab split-phase    (Graph 2, kW)
 *   CT5       -> EV charger 240V    (Graph 3, kW - one CT on each leg)
 *   CT6       -> Not used
 *   Graph 4   -> Total kW (sum of above)
 *   Graph 5   -> AC Voltage (per-leg, ~120V)
 *
 * Note: All watt readings divided by 2 - BL0906 measures full 240V but each CT
 *       is on one 120V leg, so raw readings are 2x actual power.
 *
 * Wiring:
 *   GPIO7 (UART1 TX) -> BL0906 RX pin
 *   GPIO8 (UART1 RX) -> BL0906 TX pin
 *
 * Board settings:
 *   Board: "ESP32C3 Dev Module"
 *   Flash: 4MB, Partition: Default (SPIFFS 1.5MB)
 *   USB CDC On Boot: Enabled (for Serial monitor)
 *
 * Required libraries (all included in ESP32 Arduino core 2.x+):
 *   WiFi, WebServer, SPIFFS, ArduinoOTA, HTTPUpdate
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <time.h>
#include "Telnet.h"

// ============================================================
// USER CONFIGURATION - edit config.h, not this file
// ============================================================
#include "config.h"

// ============================================================
// BUILD-TIME FIRMWARE FILENAME
// ============================================================
//
// The filename embeds the exact compiler timestamp, making every
// build uniquely named:
//   "ESP32C3-Energy-2026-02-21-23:12:32.bin"
//
// Boot-loop prevention logic:
//   - Running firmware requests ITS OWN filename from the server.
//   - If the file exists (HTTP 200) -> it IS a new build -> flash & reboot.
//   - After reboot the NEW firmware requests its filename -> server returns
//     404 (you haven't uploaded that one yet) -> no update -> done.
//   - The 404 on the new filename is also implicit confirmation the update
//     succeeded (the old filename will never be requested again).
//
// Server workflow:
//   1. Build new firmware -> IDE shows build time in filename.
//   2. Copy the .bin to your server: scp ESP32C3-Energy.ino.bin server:/firmware/
//      renaming it to the stamped filename (e.g. via a build script or
//      post_build.py that reads the timestamp from the elf/source).
//   3. Device picks it up within the hour (or use "Check now" button).
//   4. Remove the old .bin from the server at your leisure.
//
// NOTE: Colons in filenames are valid on Linux/macOS servers.
//       If your server runs Windows, change ':' to '-' in the snprintf below.

Telnet LOG;

static String ota_build_filename() {
    // __DATE__ == "Feb 21 2026"  (single-digit day is space-padded: "Feb  1 2026")
    // __TIME__ == "23:12:32"
    const char *d = __DATE__;
    const char *t = __TIME__;

    static const char *months[] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };
    int month = 1;
    for (int i = 0; i < 12; i++) {
        if (strncmp(d, months[i], 3) == 0) { month = i + 1; break; }
    }
    int day  = (d[4] == ' ') ? (d[5] - '0') : ((d[4]-'0')*10 + (d[5]-'0'));
    int year = atoi(d + 7);

    char fname[52];
    snprintf(fname, sizeof(fname),
             "ESP32C3-Energy-%04d-%02d-%02d-%s.bin",
             year, month, day, t);     // t = "HH:MM:SS" verbatim
    return String(fname);
}

// Short human-readable stamp for the UI (strips prefix and .bin)
static String ota_build_stamp() {
    String s = ota_build_filename();
    s.replace("ESP32C3-Energy-", "");
    s.replace(".bin", "");
    return s;
}

// ============================================================
// BL0906 CONVERSION CONSTANTS  (from datasheet / Tasmota)
// ============================================================
static const float BL0906_PREF =
    1.097f * 1.097f * (5.0f * 20000.0f) /
    (40.41259f * ((5.1f + 5.1f) * 1000.0f / 2000.0f) * 100.0f * 1000.0f);

static const float BL0906_UREF =
    1.097f * (5.0f * 20000.0f) / (13162.0f * 100.0f * 1000.0f);

static const float BL0906_KI =
    12875.0f * (5.1f + 5.1f) * 1000.0f / 2000.0f / 1.097f;

// ============================================================
// BL0906 REGISTER ADDRESSES
// ============================================================
#define BL0906_READ_CMD      0x35
#define BL0906_WRITE_CMD     0xCA
#define BL0906_USR_WRPROT    0x9E
#define BL0906_SOFT_RESET    0x9F
#define BL0906_V_RMS         0x16
#define BL0906_WATT_1        0x23
#define BL0906_WATT_2        0x24
#define BL0906_WATT_3        0x25
#define BL0906_WATT_4        0x26
#define BL0906_WATT_5        0x29
#define BL0906_RMSOS_1       0x78
#define BL0906_RMSOS_2       0x79
#define BL0906_RMSOS_3       0x7A
#define BL0906_RMSOS_4       0x7B
#define BL0906_RMSOS_5       0x7E

static const uint8_t BL0906_UNLOCK[] = {BL0906_WRITE_CMD, BL0906_USR_WRPROT, 0x55, 0x55, 0x00, 0xB7};
static const uint8_t BL0906_LOCK[]   = {BL0906_WRITE_CMD, BL0906_USR_WRPROT, 0x00, 0x00, 0x00, 0x61};
static const uint8_t BL0906_INIT[]   = {BL0906_WRITE_CMD, BL0906_SOFT_RESET, 0x5A, 0x5A, 0x5A, 0x52};

// ============================================================
// DATA STORAGE
// ============================================================
#define GRAPH_POINTS_MAX  2880   // 24h @ 30s/point
#define NUM_GRAPHS        5

enum GraphChannel { CH_HOUSE=0, CH_LAB=1, CH_EV=2, CH_TOTAL=3, CH_VOLTAGE=4 };

float   gGraph[NUM_GRAPHS][GRAPH_POINTS_MAX];
uint8_t gGraphHour[GRAPH_POINTS_MAX];
uint8_t gGraphMin[GRAPH_POINTS_MAX];
uint8_t gGraphSec[GRAPH_POINTS_MAX];
int     gGraphCount = 0;

double  gAccum[NUM_GRAPHS] = {0};
int     gAccumCount = 0;
int     gLastGraph30s = -1;   // tracks unique 30-second bucket: hour*120 + min*2 + sec/30

float   gLiveW[4] = {0};
float   gLiveV    = 0.0f;

struct EnergyTotals {
    double daily[4], monthly[4], yearly[4], alltime[4];
    double gen_daily, gen_monthly, gen_yearly, gen_alltime;
    int    last_day, last_month, last_year;
};
EnergyTotals gEnergy = {};

// ============================================================
// GLOBAL STATE
// ============================================================
WebServer     gServer(80);
bool          gTimeValid      = false;
unsigned long gLastSaveMs     = 0;
unsigned long gLastSecondMs   = 0;

bool          gOtaInProgress  = false;
String        gOtaStatusMsg   = "Pending first check";
bool          gMidnightRebooted = false;

// ============================================================
// BL0906 COMMUNICATION
// ============================================================
static uint8_t bl0906_checksum(uint8_t a, uint8_t l, uint8_t m, uint8_t h) {
    return (uint8_t)((a + l + m + h) ^ 0xFF);
}

static bool bl0906_read_reg(uint8_t address, bool is_signed, float ref, float &result) {
    while (Serial1.available()) Serial1.read();
    Serial1.write(BL0906_READ_CMD);
    Serial1.write(address);

    uint32_t t0 = millis();
    while (Serial1.available() < 4) {
        if (millis() - t0 > 100) {
            LOG.printf("BL0906: TIMEOUT reg 0x%02X (no response)\n", address);
            return false;
        }
        yield();
    }
    uint8_t l = Serial1.read(), m = Serial1.read(),
            h = Serial1.read(), cs = Serial1.read();
    while (Serial1.available()) Serial1.read();

    uint8_t expected = bl0906_checksum(address, l, m, h);
    if (expected != cs) {
        LOG.printf("BL0906: CRC error reg 0x%02X  bytes=%02X %02X %02X %02X  expected_cs=0x%02X\n",
                      address, l, m, h, cs, expected);
        return false;
    }
    if (is_signed) {
        result = (float)(((int32_t)(int8_t)h << 16) | ((int32_t)m << 8) | l) * ref;
    } else {
        result = (float)(((uint32_t)h << 16) | ((uint32_t)m << 8) | l) * ref;
    }
    return true;
}

static void bl0906_write_reg(uint8_t address, uint8_t l, uint8_t m, uint8_t h) {
    Serial1.write(BL0906_WRITE_CMD); Serial1.write(address);
    Serial1.write(l); Serial1.write(m); Serial1.write(h);
    Serial1.write(bl0906_checksum(address, l, m, h));
}

static void bl0906_bias_correct(uint8_t reg, float idle, float target) {
    float i0 = idle   * BL0906_KI;
    float it = target * BL0906_KI;
    int32_t v = (int32_t)((it*it - i0*i0) / 256.0f);
    bl0906_write_reg(reg, (uint8_t)(v&0xFF), (uint8_t)(v>>8&0xFF), (uint8_t)(v>>16&0xFF));
}

static void bl0906_init() {
    Serial1.write(BL0906_INIT, sizeof(BL0906_INIT)); delay(100);
    while (Serial1.available()) Serial1.read();
    Serial1.write(BL0906_UNLOCK, sizeof(BL0906_UNLOCK)); delay(10);
    bl0906_bias_correct(BL0906_RMSOS_1, 0.0160f, 0.0f);
    bl0906_bias_correct(BL0906_RMSOS_2, 0.0150f, 0.0f);
    bl0906_bias_correct(BL0906_RMSOS_3, 0.0140f, 0.0f);
    bl0906_bias_correct(BL0906_RMSOS_4, 0.0130f, 0.0f);
    bl0906_bias_correct(BL0906_RMSOS_5, 0.0120f, 0.0f);
    Serial1.write(BL0906_LOCK, sizeof(BL0906_LOCK)); delay(10);
    while (Serial1.available()) Serial1.read();
    LOG.println("BL0906: initialized");
}

static bool bl0906_read_all() {
    float w1,w2,w3,w4,w5,v;
    bool ok = true;
    ok &= bl0906_read_reg(BL0906_WATT_1, true,  BL0906_PREF, w1);
    ok &= bl0906_read_reg(BL0906_WATT_2, true,  BL0906_PREF, w2);
    ok &= bl0906_read_reg(BL0906_WATT_3, true,  BL0906_PREF, w3);
    ok &= bl0906_read_reg(BL0906_WATT_4, true,  BL0906_PREF, w4);
    ok &= bl0906_read_reg(BL0906_WATT_5, true,  BL0906_PREF, w5);
    ok &= bl0906_read_reg(BL0906_V_RMS,  false, BL0906_UREF, v);
    if (!ok) return false;
    // 240V split-phase: BL0906 sees full 240V but each CT is on one leg (120V),
    // so raw watt readings are 2x actual. Divide all channels by 2.
    w1/=2; w2/=2; w3/=2; w4/=2; w5/=2;
    // CT2 and CT4 are reverse-polarity - negate to correct
    w2 = -w2;
    w4 = -w4;
    w5 *= CT5_MULTIPLIER;  // additional per-channel trim (default 1.0 for 240V EV)
    gLiveW[CH_HOUSE] = w1+w2; gLiveW[CH_LAB] = w3+w4;
    gLiveW[CH_EV]    = w5;
    gLiveW[CH_TOTAL] = gLiveW[CH_HOUSE]+gLiveW[CH_LAB]+gLiveW[CH_EV];
    gLiveV = v / 2.0f;  // show per-leg voltage (~120V)

    // Debug: print raw values every 10 seconds
    static uint32_t dbgLast = 0;
    if (millis() - dbgLast >= 10000) {
        dbgLast = millis();
        LOG.printf("BL0906: w1=%.1f w2=%.1f w3=%.1f w4=%.1f w5=%.1f V=%.1f  heap=%u\n",
                      w1, w2, w3, w4, w5, v, ESP.getFreeHeap());
    }
    return true;
}

// ============================================================
// TIME HELPERS
// ============================================================
static bool get_local_time(struct tm &t) {
    time_t now = time(nullptr);
    if (now < 1000000000UL) return false;
    localtime_r(&now, &t);
    return true;
}

// ============================================================
// ENERGY ACCUMULATION
// ============================================================
static void accumulate_energy(float hw, float lw, float ew, float tw) {
    const double k = 1.0 / 3600000.0;
    float vals[4] = {hw, lw, ew, tw};
    for (int i = 0; i < 4; i++) {
        gEnergy.daily[i]   += vals[i]*k;
        gEnergy.monthly[i] += vals[i]*k;
        gEnergy.yearly[i]  += vals[i]*k;
        gEnergy.alltime[i] += vals[i]*k;
    }
    if (hw < 0.0f) {
        double g = -hw*k;
        gEnergy.gen_daily   += g; gEnergy.gen_monthly += g;
        gEnergy.gen_yearly  += g; gEnergy.gen_alltime  += g;
    }
}

static void check_rollover(const struct tm &t) {
    if (!gEnergy.last_day) {
        gEnergy.last_day = t.tm_mday; gEnergy.last_month = t.tm_mon;
        gEnergy.last_year = t.tm_year; return;
    }
    bool ny = t.tm_year != gEnergy.last_year;
    bool nm = ny || t.tm_mon  != gEnergy.last_month;
    bool nd = nm || t.tm_mday != gEnergy.last_day;
    if (ny) { memset(gEnergy.yearly,  0, sizeof(gEnergy.yearly));  gEnergy.gen_yearly  = 0; gEnergy.last_year  = t.tm_year; }
    if (nm) { memset(gEnergy.monthly, 0, sizeof(gEnergy.monthly)); gEnergy.gen_monthly = 0; gEnergy.last_month = t.tm_mon;  }
    if (nd) {
        memset(gEnergy.daily, 0, sizeof(gEnergy.daily));
        gEnergy.gen_daily = 0; gEnergy.last_day = t.tm_mday;
        gGraphCount = 0; memset(gGraph, 0, sizeof(gGraph)); gLastGraph30s = -1;
        LOG.println("Day rollover: graph reset");
    }
}

static void commit_graph_point(const struct tm &t) {
    if (!gAccumCount) return;
    if (gGraphCount >= GRAPH_POINTS_MAX) {
        for (int ch = 0; ch < NUM_GRAPHS; ch++)
            memmove(&gGraph[ch][0], &gGraph[ch][1], sizeof(float)*(GRAPH_POINTS_MAX-1));
        memmove(gGraphHour, gGraphHour+1, GRAPH_POINTS_MAX-1);
        memmove(gGraphMin,  gGraphMin +1, GRAPH_POINTS_MAX-1);
        memmove(gGraphSec,  gGraphSec +1, GRAPH_POINTS_MAX-1);
        gGraphCount = GRAPH_POINTS_MAX-1;
    }
    int i = gGraphCount;
    gGraph[CH_HOUSE][i]   = (float)(gAccum[CH_HOUSE]  /gAccumCount)/1000.0f;
    gGraph[CH_LAB][i]     = (float)(gAccum[CH_LAB]    /gAccumCount)/1000.0f;
    gGraph[CH_EV][i]      = (float)(gAccum[CH_EV]     /gAccumCount)/1000.0f;
    gGraph[CH_TOTAL][i]   = (float)(gAccum[CH_TOTAL]  /gAccumCount)/1000.0f;
    gGraph[CH_VOLTAGE][i] = (float)(gAccum[CH_VOLTAGE]/gAccumCount);
    gGraphHour[i] = t.tm_hour; gGraphMin[i] = t.tm_min; gGraphSec[i] = t.tm_sec;
    gGraphCount++;
    memset(gAccum, 0, sizeof(gAccum)); gAccumCount = 0;
    LOG.printf("Graph[%d] %02d:%02d:%02d house=%.3fkW lab=%.3fkW ev=%.3fkW total=%.3fkW V=%.1f\n",
               i, t.tm_hour, t.tm_min, t.tm_sec,
               gGraph[CH_HOUSE][i], gGraph[CH_LAB][i], gGraph[CH_EV][i],
               gGraph[CH_TOTAL][i], gGraph[CH_VOLTAGE][i]);
}

// ============================================================
// SPIFFS PERSISTENCE
// ============================================================
#define TOTALS_FILE "/energy_totals.json"

static void spiffs_save_totals() {
    File f = SPIFFS.open(TOTALS_FILE, "w");
    if (!f) { LOG.println("SPIFFS: write failed"); return; }
    f.printf("{\"daily\":[%.6f,%.6f,%.6f,%.6f],",
        gEnergy.daily[0],gEnergy.daily[1],gEnergy.daily[2],gEnergy.daily[3]);
    f.printf("\"monthly\":[%.6f,%.6f,%.6f,%.6f],",
        gEnergy.monthly[0],gEnergy.monthly[1],gEnergy.monthly[2],gEnergy.monthly[3]);
    f.printf("\"yearly\":[%.6f,%.6f,%.6f,%.6f],",
        gEnergy.yearly[0],gEnergy.yearly[1],gEnergy.yearly[2],gEnergy.yearly[3]);
    f.printf("\"alltime\":[%.6f,%.6f,%.6f,%.6f],",
        gEnergy.alltime[0],gEnergy.alltime[1],gEnergy.alltime[2],gEnergy.alltime[3]);
    f.printf("\"gen_daily\":%.6f,\"gen_monthly\":%.6f,\"gen_yearly\":%.6f,\"gen_alltime\":%.6f,",
        gEnergy.gen_daily,gEnergy.gen_monthly,gEnergy.gen_yearly,gEnergy.gen_alltime);
    f.printf("\"last_day\":%d,\"last_month\":%d,\"last_year\":%d}",
        gEnergy.last_day,gEnergy.last_month,gEnergy.last_year);
    f.close();
    LOG.println("SPIFFS: totals saved");
}

static double parse_json_double(const String &j, const char *key) {
    String s = "\""; s += key; s += "\":";
    int idx = j.indexOf(s);
    return (idx<0) ? 0.0 : j.substring(idx+s.length()).toDouble();
}
static void parse_json_array4(const String &j, const char *key, double out[4]) {
    String s = "\""; s += key; s += "\":[";
    int idx = j.indexOf(s); if (idx<0) return;
    String r = j.substring(idx+s.length());
    for (int i=0;i<4;i++) {
        out[i] = r.toDouble();
        int c = r.indexOf(','), b = r.indexOf(']');
        if (c<0||(b>=0&&b<c)) break;
        r = r.substring(c+1);
    }
}
static void spiffs_load_totals() {
    if (!SPIFFS.exists(TOTALS_FILE)) { LOG.println("SPIFFS: no saved totals"); return; }
    File f = SPIFFS.open(TOTALS_FILE,"r"); if(!f) return;
    String j = f.readString(); f.close();
    parse_json_array4(j,"daily",   gEnergy.daily);
    parse_json_array4(j,"monthly", gEnergy.monthly);
    parse_json_array4(j,"yearly",  gEnergy.yearly);
    parse_json_array4(j,"alltime", gEnergy.alltime);
    gEnergy.gen_daily   = parse_json_double(j,"gen_daily");
    gEnergy.gen_monthly = parse_json_double(j,"gen_monthly");
    gEnergy.gen_yearly  = parse_json_double(j,"gen_yearly");
    gEnergy.gen_alltime = parse_json_double(j,"gen_alltime");
    gEnergy.last_day    = (int)parse_json_double(j,"last_day");
    gEnergy.last_month  = (int)parse_json_double(j,"last_month");
    gEnergy.last_year   = (int)parse_json_double(j,"last_year");
    LOG.printf("SPIFFS: loaded (alltime=%.3f kWh)\n", gEnergy.alltime[CH_HOUSE]);
}

// ============================================================
// OTA SYSTEM 1: ArduinoOTA  (LAN push from Arduino IDE)
// ============================================================
// Usage: In the Arduino IDE, look for "energy-monitor" under
//   Tools > Port > Network ports after the device is on your LAN.
//   When prompted, enter the password defined in ARDUINO_OTA_PASSWORD.
//
// During the transfer the main loop is blocked (normal for OTA).
// Energy totals are saved to SPIFFS before transfer begins so no
// accumulated kWh data is lost.
// ============================================================
#if ARDUINO_OTA_ENABLED
static void ota_setup_arduino() {
    ArduinoOTA.setHostname(ARDUINO_OTA_HOSTNAME);
    ArduinoOTA.setPassword(ARDUINO_OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        LOG.println("ArduinoOTA: start -> " + type);
        gOtaInProgress = true;
        gOtaStatusMsg  = "ArduinoOTA uploading";
        spiffs_save_totals();   // protect kWh data before flash erase
    });

    ArduinoOTA.onEnd([]() {
        LOG.println("\nArduinoOTA: done, rebooting");
        gOtaInProgress = false;
        // device reboots automatically
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static int last = -1;
        int pct = progress * 100 / total;
        if (pct != last && pct % 10 == 0) {
            LOG.printf("ArduinoOTA: %u%%\n", pct);
            last = pct;
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        gOtaInProgress = false;
        const char *msg = "Unknown";
        switch (error) {
            case OTA_AUTH_ERROR:    msg = "Auth Failed";    break;
            case OTA_BEGIN_ERROR:   msg = "Begin Failed";   break;
            case OTA_CONNECT_ERROR: msg = "Connect Failed"; break;
            case OTA_RECEIVE_ERROR: msg = "Receive Failed"; break;
            case OTA_END_ERROR:     msg = "End Failed";     break;
        }
        LOG.printf("ArduinoOTA error[%u]: %s\n", error, msg);
        gOtaStatusMsg = String("ArduinoOTA error: ") + msg;
    });

    ArduinoOTA.begin();
    LOG.printf("ArduinoOTA: ready  host=%s\n", ARDUINO_OTA_HOSTNAME);
}
#endif  // ARDUINO_OTA_ENABLED

// ============================================================
// OTA SYSTEM 2: HTTP pull OTA
// ============================================================
// How boot-loop prevention works - see detailed comment near
// ota_build_filename() above for the full explanation.
//
// Short version:
//   Device requests its OWN build-stamped .bin from the server.
//   200 -> update -> reboot -> new firmware -> its filename 404s -> stop.
//   404 immediately -> nothing to do -> loop continues normally.
//
// The server sees the old filename disappear from the logs after a
// successful update - implicit success confirmation, no extra endpoint
// needed on the server side.
// ============================================================
#if HTTP_OTA_ENABLED
static void ota_check_http() {
    if (WiFi.status() != WL_CONNECTED) {
        LOG.println("HTTP OTA: skipped (WiFi not connected)");
        return;
    }

    String filename = ota_build_filename();
    String url = String(OTA_SERVER_URL) + filename + "?ip=" + WiFi.localIP().toString();
//    LOG.printf("HTTP OTA: checking %s\n", url.c_str());

    gOtaStatusMsg  = "Checking...";
    gOtaInProgress = true;

    // Save energy data BEFORE the blocking update call; if the update
    // succeeds the device reboots inside httpUpdate.update() and we
    // would never reach a post-update save.
    spiffs_save_totals();

    WiFiClient client;
    // Disable redirects so a server-side redirect doesn't hide a 404
//    httpUpdate.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);

    httpUpdate.onProgress([](int cur, int total) {
        static int last = -1;
        if (total > 0) {
            int pct = cur * 100 / total;
            if (pct != last && pct % 10 == 0) {
                LOG.printf("HTTP OTA: %d%%  (%d/%d bytes)\n", pct, cur, total);
                last = pct;
            }
        }
    });

    t_httpUpdate_return ret = httpUpdate.update(client, url);
    // If ret == HTTP_UPDATE_OK we never reach here (device rebooted)

    gOtaInProgress = false;

    switch (ret) {
        case HTTP_UPDATE_OK:
            // Unreachable - httpUpdate reboots on success
            LOG.println("HTTP OTA: update OK, rebooting");
            gOtaStatusMsg = "Updated - rebooting";
            break;

        case HTTP_UPDATE_NO_UPDATES:
            // Server returned HTTP 304 Not Modified
            LOG.println("HTTP OTA: no update available (304)");
            gOtaStatusMsg = "Up to date (304)";
            break;

        case HTTP_UPDATE_FAILED: {
            // getLastError() returns an HTTPUpdateError code; the raw HTTP
            // status code is not exposed in ESP32 core 3.x.
            // A "file not found" (404) result appears here as a normal
            // failure - this is the expected steady-state response once
            // the device has already updated and its own filename no
            // longer exists on the server.
            int errCode = httpUpdate.getLastError();
            String errStr = httpUpdate.getLastErrorString();
            LOG.printf("HTTP OTA: no update (err %d: %s) - '%s'\n",
                          errCode, errStr.c_str(), filename.c_str());
            // HTTP_UPDATE_FAILED with error -1 / "Connection failed" means
            // the server was unreachable; anything else is typically a 404
            // or similar "file not present" response = firmware is current.
            if (errCode == -1) {
                gOtaStatusMsg = "Server unreachable";
            } else {
                gOtaStatusMsg = "Current (" + errStr + ")";
            }
            break;
        }
    }
}
#endif  // HTTP_OTA_ENABLED

// ============================================================
// HTML - Part 1 (structure + styles)
// ============================================================
static const char HTML_PART1[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Energy Monitor</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d0d0d;color:#e0e0e0;font-family:'Segoe UI',Arial,sans-serif;padding:10px}
h1{text-align:center;color:#4db8ff;font-size:1.4em;margin-bottom:4px}
#status{text-align:center;font-size:.8em;color:#666;margin-bottom:10px}
.live-bar{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin-bottom:14px}
.live-card{background:#1a1a2e;border:1px solid #333;border-radius:8px;
  padding:10px 16px;min-width:130px;text-align:center}
.live-card .label{font-size:.75em;color:#888;text-transform:uppercase;letter-spacing:1px}
.live-card .value{font-size:1.5em;font-weight:700;margin-top:4px}
.positive{color:#4cff91}.negative{color:#ff4c6a}.neutral{color:#4db8ff}.voltage-val{color:#ffd740}
.charts-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:14px}
.chart-full{grid-column:1/-1}
.chart-box{background:#111;border:1px solid #2a2a2a;border-radius:8px;padding:12px}
.chart-box h3{font-size:.85em;color:#aaa;margin-bottom:6px;
  text-transform:uppercase;letter-spacing:1px}
canvas{max-height:200px}
.totals-section{background:#111;border:1px solid #2a2a2a;border-radius:8px;
  padding:12px;margin-bottom:10px}
.totals-section h2{font-size:1em;color:#4db8ff;margin-bottom:10px}
table{width:100%;border-collapse:collapse;font-size:.82em}
th{text-align:left;color:#888;padding:4px 8px;
  border-bottom:1px solid #2a2a2a;font-weight:normal}
td{padding:5px 8px;border-bottom:1px solid #1a1a1a}
tr:last-child td{border-bottom:none}
.row-house{color:#4db8ff}.row-lab{color:#b44dff}
.row-ev{color:#ffd740}.row-total{color:#4cff91}
.gen-section{margin-top:10px;padding-top:10px;border-top:1px solid #2a2a2a}
.gen-section h3{font-size:.85em;color:#ff8c4d;margin-bottom:6px}
.footer{text-align:center;margin-top:10px;font-size:.75em;color:#444;
  display:flex;align-items:center;justify-content:center;gap:14px}
.btn-reboot{padding:4px 12px;border-radius:4px;border:1px solid #4a2a2a;
  background:#1a1a1a;color:#ff6b6b;cursor:pointer;font-size:.8em}
.btn-reboot:hover{background:#2a1a1a}
@media(max-width:600px){.charts-grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<h1>&#9889; Energy Monitor</h1>
<div id="status">Updated: --</div>

<div class="live-bar">
  <div class="live-card">
    <div class="label">House</div>
    <div class="value neutral" id="lv-house">--</div>
    <div class="label">kW</div>
  </div>
  <div class="live-card">
    <div class="label">Lab</div>
    <div class="value neutral" id="lv-lab">--</div>
    <div class="label">kW</div>
  </div>
  <div class="live-card">
    <div class="label">EV</div>
    <div class="value neutral" id="lv-ev">--</div>
    <div class="label">kW</div>
  </div>
  <div class="live-card">
    <div class="label">Total</div>
    <div class="value neutral" id="lv-total">--</div>
    <div class="label">kW</div>
  </div>
  <div class="live-card">
    <div class="label">Voltage</div>
    <div class="value voltage-val" id="lv-volt">--</div>
    <div class="label">V</div>
  </div>
</div>

<div class="charts-grid">
  <div class="chart-box">
    <h3>&#127968; House kW</h3><canvas id="c0"></canvas>
  </div>
  <div class="chart-box">
    <h3>&#128300; Lab kW</h3><canvas id="c1"></canvas>
  </div>
  <div class="chart-box">
    <h3>&#128663; EV Charger kW</h3><canvas id="c2"></canvas>
  </div>
  <div class="chart-box">
    <h3>&#8721; Total kW</h3><canvas id="c3"></canvas>
  </div>
  <div class="chart-box chart-full">
    <h3>&#9889; Voltage (V)</h3><canvas id="c4"></canvas>
  </div>
</div>

<div class="totals-section">
  <h2>&#128202; Energy Totals (kWh)</h2>
  <table>
    <tr><th>Channel</th><th>Today</th><th>This Month</th>
        <th>This Year</th><th>All Time</th></tr>
    <tr class="row-house">
      <td>House</td>
      <td id="t-h-d">--</td><td id="t-h-m">--</td>
      <td id="t-h-y">--</td><td id="t-h-a">--</td>
    </tr>
    <tr class="row-lab">
      <td>Lab</td>
      <td id="t-l-d">--</td><td id="t-l-m">--</td>
      <td id="t-l-y">--</td><td id="t-l-a">--</td>
    </tr>
    <tr class="row-ev">
      <td>EV</td>
      <td id="t-e-d">--</td><td id="t-e-m">--</td>
      <td id="t-e-y">--</td><td id="t-e-a">--</td>
    </tr>
    <tr class="row-total">
      <td>Total</td>
      <td id="t-t-d">--</td><td id="t-t-m">--</td>
      <td id="t-t-y">--</td><td id="t-t-a">--</td>
    </tr>
  </table>
  <div class="gen-section">
    <h3>&#9728; Solar Export (Generated)</h3>
    <table>
      <tr><th>Period</th><th>Today</th><th>This Month</th>
          <th>This Year</th><th>All Time</th></tr>
      <tr style="color:#ff8c4d">
        <td>Generated kWh</td>
        <td id="t-g-d">--</td><td id="t-g-m">--</td>
        <td id="t-g-y">--</td><td id="t-g-a">--</td>
      </tr>
    </table>
  </div>
</div>

<div class="footer">
  <span id="fw-stamp">...</span>
  <button class="btn-reboot" onclick="doReboot()">&#9211; Reboot</button>
</div>
)rawhtml";

// ============================================================
// HTML - Part 2 (JavaScript)
// ============================================================
static const char HTML_PART2[] PROGMEM = R"rawhtml(
<script>
const COLORS = ['#4db8ff','#b44dff','#ffd740','#4cff91','#ff8c4d'];

function makeChart(id, color, yLabel) {
  return new Chart(document.getElementById(id).getContext('2d'), {
    type: 'line',
    data: { labels: [], datasets: [{
      data: [], borderColor: color, backgroundColor: color+'22',
      borderWidth: 1.5, pointRadius: 0, tension: 0.3, fill: true
    }]},
    options: {
      responsive: true, maintainAspectRatio: true, animation: false,
      interaction: { intersect: false, mode: 'index' },
      plugins: {
        legend: { display: false },
        tooltip: { callbacks: { label: c => c.parsed.y.toFixed(3)+' '+yLabel }}
      },
      scales: {
        x: { ticks:{ color:'#666', maxTicksLimit:8, font:{size:10}}, grid:{color:'#1a1a1a'}},
        y: { ticks:{ color:'#888', font:{size:10}}, grid:{color:'#1a1a1a'}, beginAtZero:false }
      }
    }
  });
}

const charts = [
  makeChart('c0', COLORS[0], 'kW'),
  makeChart('c1', COLORS[1], 'kW'),
  makeChart('c2', COLORS[2], 'kW'),
  makeChart('c3', COLORS[3], 'kW'),
  makeChart('c4', COLORS[4], 'V')
];

function updateCharts(data) {
  ['house','lab','ev','total','voltage'].forEach((k,i) => {
    charts[i].data.labels = data.labels;
    charts[i].data.datasets[0].data = data[k];
    charts[i].update('none');
  });
}

function setLive(id, w, isKw) {
  const el = document.getElementById(id);
  const v = w / (isKw ? 1000 : 1);
  el.textContent = isNaN(v) ? '--' : v.toFixed(isKw ? 3 : 1);
  if (isKw) el.className = 'value ' + (v < -0.01 ? 'negative' : v > 0.01 ? 'positive' : 'neutral');
}

function updateLive(d) {
  setLive('lv-house', d.house,   true);
  setLive('lv-lab',   d.lab,     true);
  setLive('lv-ev',    d.ev,      true);
  setLive('lv-total', d.total,   true);
  setLive('lv-volt',  d.voltage, false);
  document.getElementById('status').textContent = 'Updated: ' + d.time;
  document.getElementById('fw-stamp').textContent = 'Version: ' + d.firmware + ' - © 2026 by Phil Sadow' || '';
}

function fmt(v) { return parseFloat(v).toFixed(1); }

function updateTotals(d) {
  const ch = ['h','l','e','t'];
  const pr = ['d','m','y','a'];
  const ky = ['daily','monthly','yearly','alltime'];
  ch.forEach((c,i) => pr.forEach((p,j) => {
    document.getElementById('t-'+c+'-'+p).textContent = fmt(d[ky[j]][i]);
  }));
  document.getElementById('t-g-d').textContent = fmt(d.gen_daily);
  document.getElementById('t-g-m').textContent = fmt(d.gen_monthly);
  document.getElementById('t-g-y').textContent = fmt(d.gen_yearly);
  document.getElementById('t-g-a').textContent = fmt(d.gen_alltime);
}

async function doReboot() {
  if (!confirm('Save totals and reboot?')) return;
  const btn = document.querySelector('.btn-reboot');
  btn.disabled = true;
  btn.textContent = 'Rebooting...';
  try {
    await fetch('/api/reboot', { method: 'POST' });
  } catch(e) {}
  // Device is rebooting - poll until it comes back
  setTimeout(() => { btn.textContent = 'Waiting...'; }, 1000);
  setTimeout(() => location.reload(), 12000);
}

async function fetchGraph()  {
  try {
    const r = await fetch('/api/graph');
    if (!r.ok) { console.error('fetchGraph HTTP', r.status); return; }
    const txt = await r.text();
    let data;
    try { data = JSON.parse(txt); } catch(e) {
      console.error('fetchGraph JSON parse error:', e.message);
      console.error('Response (first 200):', txt.substring(0,200));
      return;
    }
    console.log('fetchGraph ok, count=', data.count, 'labels=', data.labels && data.labels.length);
    updateCharts(data);
  } catch(e) { console.error('fetchGraph fetch error:', e); }
}
async function fetchTotals() { try{const r=await fetch('/api/totals'); if(r.ok) updateTotals(await r.json());}catch(e){} }
async function fetchLive()   { try{const r=await fetch('/api/live');   if(r.ok) updateLive(await r.json());  }catch(e){} }

fetchGraph(); fetchTotals(); fetchLive();
setInterval(fetchLive,   1000);
setInterval(fetchGraph,  35000);
setInterval(fetchTotals, 30000);
</script>
</body>
</html>
)rawhtml";

// ============================================================
// WEB SERVER HANDLERS
// ============================================================
static void handle_api_reboot() {
    gServer.send(200, "application/json", "{\"msg\":\"Rebooting\"}");
    delay(200);
    spiffs_save_totals();
    delay(300);
    ESP.restart();
}

static void handle_root() {
    gServer.sendHeader("Cache-Control", "no-cache");
    gServer.sendHeader("Connection", "close");
    String page = FPSTR(HTML_PART1);
    page += FPSTR(HTML_PART2);
    gServer.send(200, "text/html", page);
}

static void handle_api_live() {
    struct tm t; char ts[16] = "--:-- --";
    if (get_local_time(t)) {
        int h12 = t.tm_hour % 12;
        if (h12 == 0) h12 = 12;
        snprintf(ts, sizeof(ts), "%d:%02d:%02d %s",
                 h12, t.tm_min, t.tm_sec,
                 t.tm_hour < 12 ? "am" : "pm");
    }

    char buf[320];
    snprintf(buf, sizeof(buf),
        "{\"house\":%.2f,\"lab\":%.2f,\"ev\":%.2f,\"total\":%.2f,"
        "\"voltage\":%.2f,\"time\":\"%s\",\"ip\":\"%s\","
        "\"firmware\":\"%s\",\"ota_status\":\"%s\"}",
        gLiveW[CH_HOUSE], gLiveW[CH_LAB], gLiveW[CH_EV], gLiveW[CH_TOTAL],
        gLiveV, ts, WiFi.localIP().toString().c_str(),
        ota_build_stamp().c_str(), gOtaStatusMsg.c_str());

    gServer.sendHeader("Cache-Control", "no-cache");
    gServer.send(200, "application/json", buf);
}

static void handle_api_totals() {
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"daily\":[%.4f,%.4f,%.4f,%.4f],"
        "\"monthly\":[%.4f,%.4f,%.4f,%.4f],"
        "\"yearly\":[%.4f,%.4f,%.4f,%.4f],"
        "\"alltime\":[%.4f,%.4f,%.4f,%.4f],"
        "\"gen_daily\":%.4f,\"gen_monthly\":%.4f,"
        "\"gen_yearly\":%.4f,\"gen_alltime\":%.4f}",
        gEnergy.daily[0],   gEnergy.daily[1],   gEnergy.daily[2],   gEnergy.daily[3],
        gEnergy.monthly[0], gEnergy.monthly[1], gEnergy.monthly[2], gEnergy.monthly[3],
        gEnergy.yearly[0],  gEnergy.yearly[1],  gEnergy.yearly[2],  gEnergy.yearly[3],
        gEnergy.alltime[0], gEnergy.alltime[1], gEnergy.alltime[2], gEnergy.alltime[3],
        gEnergy.gen_daily, gEnergy.gen_monthly,
        gEnergy.gen_yearly, gEnergy.gen_alltime);
    gServer.sendHeader("Cache-Control", "no-cache");
    gServer.send(200, "application/json", buf);
}

static void handle_api_graph() {
    gServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    gServer.sendHeader("Cache-Control", "no-cache");
    gServer.send(200, "application/json", "");

    // Build response in 256-byte chunks to avoid 7000+ individual TCP writes
    static char buf[256];
    int pos = 0;

    #define FLUSH() do { if (pos) { gServer.sendContent(buf, pos); pos = 0; } } while(0)
    #define APPEND(fmt, ...) do { \
        int n = snprintf(buf+pos, sizeof(buf)-pos, fmt, ##__VA_ARGS__); \
        if (n >= (int)(sizeof(buf)-pos)) { FLUSH(); n = snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); } \
        pos += n; \
    } while(0)

    APPEND("{\"labels\":[");
    for (int i = 0; i < gGraphCount; i++)
        APPEND("%s\"%02d:%02d:%02d\"", i?",":"", gGraphHour[i], gGraphMin[i], gGraphSec[i]);
    APPEND("]");

    static const char *KEYS[] = {"house","lab","ev","total","voltage"};
    for (int ch = 0; ch < NUM_GRAPHS; ch++) {
        APPEND(",\"%s\":[", KEYS[ch]);
        for (int i = 0; i < gGraphCount; i++)
            APPEND("%s%.3f", i?",":"", gGraph[ch][i]);
        APPEND("]");
    }
    APPEND(",\"count\":%d}", gGraphCount);
    FLUSH();
    gServer.sendContent("");

    #undef FLUSH
    #undef APPEND
}


// ============================================================
// SETUP
// ============================================================
void setup() {
    LOG.begin(115200);
//    delay(500);
    pinMode(HEARTBEAT_LED_PIN, OUTPUT);
    digitalWrite(HEARTBEAT_LED_PIN, HIGH);
    LOG.printf("\n\n%s Starting....\n", ota_build_filename().c_str());

    // SPIFFS
    if (!SPIFFS.begin(true)) {
        LOG.println("SPIFFS: mount failed");
    } else {
        LOG.printf("SPIFFS: mounted (free %lu bytes)\n",
                      (unsigned long)(SPIFFS.totalBytes() - SPIFFS.usedBytes()));
        spiffs_load_totals();
    }

    // BL0906
    Serial1.begin(BL0906_BAUD, SERIAL_8N1, BL0906_RX_PIN, BL0906_TX_PIN);
    delay(200);
    bl0906_init();

    // WiFi - 15 second timeout, reboot on failure (clears transient WiFi issues)
    LOG.printf("WiFi: connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500); LOG.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        LOG.printf("\nWiFi: connected  IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
        LOG.println("\nWiFi: failed - rebooting in 3s");
        delay(3000);
        ESP.restart();
    }

    // NTP
    configTime(TZ_OFFSET_SEC, TZ_DST_SEC, NTP_SERVER1, NTP_SERVER2);
    LOG.print("NTP: syncing");
    for (int i = 0; i < 20; i++) {
        struct tm t;
        if (get_local_time(t)) {
            gTimeValid = true;
            LOG.printf(" -> %04d-%02d-%02d %02d:%02d:%02d\n",
                t.tm_year+1900, t.tm_mon+1, t.tm_mday,
                t.tm_hour, t.tm_min, t.tm_sec);
            break;
        }
        delay(500); LOG.print(".");
    }
    if (!gTimeValid) LOG.println(" failed (will retry via loop)");

    // ArduinoOTA - only starts if WiFi is up
#if ARDUINO_OTA_ENABLED
    if (WiFi.status() == WL_CONNECTED) {
        ota_setup_arduino();
    } else {
        LOG.println("ArduinoOTA: skipped (no WiFi)");
    }
#endif

    // Web server routes
    gServer.on("/",           HTTP_GET,  handle_root);
    gServer.on("/api/live",   HTTP_GET,  handle_api_live);
    gServer.on("/api/graph",  HTTP_GET,  handle_api_graph);
    gServer.on("/api/totals", HTTP_GET,  handle_api_totals);
    gServer.on("/api/reboot", HTTP_POST, handle_api_reboot);
    gServer.begin();
    LOG.println("HTTP: server started on port 80");

    LOG.printf("HTTP_OTA_ENABLED=%d - running boot OTA check\n", HTTP_OTA_ENABLED);
#if HTTP_OTA_ENABLED
    ota_check_http();
#endif

    gLastSaveMs   = millis();
    gLastSecondMs = millis();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // ArduinoOTA must be polled every loop iteration to detect incoming push

OTAonly:
  LOG.handle();
  while (LOG.available()) {
    uint8_t gc = LOG.read();
    if (gc == 1) {  // Did client send CTRL-A [ENTER] (Update)?
      LOG.println("REBOOTING NOW...");
      LOG.handle();
      vTaskDelay(1000);
      ESP.restart();
    } else if (gc == 'P') {
      LOG.println("Forcing Pump Run!");
    }
  }


#if ARDUINO_OTA_ENABLED
    ArduinoOTA.handle();
    if (gOtaInProgress) {
        goto OTAonly;
    }
#endif

    gServer.handleClient();

    unsigned long now = millis();

//    digitalWrite(HEARTBEAT_LED_PIN, now & 0x40);

    // ---- 1-second sensor + energy tick ----
    if (now - gLastSecondMs >= 1000UL) {
        gLastSecondMs = now;
        digitalWrite(HEARTBEAT_LED_PIN, 1);

        if (bl0906_read_all()) {
            gAccum[CH_HOUSE]   += gLiveW[CH_HOUSE];
            gAccum[CH_LAB]     += gLiveW[CH_LAB];
            gAccum[CH_EV]      += gLiveW[CH_EV];
            gAccum[CH_TOTAL]   += gLiveW[CH_TOTAL];
            gAccum[CH_VOLTAGE] += gLiveV;
            gAccumCount++;
            accumulate_energy(gLiveW[CH_HOUSE], gLiveW[CH_LAB],
                              gLiveW[CH_EV],    gLiveW[CH_TOTAL]);
        }

        digitalWrite(HEARTBEAT_LED_PIN, 0);
        struct tm t;
        if (get_local_time(t)) {
            if (!gTimeValid) {
                gTimeValid = true;
                LOG.println("NTP: acquired");
            }
            check_rollover(t);
            int cur_30s = t.tm_hour * 120 + t.tm_min * 2 + t.tm_sec / 30;
            if (cur_30s != gLastGraph30s && gAccumCount > 0) {
                commit_graph_point(t);
                gLastGraph30s = cur_30s;
            }

            // Midnight reboot - triggers OTA check on next boot
            if (t.tm_hour == 0 && t.tm_min == 0 && !gMidnightRebooted) {
                gMidnightRebooted = true;
                LOG.println("Midnight reboot");
                spiffs_save_totals();
                delay(500);
                ESP.restart();
            }
            if (t.tm_hour != 0) gMidnightRebooted = false;
        }
    }

    // ---- Hourly SPIFFS save ----
    if (now - gLastSaveMs >= SPIFFS_SAVE_INTERVAL_MS) {
        gLastSaveMs = now;
        spiffs_save_totals();
    }
    
    yield();
}
