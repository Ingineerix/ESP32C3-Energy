// ============================================================
// ESP32C3-Energy — User Configuration
// ============================================================

// --- WiFi ---
#define WIFI_SSID           "YOUR WIFI SSID"
#define WIFI_PASSWORD       "YOURPASS"

// --- NTP / Time ---
#define NTP_SERVER1         "us.pool.ntp.org"
#define NTP_SERVER2         "time.nist.gov"
// Offset from UTC in seconds: PST=-28800, MST=-25200, CST=-21600, EST=-18000
#define TZ_OFFSET_SEC       (-8 * 3600)
// DST offset in seconds (3600 for US DST regions, 0 to disable)
#define TZ_DST_SEC          3600

// --- BL0906 UART pins ---
#define BL0906_RX_PIN       8       // GPIO5 connects to BL0906 TX
#define BL0906_TX_PIN       7       // GPIO4 connects to BL0906 RX
#define BL0906_BAUD         19200

// --- CT wiring ---
// Set 2.0 if CT5 is on ONE leg of a 240V-only load (e.g. EV charger)
// Set 1.0 if both legs are measured
#define CT5_MULTIPLIER      2.0f

// --- Hardware ---
#define HEARTBEAT_LED_PIN   10  // LED "3" on the PCB

// --- ArduinoOTA (LAN push from Arduino IDE) ---
#define ARDUINO_OTA_ENABLED     1
#define ARDUINO_OTA_HOSTNAME    "ESP32C3-Energy"
#define ARDUINO_OTA_PASSWORD    "YOUR-OTA-PASSWORD"

// --- HTTP pull OTA ---
// Base URL of your update server (trailing slash required)
#define HTTP_OTA_ENABLED        1
#define OTA_SERVER_URL          "http://192.168.1.10/"

// --- Persistence ---
// Save kWh totals to SPIFFS this often (default: every hour)
#define SPIFFS_SAVE_INTERVAL_MS  (3600UL * 1000UL)
