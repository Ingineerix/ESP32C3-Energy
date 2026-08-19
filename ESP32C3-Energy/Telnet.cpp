#include "Telnet.h"

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

static Telnet* actualObject = NULL;


static void Telnet_putc(char c) {
	if (actualObject) {
  		actualObject->write(c);
	}
}

#ifdef TELNET_IGNORE_PUTC
static void Telnet_ignore_putc(char c) {;
}
#endif

Telnet::Telnet() {
	port = Telnet_PORT;
	telnetServer = NULL;
	started = false;
	listening = false;
	firstMainLoop = true;
	usedSer = &Serial;
	storeOffline = true;
	connected = false;
	callbackConnect = NULL;
	callbackDisconnect = NULL;
	rejectMsg = strdup(Telnet_REJECT_MSG);
	minBlockSize = Telnet_MIN_BLOCK_SIZE;
	collectingTime = Telnet_COLLECTING_TIME;
	maxBlockSize = Telnet_MAX_BLOCK_SIZE;
	pingTime = Telnet_PING_TIME;
	pingRef = 0xFFFFFFFF;
	waitRef = 0xFFFFFFFF;
	telnetBuf = NULL;
	bufLen = 0;
	uint16_t size = Telnet_BUFFER_LEN;
	while (!setBufferSize(size)) {
		size = size >> 1;
		if (size < minBlockSize) {
			setBufferSize(minBlockSize);
			break;
		}
	}
	debugOutput = Telnet_CAPTURE_OS_PRINT;
	if (debugOutput) {
		setDebugOutput(true);
	}
}

Telnet::~Telnet() {
	end();
}

void Telnet::setPort(uint16_t portToUse) {
	port = portToUse;
	if (listening) {
		if (client.connected()) {
			client.flush();
			client.stop();
		}
		if (connected && (callbackDisconnect != NULL)) {
			callbackDisconnect();
		}
		connected = false;
		telnetServer->close();
		delete telnetServer;
		telnetServer = new WiFiServer(port);
		if (started) {
			telnetServer->begin();
			telnetServer->setNoDelay(bufLen > 0);
		}
	}
}

void Telnet::setMinBlockSize(uint16_t minSize) {
	minBlockSize = min(max((uint16_t) 1, minSize), maxBlockSize);
}

void Telnet::setCollectingTime(uint16_t colTime) {
	collectingTime = colTime;
}

void Telnet::setMaxBlockSize(uint16_t maxSize) {
	maxBlockSize = max(maxSize, minBlockSize);
}

bool Telnet::setBufferSize(uint16_t newSize) {
	if (telnetBuf && (bufLen == newSize)) {
		return true;
	}
	if (newSize == 0) {
		bufLen = 0;
		if (telnetBuf) {
			free(telnetBuf);
			telnetBuf = NULL;
		}
		if (telnetServer) {
			telnetServer->setNoDelay(false);
		}
		return true;
	}
	newSize = max(newSize, minBlockSize);
	uint16_t oldBufLen = bufLen;
	bufLen = newSize;
	uint16_t tmp;
	if (!telnetBuf || (bufUsed == 0)) {
		bufRdIdx = 0;
		bufWrIdx = 0;
		bufUsed = 0;
	} else {
		if (bufLen < oldBufLen) {
			if (bufRdIdx < bufWrIdx) {
				if (bufWrIdx > bufLen) {
					tmp = min(bufLen, (uint16_t) (bufWrIdx - max(bufLen, bufRdIdx)));
					memcpy(telnetBuf, &telnetBuf[bufWrIdx - tmp], tmp);
					bufWrIdx = tmp;
					if (bufWrIdx > bufRdIdx) {
						bufRdIdx = bufWrIdx;
					} else {
						if (bufRdIdx > bufLen) {
							bufRdIdx = 0;
						}
					}
					if (bufRdIdx == bufWrIdx) {
						bufUsed = bufLen;
					} else {
						bufUsed = bufWrIdx - bufRdIdx;
					}
				}
			} else {
				if (bufWrIdx > bufLen) {
					memcpy(telnetBuf, &telnetBuf[bufWrIdx - bufLen], bufLen);
					bufRdIdx = 0;
					bufWrIdx = 0;
					bufUsed = bufLen;
				} else {
					tmp = min(bufLen - bufWrIdx, oldBufLen - bufRdIdx);
					memcpy(&telnetBuf[bufLen - tmp], &telnetBuf[oldBufLen - tmp], tmp);
					bufRdIdx = bufLen - tmp;
					bufUsed = bufWrIdx + tmp;
				}
			}
		}
	}
	char* temp = (char*) realloc(telnetBuf, bufLen);
	if (!temp) {
		return false;
	}
	telnetBuf = temp;
	if (telnetBuf && (bufLen > oldBufLen) && (bufRdIdx > bufWrIdx)) {
		tmp = bufLen - (oldBufLen - bufRdIdx);
		memcpy(&telnetBuf[tmp], &telnetBuf[bufRdIdx], oldBufLen - bufRdIdx);
		bufRdIdx = tmp;
	}
	if (telnetServer) {
		telnetServer->setNoDelay(true);
	}
	return true;
}

uint16_t Telnet::getBufferSize() {
	if (!telnetBuf) {
		return 0;
	}
	return bufLen;
}

void Telnet::setStoreOffline(bool store) {
	storeOffline = store;
}

bool Telnet::getStoreOffline() {
	return storeOffline;
}

void Telnet::setPingTime(uint16_t pngTime) {
	pingTime = pngTime;
	if (pingTime == 0) {
		pingRef = 0xFFFFFFFF;
	} else {
		pingRef = (millis() & 0x7FFFFFF) + pingTime;
	}
}

void Telnet::setSerial(HardwareSerial* usedSerial) {
	usedSer = usedSerial;
}

size_t Telnet::write (uint8_t data) {
	if (telnetBuf) {
		if (storeOffline || client.connected()) {
			if (bufUsed == bufLen) {
				if (client.connected()) {
					sendBlock();
				}
				if (bufUsed == bufLen) {
					char c;
					while (bufUsed > 0) {
						c = pullTelnetBuf();
						if (c == '\n') {
							break;
						}
					}
					if (peekTelnetBuf() == '\r') {
						pullTelnetBuf();
					}
				}
			}
			addTelnetBuf(data);
		}
	} else {
		if (client.connected()) {
			client.write(data);
		}
	}
	if (usedSer) {
		return usedSer->write(data);
	}
	return 1;
}

int Telnet::available (void) {
	if (usedSer) {
		int avail = usedSer->available();
		if (avail > 0) {
			return avail;
		}
	}
	if (client.connected()) {
		return telnetAvailable();
	}
	return 0;
}

int Telnet::read (void) {
	int val;
	if (usedSer) {
		val = usedSer->read();
		if (val != -1) {
			return val;
		}
	}
	if (client.connected()) {
		if (telnetAvailable()) {
			val = client.read();
			return val;
		}
	}
	return -1;
}

int Telnet::peek (void) {
	int val;
	if (usedSer) {
		val = usedSer->peek();
		if (val != -1) {
			return val;
		}
	}
	if (client.connected()) {
		if (telnetAvailable()) {
			val = client.peek();
		}
	}
	return -1;
}

void Telnet::flush (void) {
	if (usedSer) {
		usedSer->flush();
	}
}

void Telnet::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert) {
	if (usedSer) {
		usedSer->begin(baud, config, rxPin, txPin, invert);
	}
	started = true;
}

void Telnet::end() {
	if (debugOutput) {
		setDebugOutput(false);
	}
	if (usedSer) {
		usedSer->end();
	}
	if (client.connected()) {
		client.flush();
		client.stop();
	}
	if (connected && (callbackDisconnect != NULL)) {
		callbackDisconnect();
	}
	connected = false;
	telnetServer->close();
	delete telnetServer;
	telnetServer = NULL;
	listening = false;
	started = false;
}

int Telnet::availableForWrite(void) {
	if (usedSer) {
		return min(usedSer->availableForWrite(), bufLen - bufUsed);
	}
	return bufLen - bufUsed;
}

Telnet::operator bool() const {
	if (usedSer) {
		return (bool) *usedSer;
	}
	return true;
}

void Telnet::setDebugOutput(bool en) {
	debugOutput = en;
	if (debugOutput) {
		actualObject = this;
	} else {
		if (actualObject == this) {
			actualObject = NULL;
		}
	}
}

uint32_t Telnet::baudRate(void) {
	if (usedSer) {
		return usedSer->baudRate();
	}
	return 921600;
}

void Telnet::sendBlock() {
CRITCAL_SECTION_START
	uint16_t len = bufUsed;
	if (len > maxBlockSize) {
		len = maxBlockSize;
	}
	len = min(len, (uint16_t) (bufLen - bufRdIdx));
	uint16_t idx = bufRdIdx;
CRITCAL_SECTION_END
	client.write(&telnetBuf[idx], len);
CRITCAL_SECTION_START
	bufRdIdx += len;
	if (bufRdIdx >= bufLen) {
		bufRdIdx = 0;
	}
	bufUsed -= len;
	if (bufUsed == 0) {
		bufRdIdx = 0;
		bufWrIdx = 0;
	}
CRITCAL_SECTION_END
	waitRef = 0xFFFFFFFF;
	if (pingRef != 0xFFFFFFFF) {
		pingRef = (millis() & 0x7FFFFFF) + pingTime;
		if (pingRef > 0x7FFFFFFF) {
			pingRef -= 0x80000000;
		}
	}
}

void Telnet::addTelnetBuf(char c) {
CRITCAL_SECTION_START
	telnetBuf[bufWrIdx] = c;
	if (bufUsed == bufLen) {
		bufRdIdx++;
		if (bufRdIdx >= bufLen) {
			bufRdIdx = 0;
		}
	} else {
		bufUsed++;
	}
	bufWrIdx++;
	if (bufWrIdx >= bufLen) {
		bufWrIdx = 0;
	}
CRITCAL_SECTION_END
}

char Telnet::pullTelnetBuf() {
	if (bufUsed == 0) {
		return 0;
	}
CRITCAL_SECTION_START
	char c = telnetBuf[bufRdIdx++];
	if (bufRdIdx >= bufLen) {
		bufRdIdx = 0;
	}
	bufUsed--;
CRITCAL_SECTION_END
	return c;
}

char Telnet::peekTelnetBuf() {
	if (bufUsed == 0) {
		return 0;
	}
CRITCAL_SECTION_START
char c = telnetBuf[bufRdIdx]; 
CRITCAL_SECTION_END
return c;
}

int Telnet::telnetAvailable() {
	int n = client.available();
	while (n > 0) {
		if (0xff == client.peek()) {  // If esc char for telnet NVT protocol data remove that telegram:
			client.read();  // Remove esc char
			n--;
			if (0xff == client.peek()) {  // If esc sequence for 0xFF data byte...
				return n; // ...return info about available data (just this 0xFF data byte)
			}
			client.read();  // Skip the rest of the telegram of the telnet NVT protocol data
			client.read();
			n--;
			n--;
		} else {  // If next char is a normal data byte...
			return n;   // ...return info about available data
		}
	}
	return 0;
}

bool Telnet::isClientConnected() {
	return connected;
}

void Telnet::setCallbackOnConnect(void (*callback)()) {
	callbackConnect = callback;
}

void Telnet::setCallbackOnDisconnect(void (*callback)()) {
	callbackDisconnect = callback;
}

void Telnet::handle() {
	if (firstMainLoop) {
		firstMainLoop = false;
		if (debugOutput && (actualObject == this)) {
			setDebugOutput(true);
		}
	}
	if (!started) {
		return;
	}
	if (!listening) {
		if (WiFi.status() != WL_CONNECTED) {
			return;
		}
		telnetServer = new WiFiServer(port);
		telnetServer->begin();
		telnetServer->setNoDelay(bufLen > 0);
		listening = true;
	}
    if (telnetServer->hasClient()) {
        if (client.connected()) {
            WiFiClient rejectClient = telnetServer->available();
			if (strlen(rejectMsg) > 0) {
				rejectClient.write((const uint8_t*) rejectMsg, strlen(rejectMsg));
			}
			rejectClient.flush();
            rejectClient.stop();
        } else {
            client = telnetServer->available();
//	    client.write((uint8_t*) build, strlen(build));
	    client.write("\r\n",2);
        }
    }
    if (client.connected()) {
    	if (!connected) {
    		connected = true;
    		if (pingTime != 0) {
    			pingRef = (millis() & 0x7FFFFFF) + pingTime;
    		}
			if (callbackConnect != NULL) {
				callbackConnect();
			}
		}
	} else {
    	if (connected) {
    		connected = false;
        	client.flush();
            client.stop();
			pingRef = 0xFFFFFFFF;
			waitRef = 0xFFFFFFFF;
			if (callbackDisconnect != NULL) {
				callbackDisconnect();
			}
		}
	}

	if (client.connected() && (bufUsed > 0)) {
		if (bufUsed >= minBlockSize) {
			sendBlock();
		} else {
			unsigned long m = millis() & 0x7FFFFFF;
			if (waitRef == 0xFFFFFFFF) {
				waitRef = m + collectingTime;
				if (waitRef > 0x7FFFFFFF) {
					waitRef -= 0x80000000;
				}
			} else {
				if (!((waitRef < 0x20000000) && (m > 0x60000000)) && (m >= waitRef)) {
					sendBlock();
				}
			}
		}
	}
	if (client.connected() && (pingRef != 0xFFFFFFFF)) {
		unsigned long m = millis() & 0x7FFFFFF;
		if (!((pingRef < 0x20000000) && (m > 0x60000000)) && (m >= pingRef)) {
			addTelnetBuf(0);
			sendBlock();
		}
	}
}
