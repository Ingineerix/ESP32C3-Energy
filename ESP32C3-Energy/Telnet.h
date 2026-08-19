#ifndef Telnet_h
#define Telnet_h

#define Telnet_BUFFER_LEN 3000
#define Telnet_MIN_BLOCK_SIZE 64
#define Telnet_COLLECTING_TIME 100
#define Telnet_MAX_BLOCK_SIZE 512
#define Telnet_PING_TIME 1500
#define Telnet_PORT 23
#define Telnet_CAPTURE_OS_PRINT true
#define Telnet_REJECT_MSG "Telnet: Only one connection possible.\r\n"

#include <WiFi.h>
// add spinlock for ESP32
#define CRITCAL_SECTION_MUTEX portMUX_TYPE AtomicMutex = portMUX_INITIALIZER_UNLOCKED;
// Non-static Data Member Initializers, see: https://web.archive.org/web/20160316174223/https://blogs.oracle.com/pcarlini/entry/c_11_tidbits_non_static
#define CRITCAL_SECTION_START portENTER_CRITICAL(&AtomicMutex);
#define CRITCAL_SECTION_END portEXIT_CRITICAL(&AtomicMutex);
#include <WiFiClient.h>

extern char build[40];

class Telnet : public Stream {
	public:
		Telnet();
		~Telnet();
		void handle(void);
		void setPort(uint16_t portToUse);
		void setMinBlockSize(uint16_t minSize);
		void setCollectingTime(uint16_t colTime);
		void setMaxBlockSize(uint16_t maxSize);
		bool setBufferSize(uint16_t newSize);
		uint16_t getBufferSize();
		void setStoreOffline(bool store);
		bool getStoreOffline();
		void setPingTime(uint16_t pngTime);
		void setSerial(HardwareSerial* usedSerial);
		bool isClientConnected();
		void setCallbackOnConnect(void (*callback)());
		void setCallbackOnDisconnect(void (*callback)());
		// Functions offered by HardwareSerial class:
		void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false);
		void end();
		int available(void) override;
		int peek(void) override;
		int read(void) override;
		int availableForWrite(void);
		void flush(void) override;
		size_t write(uint8_t) override;
		inline size_t write(unsigned long n) { return write((uint8_t) n); }
		inline size_t write(long n) { return write((uint8_t) n); }
		inline size_t write(unsigned int n) { return write((uint8_t) n); }
		inline size_t write(int n) { return write((uint8_t) n); }
		using Print::write;
		operator bool() const;
		void setDebugOutput(bool);
		uint32_t baudRate(void);

	protected:
		CRITCAL_SECTION_MUTEX
		void sendBlock(void);
		void addTelnetBuf(char c);
		char pullTelnetBuf();
		char peekTelnetBuf();
		int telnetAvailable();
		WiFiServer* telnetServer;
		WiFiClient client;
		uint16_t port;
		HardwareSerial* usedSer;
		bool storeOffline;
		bool started;
		bool listening;
		bool firstMainLoop;
		unsigned long waitRef;
		unsigned long pingRef;
		uint16_t pingTime;
		char* welcomeMsg;
		char* rejectMsg;
		uint16_t minBlockSize;
		uint16_t collectingTime;
		uint16_t maxBlockSize;
		bool debugOutput;
		char* telnetBuf;
		uint16_t bufLen;
		uint16_t bufUsed;
		uint16_t bufRdIdx;
		uint16_t bufWrIdx;
		bool connected;
		void (*callbackConnect)();
		void (*callbackDisconnect)();
};

#endif

