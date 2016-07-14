#ifndef __GATEWAYUTIL_H__
#define __GATEWAYUTIL_H__

#ifdef ARDUINO

static uint8_t inclusionTime = 1; // Number of minutes inclusion mode is enabled
static uint8_t pinInclusion =  3; // Input pin that should trigger inclusion mode

#define MAX_RECEIVE_LENGTH 100 // Max buffersize needed for messages coming from controller
#define MAX_SEND_LENGTH 120 // Max buffersize needed for messages destined for controller

// Added by K. OSTER for OLED IIC
#define OLED_WIFI_REFRESH_S  5000
#define OLED_BLINK_TIME_MS   300
unsigned long oledRefreshWifiTimeref = 0;
unsigned long oledBlinkTimerefTx = 0;
unsigned long oledBlinkTimerefRx = 0;
unsigned long oledBlinkTimerefErr = 0;
bool oledBlinkTxOn = false;
bool oledBlinkRxOn = false;
bool oledBlinkErrOn = false;

static const unsigned char PROGMEM bmp_arrowUp[] =
{ B00010000, 
  B00111000, 
  B01111100, 
  B11111110, 
  B00111000, 
  B00111000, 
  B00111000, 
  B00111000};
static const unsigned char PROGMEM bmp_arrowDown[] =
{ 
  B00111000, 
  B00111000, 
  B00111000, 
  B00111000,
  B11111110, 
  B01111100, 
  B00111000, 
  B00010000};
static const unsigned char PROGMEM bmp_error[] =
{ 
  B01000010, 
  B11100111, 
  B01111110, 
  B00111100, 
  B00111100, 
  B01111110, 
  B11100111, 
  B01000010};
static const unsigned char PROGMEM bmp_inclusion[] =
{ 
  B00011000, 
  B00011000, 
  B00011000, 
  B11111111, 
  B11111111, 
  B00011000, 
  B00011000, 
  B00011000, 
  };
//
Adafruit_SSD1306 display(LED_BUILTIN);
void oled_blink(const uint8_t *bitmap, uint8_t x, uint8_t y, uint8_t color) {
  display.drawBitmap(x, y, bitmap, 8, 8, color);
  display.display();
}
void oled_txOn()    { oled_blink(bmp_arrowUp,         0, 9, WHITE); oledBlinkTxOn = true; oledBlinkTimerefTx = millis(); }
void oled_txOff()   { oled_blink(bmp_arrowUp,         0, 9, BLACK); oledBlinkTxOn = false; }
void oled_rxOn()    { oled_blink(bmp_arrowDown,       9, 9, WHITE); oledBlinkRxOn = true; oledBlinkTimerefRx = millis(); }
void oled_rxOff()   { oled_blink(bmp_arrowDown,       9, 9, BLACK); oledBlinkRxOn = false; }
void oled_errOn()   { oled_blink(bmp_error,           18, 9, WHITE); oledBlinkErrOn = true; oledBlinkTimerefErr = millis(); }
void oled_errOff()  { oled_blink(bmp_error,           18, 9, BLACK); oledBlinkErrOn = false; }
void oled_inclusionOn()   { oled_blink(bmp_inclusion, 27, 9, WHITE); }
void oled_inclusionOff()  { oled_blink(bmp_inclusion, 27, 9, BLACK); }

typedef struct
{
  char    string[MAX_RECEIVE_LENGTH];
  uint8_t idx;
} inputBuffer;

static volatile boolean buttonTriggeredInclusion;
static boolean inclusionMode; // Keeps track on inclusion mode
bool inclusionButtonSupported = false;
void (*serial)(const char *fmt, ... );

MyParserSerial parser;

void setInclusionMode(boolean newMode);

char convBuf[MAX_PAYLOAD*2+1];
unsigned long inclusionStartTime;


void startInclusionInterrupt() {
  buttonTriggeredInclusion = true;
  oled_inclusionOn();
}

void setupGateway(uint8_t _inc, uint8_t _incTime, void (* _serial)(const char *, ... )) {
  inclusionMode = 0;
  buttonTriggeredInclusion = false;
  serial = _serial;

  inclusionTime = _incTime;
  inclusionButtonSupported = (_inc != 255);
  if (inclusionButtonSupported)
  {
    pinInclusion = _inc;

    // Setup digital in that triggers inclusion mode
    pinMode(pinInclusion, INPUT_PULLUP);

    // Add interrupt for inclusion button to pin
    attachInterrupt(pinInclusion, startInclusionInterrupt, FALLING);
  }
}

void incomingMessage(const MyMessage &message) {
//  if (mGetCommand(message) == C_PRESENTATION && inclusionMode) {
//	gw.rxBlink(3);
//   } else {
//	gw.rxBlink(1);
//   }
   // Pass along the message from sensors to serial line
   oled_rxOn();
   serial(PSTR("%d;%d;%d;%d;%d;%s\n"),message.sender, message.sensor, mGetCommand(message), mGetAck(message), message.type, message.getString(convBuf));
   
} 



void checkButtonTriggeredInclusion() {
   if (inclusionButtonSupported)
   {
     if (buttonTriggeredInclusion) {
      // Ok, someone pressed the inclusion button on the gateway
      // start inclusion mode for 1 munute.
  #ifdef DEBUG
      serial(PSTR("0;0;%d;0;%d;Inclusion started by button.\n"),  C_INTERNAL, I_LOG_MESSAGE);
  #endif
      buttonTriggeredInclusion = false;
      
      setInclusionMode(true);
      
    }
  }
}

void checkInclusionFinished() {
  if (inclusionMode && millis()-inclusionStartTime>60000UL*inclusionTime) {
    // inclusionTimeInMinutes minute(s) has passed.. stop inclusion mode
    setInclusionMode(false);
    oled_inclusionOff();
  }
}

void parseAndSend(MySensor &gw, char *commandBuffer) {
  boolean ok;
  MyMessage &msg = gw.getLastMessage();

  if (parser.parse(msg, commandBuffer)) {
    uint8_t command = mGetCommand(msg);
    if (msg.destination==GATEWAY_ADDRESS && command==C_INTERNAL) {
      oled_rxOn();
      // Handle messages directed to gateway
      if (msg.type == I_VERSION) {
        // Request for version
        Serial.println("Requested version by controller");
        serial(PSTR("0;0;%d;0;%d;%s\n"), C_INTERNAL, I_VERSION, LIBRARY_VERSION);
      } else if (msg.type == I_INCLUSION_MODE) {
        // Request to change inclusion mode
        Serial.println("Requested to enable inclusion mode by controller");
        setInclusionMode(atoi(msg.data) == 1);
      }
    } else {
      oled_txOn();
      #ifdef WITH_LEDS_BLINKING
      gw.txBlink(1);
      #endif
      ok = gw.sendRoute(msg);
      if (!ok) {
        #ifdef WITH_LEDS_BLINKING
        gw.errBlink(1);
        #endif
        oled_errOn();
      }
    }
  }
}

void setInclusionMode(boolean newMode) {
  if (newMode != inclusionMode) {
    inclusionMode = newMode;
    // Send back mode change on serial line to ack command
    serial(PSTR("0;0;%d;0;%d;%d\n"), C_INTERNAL, I_INCLUSION_MODE, inclusionMode?1:0);

    if (inclusionMode) {
      inclusionStartTime = millis();
    }
  }
}


#else
#error This example is only for use on Arduino.
#endif // ARDUINO

#endif 
