// after mobile adapter (re)start (HW or SW):
// <"garbage">
// RDY                    
//                                     
// OK  // only on SW reset                                                         
//
// +CFUN: 1                                                    
//                                                      
// +CPIN: SIM PIN

#include <EEPROM.h>

#include <TinyGPS++.h>
#include <AltSoftSerial.h>

AltSoftSerial altSerial;

TinyGPSPlus gps;

const char * receivedSmsPrefix = "+CMT: \"+421908363848\"";

const int resetPin = 6;
const int activityIndicationPin = 3;
const int gpsStatusIndicationPin = 13;

const char * initCommands[] = {
  "ATE0", // no echo
  "AT+CPIN=\"0000\"", // PIN
  "AT+CMGF=1", // TEXT SMS mode
  "AT+CNMI=2,2,0,0,0", // dump SMS on receive
  "AT+CLIP=1" // CLIP on
};
int initCommand; // pointer to current command in initCommands

typedef enum { ST_BOOTING, ST_IDLE, ST_WAIT_FOR_RESPONSE, ST_SMS_READ_TEXT, ST_SMS_WAIT_FOR_PROMPT } state_t;
state_t state;

typedef enum { MACS_IDLE, MACS_IN_PROGRESS, MACS_OK, MACS_ERROR } ma_checking_state_t;
ma_checking_state_t maCheckingState;

char line[256]; // buffer for reading data from mobile adapter
unsigned short p; // pointer to the character in the line

char smsText[161];

unsigned long lastCommandInitTime;
unsigned long resetStartTime;
unsigned long maLastCheckingTime;

unsigned int meters;
double refLat, refLng;

void setup() {
  Serial.begin(9600);
  altSerial.begin(9600);
  
  pinMode(activityIndicationPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(gpsStatusIndicationPin, OUTPUT);

  readConfiguration();

  resetMobileAdapter();
}

void readConfiguration() {
  int addr = 0;
  EEPROM.get(addr, meters);
  addr += sizeof(int);
  EEPROM.get(addr, refLat);
  addr += sizeof(double);
  EEPROM.get(addr, refLng);  
}

void writeConfiguration() {
  int addr = 0;
  EEPROM.put(addr, meters);
  addr += sizeof(int);
  EEPROM.put(addr, refLat);
  addr += sizeof(double);
  EEPROM.put(addr, refLng);
}

void resetMobileAdapter() {
  digitalWrite(resetPin, LOW); // reset
  
  unsigned long now = millis();
  resetStartTime = now;
  maLastCheckingTime = now;
  lastCommandInitTime = now;
  initCommand = 0;
  smsText[0] = '\0';
  state = ST_BOOTING;
  maCheckingState = MACS_IDLE;
  line[0] = '\0';
  p = 0;
}

bool isInitializing() {
  return initCommand < sizeof(initCommands) / sizeof(initCommands[0]);
}

void loop() {
  unsigned long now = millis();

  digitalWrite(activityIndicationPin, state == ST_IDLE ? LOW : HIGH);
 
  if (state == ST_BOOTING && now - resetStartTime > 100) {
    digitalWrite(resetPin, HIGH);
  }
  
  if (state == ST_IDLE) {
    if (isInitializing()) {
      altSerial.println(initCommands[initCommand]);
      state = ST_WAIT_FOR_RESPONSE;
    } else if (smsText[0]) {
      altSerial.println("AT+CMGS=\"+421908363848\""); // begin sending SMS to the MS
      state = ST_SMS_WAIT_FOR_PROMPT;
    } else if (now - maLastCheckingTime > 60000) { // one minute after last check
      maLastCheckingTime = now;
      altSerial.println("AT+CPAS"); // query mobile adapter state
      maCheckingState = MACS_IN_PROGRESS;
      state = ST_WAIT_FOR_RESPONSE;
    } else if (maCheckingState == MACS_ERROR) {
      resetMobileAdapter();
    }
    lastCommandInitTime = now;
  } else if (now - lastCommandInitTime > 60000) { // command timed out after one minute
    resetMobileAdapter();
    return;
  }
  
  if (altSerial.available()) {
    char c = altSerial.read();
    
    if (c == '\r') {
      // ingore
    } else if (c == '\n') {
      switch (state) {
        case ST_BOOTING:
          if (!strcmp(line, "RDY")) {
            state = ST_IDLE;
          }
          break;
        case ST_SMS_READ_TEXT:
          state = ST_IDLE;
          
          if (!strcasecmp(line, "locate")) {
            prepareSmsWithLocation();
          } else if (!strncasecmp(line, "guard ", 6)) {
            if (gps.location.isValid()) {
              meters = atoi(line + 6);
              refLat = gps.location.lat();
              refLng = gps.location.lng();

              writeConfiguration();
            } else {
              strcpy(smsText, "No fix.");
            }
          } else {
              strcpy(smsText, "Unknown command.");            
          }
          
          break;
        case ST_WAIT_FOR_RESPONSE:
          if (!strcmp(line, "OK") || !strcmp(line, "ERROR")) {
            state = ST_IDLE;
            if (isInitializing()) {
              initCommand++;
            }
            maCheckingState = maCheckingState == MACS_IN_PROGRESS ? MACS_ERROR : MACS_IDLE;
          } else if (maCheckingState != ST_IDLE && (!strcmp(line, "+CPAS: 0") || !strcmp(line, "+CPAS: 3"))) { // mobile adapter ready
            maCheckingState = MACS_OK;
          }
          break;
        case ST_IDLE:
          if (!strncmp(receivedSmsPrefix, line, strlen(receivedSmsPrefix))) {
            // received sms, read next line
            state = ST_SMS_READ_TEXT;
          } else if (!strcmp("+CLIP: \"+421908363848\",145,\"\",,\"\",0", line)) {
            altSerial.println("ATH"); // hang up
            state = ST_WAIT_FOR_RESPONSE;
            prepareSmsWithLocation();
          }
          break;
        default:
          // do nothing
          break;
      }

      line[p = 0] = '\0';
    } else if (state == ST_SMS_WAIT_FOR_PROMPT && !strcmp(line, ">")) {
      altSerial.print(smsText);
      altSerial.write(0x1A);
      smsText[0] = '\0';
      state = ST_WAIT_FOR_RESPONSE;
    } else if (p < sizeof(line) - 1) {
      line[p++] = c;
      line[p] = '\0';
    }
  }

  processGps();
}

void processGps() {
  if (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  int hdop = gps.hdop.value();
  if (hdop == 0) {
    hdop = 10000;
  }
  
  unsigned long now = millis();
  digitalWrite(gpsStatusIndicationPin, now % 100 > 50 && (now / 100) % 50 < (hdop / 100.0 - 0.95) * 10);

  int distance;
  if (meters && gps.location.isValid() && (distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), refLat, refLng)) > meters) {
    meters = 0; // disable guard; TODO disable it only after the SMS has been sent successfully
    writeConfiguration();
    snprintf(smsText, 161, "Out of guard. Distance: %d m", distance);
  }
}

void prepareSmsWithLocation() {
  char latStr[11], lngStr[11];
  
  dtostrf(gps.location.lat(), 9, 6, latStr);
  dtostrf(gps.location.lng(), 9, 6, lngStr);
  
  snprintf(smsText, 161, "%0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d @ %s %s, %d mnm, %d km/h, %d deg, HDOP: %d, sats: %d",
    gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second(),
    latStr, lngStr, (int) gps.altitude.meters(), (int) gps.speed.kmph(), (int) gps.course.deg(), gps.hdop.value(), gps.satellites.value());
}

