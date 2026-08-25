/*
  =====================================================================
  SMART ELECTRONIC DOOR LOCK SYSTEM - ESP32   
  =====================================================================
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Adafruit_PN532.h>
#include <Adafruit_Fingerprint.h>
#include <RTClib.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>


// ---------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------
#define PIN_PN532_IRQ   18
#define PIN_PN532_RESET 19
#define PIN_LIMIT_SW    34
#define PIN_EXIT_BTN    35   
#define PIN_BATT_ADC    27   
#define PIN_SOLENOID    5    
#define PIN_BUZZER      15   
#define PIN_LED_SW_RED    23 
#define PIN_LED_SW_GREEN   2 

const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {13, 12, 14, 26};
byte colPins[COLS] = {25, 33, 32, 4};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ---------------------------------------------------------------------
// OBJECTS & FORWARD DECLARATIONS
// ---------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 4);
RTC_DS3231 rtc;
Adafruit_PN532 nfc(PIN_PN532_IRQ, PIN_PN532_RESET);
Preferences prefs;

HardwareSerial fingerSerial(2);
Adafruit_Fingerprint finger(&fingerSerial);

void notifyStatus(String msg);
void logIncident(String msg);
void grantAccess(String method, String name, String personId);
void denyAccess(String reason);
void unlockSolenoid();
void updateSolenoid();
void updateBuzzer();
void beepKeyClick();
void beepSuccess();
void beepAccessGranted();
void beepError();
void beepAlarm();
float readBatteryVoltage();
int batteryPercent();
void checkLowBattery();
void handleLimitSwitch();
void handleExitButton();
void handleLockoutTimer();
void updateFPEnroll();
void updateRFIDEnroll();
void handleKeypad();
void handleFingerprint();
void handleRFID();
void drawCurrentPage();
void updateIncidentDisplay();
void setupBLE();
String uidToString(uint8_t *uid, uint8_t length);
int findRFIDIndex(String uid);
bool isRFIDAuthorized(String uid);
void addRFIDEntry(String uid, String personId, String name);
void loadRFIDList();
void saveRFIDList();
void lcdLine(uint8_t row, String text);
String maskPin(String pin);

// ---------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define COMMAND_CHAR_UUID    "12345678-1234-1234-1234-1234567890ac"
#define STATUS_CHAR_UUID     "12345678-1234-1234-1234-1234567890ad"

BLEServer *pServer = nullptr;
BLECharacteristic *pStatusChar = nullptr;
bool bleDeviceConnected = false;

void notifyStatus(String msg) {
  Serial.println("[BLE STATUS] " + msg);
  if (bleDeviceConnected && pStatusChar) {
    pStatusChar->setValue(msg.c_str());
    pStatusChar->notify();
  }
}

// ---------------------------------------------------------------------
// CONSTANTS / TUNABLES
// ---------------------------------------------------------------------
const float BATT_DIVIDER_RATIO   = 0.32;
const float BATT_FULL_VOLTAGE    = 8.4;
const float BATT_EMPTY_VOLTAGE   = 6.0;
const unsigned long UNLOCK_MS    = 5000;
const uint8_t MAX_PIN_ATTEMPTS   = 3;
const unsigned long LOCKOUT_MS   = 30000;
const unsigned long INCIDENT_HOLD_MS = 2500;
const unsigned long WELCOME_HOLD_MS  = 4000;
const unsigned long LONG_PRESS_MS    = 700;
const unsigned long DEBOUNCE_MS      = 30;
const uint8_t WDT_TIMEOUT_S          = 5;
const String DEFAULT_PIN = "1234";
const int MAX_RFID = 20;
const int NUM_PAGES = 4;

// ---------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------
enum SystemState { STATE_IDLE, STATE_ENTER_PIN, STATE_CHANGE_PIN_OLD, STATE_CHANGE_PIN_NEW, STATE_CHANGE_PIN_CONFIRM };
SystemState state = STATE_IDLE;

String pinBuffer = "";
String newPinTemp = "";
String storedPin = "1234";
uint8_t failedAttempts = 0;
bool lockedOut = false;
unsigned long lockoutStart = 0;

uint8_t displayPage = 0;
bool showingIncident = false;
unsigned long incidentUntil = 0;
bool showingWelcome = false;
unsigned long welcomeUntil = 0;
String lastIncident = "System Ready";
bool lastMainsPresent = true;

String rfidUIDs[MAX_RFID];
String rfidNames[MAX_RFID];
String rfidIds[MAX_RFID];
int rfidCount = 0;

enum FPEnrollState { FP_IDLE, FP_WAIT_FIRST, FP_WAIT_REMOVE, FP_WAIT_SECOND };
FPEnrollState fpEnrollState = FP_IDLE;
int fpEnrollId = -1;
String fpEnrollName = "";

bool rfidEnrollActive = false;
String rfidEnrollName = "";
String rfidEnrollId = "";

bool solenoidOpen = false;
unsigned long solenoidOpenUntil = 0;

bool buzzerActive = false;
bool buzzerCurrentlyOn = false;
unsigned long buzzerStepStart = 0;
uint8_t buzzerStepIndex = 0;
uint8_t buzzerStepCount = 0;
uint16_t buzzerOnMs[8];
uint16_t buzzerOffMs[8];

byte lockIcon[8] = {
  B01110,
  B01001,
  B01001,
  B11111,
  B11011,
  B11011,
  B11111,
  B00000
};

// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  keypad.setDebounceTime(20);

  pinMode(PIN_LIMIT_SW, INPUT);
  pinMode(PIN_EXIT_BTN, INPUT);
  pinMode(PIN_SOLENOID, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_SW_RED, OUTPUT);
  pinMode(PIN_LED_SW_GREEN, OUTPUT);
  digitalWrite(PIN_SOLENOID, HIGH);
  digitalWrite(PIN_BUZZER, LOW);

  bool limitSwState = digitalRead(PIN_LIMIT_SW);
  digitalWrite(PIN_LED_SW_GREEN, limitSwState == HIGH ? HIGH : LOW);
  digitalWrite(PIN_LED_SW_RED,   limitSwState == LOW  ? HIGH : LOW);

  Wire.begin(21, 22);
  Wire.setTimeOut(50);

  esp_task_wdt_config_t twdtConfig = {
    .timeout_ms = (uint32_t)WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdtConfig);
  esp_task_wdt_add(NULL);
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(1, lockIcon);

  lcdLine(0, "SMART DOOR LOCK");

  for (int cycle = 0; cycle < 3; cycle++) {
    for (int dots = 1; dots <= 3; dots++) {
      String statusText = "Initializing";
      for (int i = 0; i < dots; i++) {
        statusText += ".";
      }
      lcdLine(2, statusText);
      delay(400);
    }
  }

  lcd.clear();

  if (!rtc.begin()) {
    lcd.setCursor(0, 2);
    lcd.print("RTC NOT FOUND!");
  }

  prefs.begin("doorlock", false);
  bool rtcSet = prefs.getBool("rtcset", false);
  if (!rtcSet) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    prefs.putBool("rtcset", true);
    Serial.println("RTC updated to compile time on first boot.");
  }
  storedPin = prefs.getString("pin", DEFAULT_PIN);
  prefs.end();

  fingerSerial.begin(57600, SERIAL_8N1, 16, 17);
  finger.verifyPassword();

  nfc.begin();
  uint32_t nfcVersion = nfc.getFirmwareVersion();
  if (!nfcVersion) {
    lcd.setCursor(0, 3);
    lcd.print("PN532 NOT FOUND!");
  } else {
    nfc.SAMConfig();
  }

  loadRFIDList();
  setupBLE();
}

// ---------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------
void loop() {
  esp_task_wdt_reset();

  updateBuzzer();
  updateSolenoid();
  updateIncidentDisplay();
  checkLowBattery();
  handleLimitSwitch();
  handleExitButton();
  handleLockoutTimer();

  updateFPEnroll();
  updateRFIDEnroll();

  if (!lockedOut && fpEnrollState == FP_IDLE && !rfidEnrollActive) {
    handleKeypad();
    handleFingerprint();
    handleRFID();
  }
}

// ---------------------------------------------------------------------
// BLE SETUP + CALLBACKS
// ---------------------------------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleDeviceConnected = true;
    notifyStatus("READY");
  }
  void onDisconnect(BLEServer* s) override {
    bleDeviceConnected = false;
    fpEnrollState = FP_IDLE;
    rfidEnrollActive = false;
    pServer->getAdvertising()->start();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = String(c->getValue().c_str());

    if (cmd.startsWith("ENROLL_FP|")) {
      int p1 = cmd.indexOf('|');
      int p2 = cmd.indexOf('|', p1 + 1);
      if (p2 > 0) {
        int id = cmd.substring(p1 + 1, p2).toInt();
        String name = cmd.substring(p2 + 1);
        fpEnrollId = id;
        fpEnrollName = name;
        fpEnrollState = FP_WAIT_FIRST;
        notifyStatus("FP_PLACE_FINGER");
      }
    }
    else if (cmd.startsWith("ENROLL_RFID|")) {
      int p1 = cmd.indexOf('|');
      int p2 = cmd.indexOf('|', p1 + 1);
      if (p2 > 0) {
        rfidEnrollId = cmd.substring(p1 + 1, p2);
        rfidEnrollName = cmd.substring(p2 + 1);
        rfidEnrollActive = true;
        notifyStatus("RFID_WAIT_SCAN");
      }
    }
    else if (cmd == "CANCEL") {
      fpEnrollState = FP_IDLE;
      rfidEnrollActive = false;
      notifyStatus("READY");
    }
  }
};

void setupBLE() {
  BLEDevice::init("SmartDoorLock");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  BLECharacteristic *cmdChar = service->createCharacteristic(
      COMMAND_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CommandCallbacks());

  pStatusChar = service->createCharacteristic(
      STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pStatusChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising = pServer->getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  advertising->start();
}

// ---------------------------------------------------------------------
// FINGERPRINT ENROLLMENT
// ---------------------------------------------------------------------
void updateFPEnroll() {
  if (fpEnrollState == FP_IDLE) return;

  switch (fpEnrollState) {
    case FP_WAIT_FIRST: {
      if (finger.getImage() == FINGERPRINT_OK) {
        if (finger.image2Tz(1) == FINGERPRINT_OK) {
          notifyStatus("FP_REMOVE_FINGER");
          fpEnrollState = FP_WAIT_REMOVE;
        } else {
          notifyStatus("FP_FAIL|Could not read finger clearly");
          fpEnrollState = FP_IDLE;
        }
      }
      break;
    }
    case FP_WAIT_REMOVE: {
      if (finger.getImage() == FINGERPRINT_NOFINGER) {
        notifyStatus("FP_PLACE_AGAIN");
        fpEnrollState = FP_WAIT_SECOND;
      }
      break;
    }
    case FP_WAIT_SECOND: {
      if (finger.getImage() == FINGERPRINT_OK) {
        bool ok = (finger.image2Tz(2) == FINGERPRINT_OK) &&
                  (finger.createModel() == FINGERPRINT_OK) &&
                  (finger.storeModel(fpEnrollId) == FINGERPRINT_OK);
        if (ok) {
          prefs.begin("doorlock", false);
          prefs.putString(("fpname" + String(fpEnrollId)).c_str(), fpEnrollName);
          prefs.end();
          notifyStatus("FP_OK|" + String(fpEnrollId) + "|" + fpEnrollName);
          logIncident("Enrolled FP: " + fpEnrollName);
        } else {
          notifyStatus("FP_FAIL|Prints did not match, try again");
        }
        fpEnrollState = FP_IDLE;
      }
      break;
    }
    default: break;
  }
}

// ---------------------------------------------------------------------
// NFC/RFID ENROLLMENT
// ---------------------------------------------------------------------
void updateRFIDEnroll() {
  if (!rfidEnrollActive) return;

  uint8_t uid[7];
  uint8_t uidLength;
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50)) return;

  String uidStr = uidToString(uid, uidLength);

  if (findRFIDIndex(uidStr) >= 0) {
    notifyStatus("RFID_DUPLICATE|" + uidStr);
  } else if (rfidCount >= MAX_RFID) {
    notifyStatus("RFID_FAIL|Card list full");
  } else {
    addRFIDEntry(uidStr, rfidEnrollId, rfidEnrollName);
    notifyStatus("RFID_OK|" + uidStr + "|" + rfidEnrollId + "|" + rfidEnrollName);
    logIncident("Enrolled RFID: " + rfidEnrollName);
  }
  rfidEnrollActive = false;
}

String uidToString(uint8_t *uid, uint8_t length) {
  String s = "";
  for (uint8_t i = 0; i < length; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

int findRFIDIndex(String uid) {
  for (int i = 0; i < rfidCount; i++) {
    if (rfidUIDs[i] == uid) return i;
  }
  return -1;
}

bool isRFIDAuthorized(String uid) {
  return findRFIDIndex(uid) >= 0;
}

void addRFIDEntry(String uid, String personId, String name) {
  if (rfidCount >= MAX_RFID) return;

  rfidUIDs[rfidCount] = uid;
  rfidIds[rfidCount] = personId;
  rfidNames[rfidCount] = name;
  rfidCount++;
  saveRFIDList();
}

void loadRFIDList() {
  prefs.begin("rfidlist", true);
  rfidCount = prefs.getInt("count", 0);
  if (rfidCount > MAX_RFID) rfidCount = MAX_RFID;
  for (int i = 0; i < rfidCount; i++) {
    rfidUIDs[i]  = prefs.getString(("u" + String(i)).c_str(), "");
    rfidNames[i] = prefs.getString(("n" + String(i)).c_str(), "");
    rfidIds[i]   = prefs.getString(("i" + String(i)).c_str(), "");
  }
  prefs.end();
}

void saveRFIDList() {
  prefs.begin("rfidlist", false);
  prefs.putInt("count", rfidCount);
  for (int i = 0; i < rfidCount; i++) {
    prefs.putString(("u" + String(i)).c_str(), rfidUIDs[i]);
    prefs.putString(("n" + String(i)).c_str(), rfidNames[i]);
    prefs.putString(("i" + String(i)).c_str(), rfidIds[i]);
  }
  prefs.end();
}

// ---------------------------------------------------------------------
// LCD HELPERS
// ---------------------------------------------------------------------
void lcdLine(uint8_t row, String text) {
  if (text.length() > 16) text = text.substring(0, 16);
  while (text.length() < 16) text += ' ';
  lcd.setCursor(0, row);
  lcd.print(text);
}

void drawCurrentPage() {
  if (showingIncident || showingWelcome || lockedOut) return;
  DateTime now = rtc.now();

  switch (displayPage) {
   case 0: {
      char timeBuf[17];
      snprintf(timeBuf, sizeof(timeBuf), " %02d:%02d  %02d/%02d/%02d ", 
               now.hour(), now.minute(), now.day(), now.month(), now.year() % 100);

      lcdLine(0, "  DOOR LOCKED \x01 "); 
      lcdLine(1, timeBuf);           
      lcdLine(2, "ENTER PIN/SCAN.."); 
      lcdLine(3, "");                
      break;
    }
    case 1: {
      lcdLine(0, "Battery Status");
      lcdLine(1, String(readBatteryVoltage(), 2) + " V");
      lcdLine(2, String(batteryPercent()) + " % remaining");
      lcdLine(3, "");
      break;
    }
    case 2: {
      lcdLine(0, "Last Incident:");
      lcdLine(1, lastIncident.substring(0, 16));
      lcdLine(2, lastIncident.length() > 16 ? lastIncident.substring(16, 32) : "");
      lcdLine(3, "");
      break;
    }
    case 3: {
      lcdLine(0, "Password Info");
      lcdLine(1, "Default:" + DEFAULT_PIN);
      lcdLine(2, "Current:" + storedPin);
      lcdLine(3, "");
      break;
    }
  }
}

void logIncident(String msg) {
  lastIncident = msg;
  DateTime now = rtc.now();
  Serial.printf("[%02d:%02d:%02d] INCIDENT: %s\n", now.hour(), now.minute(), now.second(), msg.c_str());

  lcdLine(0, "** INCIDENT **");
  lcdLine(1, msg.substring(0, 16));
  lcdLine(2, msg.length() > 16 ? msg.substring(16, 32) : "");
  lcdLine(3, "");
  showingIncident = true;
  incidentUntil = millis() + INCIDENT_HOLD_MS;
}

void showWelcome(String method, String name, String personId) {
  DateTime now = rtc.now();
  String logLine = name.length() ? ("Access: " + name + " (" + method + ")") : ("Access: " + method);
  lastIncident = logLine;
  Serial.printf("[%02d:%02d:%02d] ACCESS: %s\n", now.hour(), now.minute(), now.second(), logLine.c_str());

  lcdLine(0, "ACCESS GRANTED");
  lcdLine(1, name.length() ? ("Welcome " + name) : "Welcome");
  lcdLine(2, personId.length() ? ("ID: " + personId) : "");
  lcdLine(3, "via " + method);

  showingWelcome = true;
  welcomeUntil = millis() + WELCOME_HOLD_MS;
}

void updateIncidentDisplay() {
  if (showingIncident && millis() >= incidentUntil) {
    showingIncident = false;
    if (!lockedOut) drawCurrentPage();
  }
  if (showingWelcome && millis() >= welcomeUntil) {
    showingWelcome = false;
    displayPage = 0;
    if (!lockedOut) drawCurrentPage();
  }
}

// ---------------------------------------------------------------------
// KEYPAD / PIN HANDLING
// ---------------------------------------------------------------------
void handleKeypad() {
  char key = keypad.getKey();
  if (!key) return;

  if (state == STATE_IDLE) state = STATE_ENTER_PIN;

  if (state == STATE_ENTER_PIN) {
    if (key == 'A') {
      if (pinBuffer.length() == 0) return;
      if (pinBuffer == storedPin) {
        grantAccess("Keypad PIN", "", "");
      } else {
        denyAccess("Wrong PIN Entered");
      }
      pinBuffer = "";
      state = STATE_IDLE;
    } else if (key == 'B') {
      pinBuffer = "";
      state = STATE_CHANGE_PIN_OLD;
      lcd.clear();
      lcdLine(0, "Change PIN");
      lcdLine(1, "Enter current:");
      lcdLine(2, "PIN: ");
    } else if (key == 'C') {
      if (pinBuffer.length() > 0) {
        pinBuffer.remove(pinBuffer.length() - 1);
        beepKeyClick();
      }
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    } else if (key == 'D') {
      pinBuffer = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    }
  }
  else if (state == STATE_CHANGE_PIN_OLD) {
    if (key == 'A') {
      if (pinBuffer == storedPin) {
        pinBuffer = "";
        state = STATE_CHANGE_PIN_NEW;
        lcd.clear();
        lcdLine(0, "Enter NEW PIN:");
        lcdLine(2, "PIN: ");
      } else {
        denyAccess("Failed PIN Change Auth");
        pinBuffer = "";
        state = STATE_IDLE;
      }
    } else if (key == 'C') {
      if (pinBuffer.length() > 0) {
        pinBuffer.remove(pinBuffer.length() - 1);
        beepKeyClick();
      }
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    } else if (key == 'D') {
      pinBuffer = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    }
  }
  else if (state == STATE_CHANGE_PIN_NEW) {
    if (key == 'A') {
      if (pinBuffer.length() >= 4) {
        newPinTemp = pinBuffer;
        pinBuffer = "";
        state = STATE_CHANGE_PIN_CONFIRM;
        lcd.clear();
        lcdLine(0, "Confirm NEW PIN:");
        lcdLine(2, "PIN: ");
      } else {
        lcdLine(2, "Min 4 digits!");
      }
    } else if (key == 'C') {
      if (pinBuffer.length() > 0) {
        pinBuffer.remove(pinBuffer.length() - 1);
        beepKeyClick();
      }
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    } else if (key == 'D') {
      pinBuffer = ""; newPinTemp = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    }
  }
  else if (state == STATE_CHANGE_PIN_CONFIRM) {
    if (key == 'A') {
      if (pinBuffer == newPinTemp) {
        storedPin = newPinTemp;
        prefs.begin("doorlock", false);
        prefs.putString("pin", storedPin);
        prefs.end();
        logIncident("PIN Changed OK");
        beepSuccess();
      } else {
        logIncident("PIN Change Mismatch");
        beepError();
      }
      pinBuffer = ""; newPinTemp = "";
      state = STATE_IDLE;
    } else if (key == 'C') {
      if (pinBuffer.length() > 0) {
        pinBuffer.remove(pinBuffer.length() - 1);
        beepKeyClick();
      }
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    } else if (key == 'D') {
      pinBuffer = ""; newPinTemp = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
      lcdLine(2, "PIN: " + maskPin(pinBuffer));
    }
  }
}

String maskPin(String pin) {
  String masked = "";
  for (unsigned int i = 0; i < pin.length(); i++) masked += "*";
  return masked;
}

// ---------------------------------------------------------------------
// SCANNING ANIMATIONS
// ---------------------------------------------------------------------
void playScanningAnimation(String methodText) {
  lcdLine(0, "PROCESSING...");
  lcdLine(1, methodText);
  
  for (int cycle = 0; cycle < 2; cycle++) {
    for (int dots = 1; dots <= 3; dots++) {
      String statusText = "Scanning";
      for (int i = 0; i < dots; i++) {
        statusText += ".";
      }
      lcdLine(2, statusText);
      delay(150);
    }
  }
}

// ---------------------------------------------------------------------
// AUTHENTICATION MODULES
// ---------------------------------------------------------------------
void handleFingerprint() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return;

  playScanningAnimation("Fingerprint");

  if (finger.image2Tz() != FINGERPRINT_OK) {
    denyAccess("FP Read Error");
    return;
  }

  if (finger.fingerFastSearch() == FINGERPRINT_OK) {
    int id = finger.fingerID;
    prefs.begin("doorlock", true);
    String name = prefs.getString(("fpname" + String(id)).c_str(), "");
    prefs.end();
    grantAccess("Fingerprint", name, String(id));
  } else {
    denyAccess("Unknown Fingerprint");
  }
}

void handleRFID() {
  uint8_t uid[7];
  uint8_t uidLength;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50)) return;

  playScanningAnimation("RFID Card");

  String uidStr = uidToString(uid, uidLength);
  int idx = findRFIDIndex(uidStr);

  if (idx >= 0) {
    grantAccess("NFC/RFID", rfidNames[idx], rfidIds[idx]);
  } else {
    denyAccess("Unknown NFC/RFID: " + uidStr);
  }
}

// ---------------------------------------------------------------------
// SWITCHES & BUTTONS
// ---------------------------------------------------------------------
void handleLimitSwitch() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChangeTime = 0;

  bool reading = digitalRead(PIN_LIMIT_SW);

  if (reading != lastReading) {
    lastChangeTime = millis();
  }

  if (millis() - lastChangeTime > DEBOUNCE_MS && reading != stableState) {
    stableState = reading;

    digitalWrite(PIN_LED_SW_GREEN, stableState == HIGH ? HIGH : LOW);
    digitalWrite(PIN_LED_SW_RED,   stableState == LOW  ? HIGH : LOW);

    if (stableState == LOW) {
      logIncident("Key Override Used");
      unlockSolenoid();
      beepAccessGranted();
    }
  }

  lastReading = reading;
}

void handleExitButton() {
  static bool lastReading = LOW;
  static bool stableState = LOW;
  static unsigned long lastChangeTime = 0;
  static unsigned long pressStart = 0;

  bool reading = digitalRead(PIN_EXIT_BTN);

  if (reading != lastReading) {
    lastChangeTime = millis();
  }

  if (millis() - lastChangeTime > DEBOUNCE_MS && reading != stableState) {
    stableState = reading;

    if (stableState == HIGH) {
      pressStart = millis();
    } else {
      unsigned long heldFor = millis() - pressStart;
      if (heldFor >= LONG_PRESS_MS) {
        logIncident("Exit Button Pressed");
        unlockSolenoid();
        beepAccessGranted();
      } else {
        displayPage = (displayPage + 1) % NUM_PAGES;
        beepKeyClick();
        if (state == STATE_IDLE) drawCurrentPage();
      }
    }
  }

  lastReading = reading;
}

// ---------------------------------------------------------------------
// ACCESS MANAGEMENT
// ---------------------------------------------------------------------
void grantAccess(String method, String name, String personId) {
  failedAttempts = 0;
  beepAccessGranted();
  unlockSolenoid();
  showWelcome(method, name, personId);
  state = STATE_IDLE;
}

void denyAccess(String reason) {
  failedAttempts++;
  logIncident(reason + " (" + String(failedAttempts) + "/" + String(MAX_PIN_ATTEMPTS) + ")");

  if (failedAttempts >= MAX_PIN_ATTEMPTS) {
    lockedOut = true;
    lockoutStart = millis();
    beepAlarm();
    logIncident("ALARM: 3x Failed - Locked 30s");
  } else {
    beepError();
  }
}

// Lockout countdown handler updates LCD dynamically each second
void handleLockoutTimer() {
  if (!lockedOut) return;

  unsigned long elapsed = millis() - lockoutStart;

  if (elapsed >= LOCKOUT_MS) {
    lockedOut = false;
    failedAttempts = 0;
    logIncident("Lockout Cleared");
    drawCurrentPage();
    return;
  }

  // Display 30-second countdown on LCD once initial incident banner finishes
  if (!showingIncident) {
    static uint32_t lastSec = 999;
    uint32_t remainingSec = (LOCKOUT_MS - elapsed + 999) / 1000;

    if (remainingSec != lastSec) {
      lastSec = remainingSec;

      char secBuf[17];
      snprintf(secBuf, sizeof(secBuf), "   Wait: %2ds    ", remainingSec);

      lcdLine(0, "!! SYSTEM LOCKED !!");
      lcdLine(1, " Too Many Failed ");
      lcdLine(2, secBuf);
      lcdLine(3, "  Please Wait... ");
    }
  }
}

// ---------------------------------------------------------------------
// ACTUATORS & PERIPHERALS
// ---------------------------------------------------------------------
void unlockSolenoid() {
  digitalWrite(PIN_SOLENOID, LOW);
  solenoidOpen = true;
  solenoidOpenUntil = millis() + UNLOCK_MS;
}

void updateSolenoid() {
  if (solenoidOpen && millis() >= solenoidOpenUntil) {
    digitalWrite(PIN_SOLENOID, HIGH);
    solenoidOpen = false;
  }
}

void startBuzzerPattern(uint16_t *onArr, uint16_t *offArr, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    buzzerOnMs[i] = onArr[i];
    buzzerOffMs[i] = offArr[i];
  }
  buzzerStepCount = count;
  buzzerStepIndex = 0;
  buzzerActive = true;
  buzzerCurrentlyOn = true;
  buzzerStepStart = millis();
  digitalWrite(PIN_BUZZER, HIGH);
}

void updateBuzzer() {
  if (!buzzerActive) return;
  unsigned long elapsed = millis() - buzzerStepStart;

  if (buzzerCurrentlyOn) {
    if (elapsed >= buzzerOnMs[buzzerStepIndex]) {
      digitalWrite(PIN_BUZZER, LOW);
      buzzerCurrentlyOn = false;
      buzzerStepStart = millis();
    }
  } else {
    if (elapsed >= buzzerOffMs[buzzerStepIndex]) {
      buzzerStepIndex++;
      if (buzzerStepIndex >= buzzerStepCount) {
        buzzerActive = false;
        digitalWrite(PIN_BUZZER, LOW);
      } else {
        digitalWrite(PIN_BUZZER, HIGH);
        buzzerCurrentlyOn = true;
        buzzerStepStart = millis();
      }
    }
  }
}

void beepKeyClick() {
  uint16_t on[] = {30}; uint16_t off[] = {0};
  startBuzzerPattern(on, off, 1);
}

void beepSuccess() {
  uint16_t on[] = {150}; uint16_t off[] = {0};
  startBuzzerPattern(on, off, 1);
}

void beepAccessGranted() {
  uint16_t on[]  = {100, 100, 100, 100};
  uint16_t off[] = {50, 50, 50, 0};
  startBuzzerPattern(on, off, 4);
}

void beepError() {
  uint16_t on[]  = {150, 150, 150};
  uint16_t off[] = {200, 200, 0};
  startBuzzerPattern(on, off, 3);
}

void beepAlarm() {
  uint16_t on[]  = {300, 300, 300, 300, 300, 300};
  uint16_t off[] = {150, 150, 150, 150, 150, 0};
  startBuzzerPattern(on, off, 6);
}

// ---------------------------------------------------------------------
// POWER MANAGEMENT
// ---------------------------------------------------------------------
float readBatteryVoltage() {
  int raw = analogRead(PIN_BATT_ADC);
  float vAdc = (raw / 4095.0) * 3.3;
  return vAdc / BATT_DIVIDER_RATIO;
}

int batteryPercent() {
  float v = readBatteryVoltage();
  float pct = (v - BATT_EMPTY_VOLTAGE) / (BATT_FULL_VOLTAGE - BATT_EMPTY_VOLTAGE) * 100.0;
  if (pct > 100) pct = 100;
  if (pct < 0) pct = 0;
  return (int)pct;
}

void checkLowBattery() {
  static unsigned long lastCheck = 0;
  static bool lowBattWarned = false;
  if (millis() - lastCheck < 10000) return;
  lastCheck = millis();

  int pct = batteryPercent();
  if (pct <= 15 && !lowBattWarned) {
    lowBattWarned = true;
    logIncident("LOW BATTERY " + String(pct) + "%");
    beepError();
  } else if (pct > 25) {
    lowBattWarned = false;
  }
}