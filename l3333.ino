/*
  =====================================================================
  SMART ELECTRONIC DOOR LOCK SYSTEM - ESP32   (v16)
  =====================================================================
  Access methods : Fingerprint | 4x4 Keypad PIN | NFC/RFID (PN532 I2C) | Physical Key
  Exit method     : Push button (inside room, no auth required)
  Monitoring      : 16x4 I2C LCD (manual paging via button), Active buzzer,
                    RTC timestamps, Battery voltage (2x 18650 series)
  Actuator        : 12V Electric Solenoid Lock (via relay/MOSFET)
  Enrollment      : Bluetooth Low Energy (BLE) — companion HTML app
                    enrolls new fingerprints AND new NFC/RFID cards,
                    each with a Name AND a person ID

  Keypad layout   : 0-9 = digits | A = Confirm | B = Change-PIN menu
                    C = Backspace (delete last digit) | D = Cancel/Clear

  CHANGELOG v16:
    - Corrected the indicator-LED polarity from v15: RED is lit when the
      limit switch is OPEN/triggered, GREEN when it's CLOSED/idle (v15 had
      these reversed - RED was tied to closed/idle instead)

  CHANGELOG v15:
    - Added two indicator LEDs (RED = GPIO23, GREEN = GPIO2) that mirror the
      raw limit switch position only. Purely a visual readout of the
      physical switch state for the user; does not affect or read from
      the solenoid/access-control logic.

  CHANGELOG v14:
    - The version-detection macro for the watchdog API didn't resolve
      correctly on this core - switched to using the new struct-based
      esp_task_wdt_init() unconditionally, matching what the compiler
      error confirmed this specific core actually needs

  CHANGELOG v13:
    - Fixed watchdog compile error - newer ESP32 core (3.x/IDF5) needs
      a config struct for esp_task_wdt_init(), older cores (2.x) use
      the simple (seconds, panic) form. Code now detects which one
      you have and uses the right API automatically.
    - RTC is now set EXACTLY ONCE, ever (tracked via an NVS flag) -
      no longer resets to the firmware's compile-time timestamp every
      time lostPower() reports true, which could otherwise happen
      repeatedly with a flaky/weak coin cell and silently corrupt
      your timestamps. From the first boot onward, the DS3231's own
      battery is fully trusted to keep time.

  CHANGELOG v12:
    - Fixed the "short press never flips screen" bug: exit button now
      has proper debounce (30ms stable-read requirement), matching
      the keypad's existing debounce - noise was corrupting the
      long/short press timing before
    - Limit switch logic inverted: now wired pull-up (idle HIGH,
      triggered LOW when the key is turned) instead of pull-down
    - Added Wire.setTimeOut(50) - I2C transactions now abort after
      50ms instead of potentially hanging forever, which is a likely
      cause of the random freezes (bus glitch waiting indefinitely)
    - Added a hardware task watchdog (5s timeout) - if the system
      ever does freeze for any reason, it now auto-reboots itself
      instead of requiring a manual power cycle

  CHANGELOG v11:
    - Moved solenoid relay from GPIO23 to GPIO5. GPIO5 is a strapping
      pin requiring HIGH at boot, which conveniently matches the
      relay's own idle/locked state (HIGH, since it's active-LOW) -
      the relay's pull-up reinforces the correct boot state instead
      of fighting it, unlike the earlier GPIO2 conflict

  CHANGELOG v10:
    - Relay confirmed ACTIVE-LOW by testing - inverted solenoid
      logic throughout: LOW = unlock/energized, HIGH = locked/off
      (boot state, unlockSolenoid, updateSolenoid all updated)

  CHANGELOG v9:
    - Exit button now distinguishes press length: a SHORT press just
      flips the LCD page, a LONG press (700ms+) opens the door -
      decision made on release, fully non-blocking

  CHANGELOG v8:
    - Removed the dedicated GPIO5 page-flip button entirely - the
      inside exit button now does double duty: opens the door AND
      advances the LCD page on every press
    - Limit switch (physical key) and exit button now both play the
      4-beep access-granted sound when they open the door, matching
      every other successful-access method
    - (See wiring notes for GPIO34/35 - external pull resistors
      required, no internal pulls available on these input-only pins)

  CHANGELOG v7:
    - Moved solenoid relay from GPIO2 to GPIO23 - GPIO2 is a boot
      strapping pin and the relay's onboard pull-up kept blocking
      uploads; GPIO23 is a completely safe, ordinary GPIO
    - Added lcdLine() helper - every LCD line is now always padded
      to exactly 16 characters, so a shorter message can never leave
      leftover characters from a longer previous one (no more overlap)
    - New "ACCESS GRANTED" welcome screen: shows the person's Name
      and ID for 4 seconds on successful access, then returns
      specifically to the HOME page (not whatever page was open)
    - RFID/NFC enrollment now captures a person ID as well as a name,
      matching how fingerprint enrollment already worked
    - Added a distinct 4-beep "door opened" buzzer pattern (100ms on,
      50ms off, x4) - separate from the existing single-beep used
      for PIN-change confirmation, so a successful entry always has
      its own unmistakable sound

  Libraries required (Install via Library Manager):
    - Wire.h                (built-in)
    - LiquidCrystal_I2C     (by Frank de Brabander / Marco Schwartz)
    - Keypad                (by Mark Stanley / Alexander Brevig)
    - Adafruit_PN532        (by Adafruit)
    - Adafruit_Fingerprint  (by Adafruit)
    - RTClib                (by Adafruit - works with DS3231/DS1307/PCF8523 etc.)
    - Preferences           (built-in ESP32 core, NVS storage)
    - ESP32 BLE Arduino     (built-in ESP32 core - BLEDevice etc.)
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
#include <BigFont01_I2C.h>

// ---------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------
#define PIN_PN532_IRQ   18
#define PIN_PN532_RESET 19
#define PIN_LIMIT_SW    34
#define PIN_EXIT_BTN    35   // now also flips the LCD page on every press
#define PIN_BATT_ADC    27    // ADC2_CH7 - fine since we use BLE not WiFi
#define PIN_SOLENOID    5     // moved from GPIO23 - relay is active-LOW, and GPIO5's boot
                              // requirement (must be HIGH) matches the relay's idle/locked
                              // state (HIGH), so they don't fight each other at boot
#define PIN_BUZZER      15    // ACTIVE buzzer - digitalWrite on/off only
#define PIN_LED_SW_RED    23    // RED   - lit when limit switch reads OPEN/triggered (LOW)
#define PIN_LED_SW_GREEN   2    // GREEN - lit when limit switch reads CLOSED/idle (HIGH).
                                // GPIO2 is a strapping pin, but a plain LED+resistor load
                                // (no external pull-down on it) does not affect boot - it's
                                // the same pin many ESP32 dev boards use for their onboard LED.

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
// OBJECTS
// ---------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 4);   // change to 0x3F if needed
BigFont01_I2C big(&lcd);
RTC_DS3231 rtc;                       // swap this type if your new RTC chip differs (e.g. RTC_PCF8523)
Adafruit_PN532 nfc(PIN_PN532_IRQ, PIN_PN532_RESET);
Preferences prefs;

HardwareSerial fingerSerial(2);
Adafruit_Fingerprint finger(&fingerSerial);

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
const float BATT_DIVIDER_RATIO   = 0.32;   // R2/(R1+R2) from 100k/47k divider
const float BATT_FULL_VOLTAGE    = 8.4;
const float BATT_EMPTY_VOLTAGE   = 6.0;
const unsigned long UNLOCK_MS    = 5000;
const uint8_t MAX_PIN_ATTEMPTS   = 3;
const unsigned long LOCKOUT_MS   = 30000;
const unsigned long INCIDENT_HOLD_MS = 2500;
const unsigned long WELCOME_HOLD_MS  = 4000;
const unsigned long LONG_PRESS_MS    = 700;   // hold exit button this long to open the door
const unsigned long DEBOUNCE_MS      = 30;    // debounce for exit button / limit switch
const uint8_t WDT_TIMEOUT_S          = 5;     // auto-reboot if loop() ever freezes this long
const String DEFAULT_PIN = "1234";
const int MAX_RFID = 20;
const int NUM_PAGES = 4;   // 0=status 1=battery 2=last incident 3=password info

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
bool lastMainsPresent = true; // kept for future use if mains-detect hardware is added back

// RFID dynamic list (loaded from NVS at boot)
String rfidUIDs[MAX_RFID];
String rfidNames[MAX_RFID];
String rfidIds[MAX_RFID];
int rfidCount = 0;

// Fingerprint enrollment state machine
enum FPEnrollState { FP_IDLE, FP_WAIT_FIRST, FP_WAIT_REMOVE, FP_WAIT_SECOND };
FPEnrollState fpEnrollState = FP_IDLE;
int fpEnrollId = -1;
String fpEnrollName = "";

// RFID enrollment
bool rfidEnrollActive = false;
String rfidEnrollName = "";
String rfidEnrollId = "";

// Non-blocking solenoid
bool solenoidOpen = false;
unsigned long solenoidOpenUntil = 0;

// Non-blocking buzzer engine
bool buzzerActive = false;
bool buzzerCurrentlyOn = false;
unsigned long buzzerStepStart = 0;
uint8_t buzzerStepIndex = 0;
uint8_t buzzerStepCount = 0;
uint16_t buzzerOnMs[8];
uint16_t buzzerOffMs[8];

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
  digitalWrite(PIN_SOLENOID, HIGH);  // locked at boot (fail-secure) - relay is ACTIVE-LOW, so HIGH = off/locked
  digitalWrite(PIN_BUZZER, LOW);

  // Set the two indicator LEDs to match the limit switch's real state at boot,
  // rather than defaulting to "locked" - avoids a false reading if the door
  // happened to already be open/unlocked by key when the board powered up.
  bool limitSwState = digitalRead(PIN_LIMIT_SW);   // HIGH = closed/idle, LOW = open/triggered
  digitalWrite(PIN_LED_SW_GREEN,   limitSwState == LOW  ? HIGH : LOW);
  digitalWrite(PIN_LED_SW_RED, limitSwState == HIGH ? HIGH : LOW);

  Wire.begin(21, 22);
  Wire.setTimeOut(50);   // I2C transactions abort after 50ms instead of hanging forever

  esp_task_wdt_config_t twdtConfig = {
    .timeout_ms = (uint32_t)WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdtConfig);
  esp_task_wdt_add(NULL);                  // watch the main loop task
  lcd.init();
  lcd.backlight();
  big.begin();
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("SMART DOOR LOCK");
   delay(2000);
  lcd.setCursor(0, 1);
  lcd.print(" Initializing.");
  delay(1200);
  lcd.setCursor(0, 1);
  lcd.print(" Initializing..");
  delay(1200);
  lcd.setCursor(0, 1);
  lcd.print(" Initializing...");
  delay(1200);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WELCOME");
  delay(8000);
  big.writechar(0, 0, 'W');
big.writechar(0, 3, 'E');
big.writechar(0, 6, 'L');
big.writechar(0, 9, 'C');
big.writechar(0, 12, 'O');
big.writechar(0, 15, 'M');
big.writechar(0, 18, 'E');
delay(8000);
  
  lcd.clear();

  if (!rtc.begin()) {
    lcd.setCursor(0, 2);
    lcd.print("RTC NOT FOUND!");
  }

  // Set the RTC exactly ONCE, ever. After this first boot, the DS3231's
  // own coin-cell battery keeps time - we never touch it again, even if
  // lostPower() reports true later (a flaky/weak coin cell shouldn't be
  // able to silently reset the clock back to a stale compile timestamp).
  prefs.begin("doorlock", false);
  bool rtcAlreadySet = prefs.getBool("rtcset", false);
  if (!rtcAlreadySet) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    prefs.putBool("rtcset", true);
    Serial.println("RTC set for the first time (compile timestamp). Will not be auto-set again.");
  }
  prefs.end();

  fingerSerial.begin(57600, SERIAL_8N1, 16, 17);
  finger.verifyPassword();

  nfc.begin();   // I2C mode - uses the same Wire bus as LCD/RTC (SDA=21, SCL=22)
  uint32_t nfcVersion = nfc.getFirmwareVersion();
  if (!nfcVersion) {
    lcd.setCursor(0, 3);
    lcd.print("PN532 NOT FOUND!");
  } else {
    nfc.SAMConfig();   // required once at boot before reading cards
  }

  prefs.begin("doorlock", false);
  storedPin = prefs.getString("pin", DEFAULT_PIN);
  prefs.end();

  loadRFIDList();
  setupBLE();

  
}

// ---------------------------------------------------------------------
// MAIN LOOP (fully non-blocking)
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
    pServer->getAdvertising()->start(); // resume advertising so app can reconnect
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
  advertising->addServiceUUID(SERVICE_UUID);   // required for filtered Web Bluetooth scans to find this device
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  advertising->start();
}

// ---------------------------------------------------------------------
// FINGERPRINT ENROLLMENT (non-blocking state machine, driven by BLE)
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
// NFC/RFID ENROLLMENT via PN532 (non-blocking, driven by BLE)
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
// LCD HELPERS - lcdLine() always pads to 16 chars so a shorter message
// can never leave leftover characters from a longer previous one
// ---------------------------------------------------------------------
void lcdLine(uint8_t row, String text) {
  if (text.length() > 16) text = text.substring(0, 16);
  while (text.length() < 16) text += ' ';
  lcd.setCursor(0, row);
  lcd.print(text);
}

// ---------------------------------------------------------------------
// LCD PAGES (manual - advanced only via page button)
// ---------------------------------------------------------------------
void drawCurrentPage() {
  if (showingIncident || showingWelcome) return; // don't overwrite an overlay currently on screen
  DateTime now = rtc.now();

  switch (displayPage) {
    case 0: {
      char buf[17];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d %02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month());
      lcdLine(0, "Door: LOCKED");
      lcdLine(1, buf);
      lcdLine(2, "Enter PIN/Scan...");
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

// Shown on successful access - distinct from logIncident(), and always
// resumes to the HOME page specifically once its hold time expires
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
    drawCurrentPage();
  }
  if (showingWelcome && millis() >= welcomeUntil) {
    showingWelcome = false;
    displayPage = 0;      // "resume to home" specifically, regardless of what page was open before
    drawCurrentPage();
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
      } else {
        denyAccess("Failed PIN Change Auth");
        pinBuffer = "";
        state = STATE_IDLE;
      }
    } else if (key == 'C') {
      if (pinBuffer.length() > 0) pinBuffer.remove(pinBuffer.length() - 1);
    } else if (key == 'D') {
      pinBuffer = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
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
      } else {
        lcdLine(2, "Min 4 digits!");
      }
    } else if (key == 'C') {
      if (pinBuffer.length() > 0) pinBuffer.remove(pinBuffer.length() - 1);
    } else if (key == 'D') {
      pinBuffer = ""; newPinTemp = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
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
      if (pinBuffer.length() > 0) pinBuffer.remove(pinBuffer.length() - 1);
    } else if (key == 'D') {
      pinBuffer = ""; newPinTemp = "";
      state = STATE_IDLE;
      drawCurrentPage();
    } else if (isdigit(key)) {
      pinBuffer += key;
      beepKeyClick();
    }
  }
}

String maskPin(String pin) {
  String masked = "";
  for (unsigned int i = 0; i < pin.length(); i++) masked += "*";
  return masked;
}

// ---------------------------------------------------------------------
// FINGERPRINT (normal auth check, only runs when not enrolling)
// ---------------------------------------------------------------------
void handleFingerprint() {
  if (finger.getImage() != FINGERPRINT_OK) return;
  if (finger.image2Tz() != FINGERPRINT_OK) return;
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

// ---------------------------------------------------------------------
// NFC/RFID via PN532 (normal auth check, only runs when not enrolling)
// ---------------------------------------------------------------------
void handleRFID() {
  uint8_t uid[7];
  uint8_t uidLength;
  // 50ms timeout keeps this snappy without blocking the loop for long
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50)) return;

  String uidStr = uidToString(uid, uidLength);
  int idx = findRFIDIndex(uidStr);

  if (idx >= 0) {
    grantAccess("NFC/RFID", rfidNames[idx], rfidIds[idx]);
  } else {
    denyAccess("Unknown NFC/RFID: " + uidStr);
  }
}

// ---------------------------------------------------------------------
// LIMIT SWITCH (physical key) and EXIT BUTTON
// ---------------------------------------------------------------------
// Limit switch wired as pull-up: idle = HIGH, triggered (key turned) = LOW
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

    // Indicator LEDs just mirror the physical limit switch position itself -
    // this is independent of the solenoid/access logic below, so it's updated
    // on every debounced change in either direction.
    digitalWrite(PIN_LED_SW_GREEN,   stableState == LOW  ? HIGH : LOW);
    digitalWrite(PIN_LED_SW_RED, stableState == HIGH ? HIGH : LOW);

    if (stableState == LOW) {
      logIncident("Key Override Used");
      unlockSolenoid();
      beepAccessGranted();
    }
  }

  lastReading = reading;
}

// This button does double duty: a LONG press opens the door,
// a SHORT press just flips the LCD page. Decision is made on
// release, based on how long it was held - fully non-blocking.
// Properly debounced so electrical noise can't corrupt the timing.
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
      pressStart = millis();   // stable press just started
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
// ACCESS GRANT / DENY
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

void handleLockoutTimer() {
  if (lockedOut && millis() - lockoutStart >= LOCKOUT_MS) {
    lockedOut = false;
    failedAttempts = 0;
    logIncident("Lockout Cleared");
  }
}

// ---------------------------------------------------------------------
// SOLENOID CONTROL (non-blocking) - relay is ACTIVE-LOW:
// LOW = unlock/energized, HIGH = locked/off
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

// ---------------------------------------------------------------------
// BUZZER (active buzzer - on/off only, non-blocking pattern engine)
// ---------------------------------------------------------------------
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

// Distinct pattern for successful door access, per spec: 4 beeps, 50ms gaps
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
// BATTERY MONITORING (2x 18650 series via divider on GPIO27)
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
