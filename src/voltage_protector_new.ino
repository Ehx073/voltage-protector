/*
 * ============================================================================
 *  MAINS VOLTAGE / CURRENT PROTECTOR  (Arduino + 16x2 LCD + relay)
 * ============================================================================
 *  Flow of the program:
 *    loop() -> measure()        read voltage & current (one 20 ms mains cycle)
 *           -> handleSerial()   receive PC commands (after the splash screen)
 *           -> handleResetButton()
 *           -> protect()        (relay ON)  check for faults, trip if needed
 *              recover()        (relay OFF) wait for good mains, reconnect
 *           -> updateOutputs()  LEDs + buzzer
 *           -> updateDisplay()  LCD + optional serial stream, every 500 ms
 *
 *  States:  WAITING (boot, not yet connected)
 *           NORMAL  (relay ON, protecting)
 *           FAULT   (relay OFF after a trip or manual OFF)
 * ============================================================================
 */

#include <LiquidCrystal.h>
#include <EEPROM.h>
/* ========================= 1. PIN DEFINITIONS ============================ */
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);   // RS, E, D4-D7

#define PIN_RELAY 8    // relay output (HIGH = load connected)
#define PIN_GREEN 9    // green LED  (NORMAL)
#define PIN_RED   10   // red LED    (WAITING / FAULT)
#define PIN_BUZ   11   // buzzer     (FAULT only)
#define PIN_RESET 12   // reset button, active LOW (to GND)
#define PIN_SEL   A4   // menu: select / next item (active LOW)
#define PIN_UP    A2   // menu: increase value     (active LOW)
#define PIN_DOWN  A3   // menu: decrease value     (active LOW)

/* ========================= 2. CALIBRATION ================================ */
// Adjust after the first test run against a true-RMS meter
const float V_SCALE = 0.8168;    // volts per ADC count
const float I_SCALE = 0.09775;   // amps per ADC count (0.05 ohm shunt)

/* ========================= 3. PROTECTION SETTINGS ======================== */
// User-changeable over serial (SET OVP / UVP / OCP / DELAY). Lost on reset.
float OVP = 250;                 // over-voltage limit  (V RMS)
float UVP = 180;                 // under-voltage limit (V RMS)
float OCP = 10;                  // overload current limit (A RMS)
unsigned int reconnectDelay = 1; // seconds mains must be good before reconnect
                                 // (use 180 on real hardware)

// Fixed limits (not changeable over serial)
const float SURGE_V  = 270;      // instant trip above this RMS voltage
const float PEAK_V   = 400;      // instant trip above this peak voltage
const float HEAVY_OC = 15;       // heavy overload current (A)
const float SHORT_OC = 30;       // short-circuit current (A), instant trip

/* ========================= 4. STATE VARIABLES ============================ */
enum State { WAITING, NORMAL, FAULT };
const char* stateName[] = {"WAITING", "NORMAL", "FAULT"};
State state = WAITING;           // current system state
const char* cause = "NONE";      // reason shown on LCD / serial

// Latest measurements (updated by measure())
float vrms, vpk, irms;

// Protection counters
float thermal = 0;                       // inverse-time overload accumulator
int ovCnt = 0, uvCnt = 0, hvCnt = 0;     // consecutive-cycle fault counters
bool latched = false;                    // trip needs reset button / RELAY ON
bool manualOff = false;                  // relay turned off by RELAY OFF cmd

// Timing
unsigned long goodSince = 0;     // when mains became good (0 = not good)
unsigned long lastTx = 0;        // last display / stream update time
unsigned long splashStart = 0;   // when the splash screen started
bool splashDone = false;         // true after 1 s splash

// Serial input
bool streamOn = false;           // STREAM ON/OFF
char rxBuf[32];                  // incoming command buffer
byte rxN = 0;                    // number of chars in rxBuf

// Trip log (last 5 trips, oldest overwritten)
struct Rec { const char* c; int v; float i; unsigned long t; };
Rec logBuf[5];
int logN = 0;

/* ========================= 4b. EEPROM + MENU STATE ======================= */
// Settings saved in EEPROM so they survive power-off
struct Settings { byte magic; float ovp; float uvp; float ocp; unsigned int delaySec; };
const byte EE_MAGIC = 0xA5;      // marks EEPROM as holding valid settings

bool menuActive = false;         // true while the on-device menu is open
byte menuItem = 0;               // 0=OVP 1=UVP 2=OCP 3=DELAY
unsigned long menuLastKey = 0;   // time of last key press (for 10 s timeout)
bool keyFast = false;            // true when UP/DOWN has been held > 2 s

// Writes the current settings to EEPROM (put() only writes changed bytes)
void saveSettings() {
  Settings s = {EE_MAGIC, OVP, UVP, OCP, reconnectDelay};
  EEPROM.put(0, s);
}

// Loads settings from EEPROM at boot. Ignored if empty or out of range.
void loadSettings() {
  Settings s;
  EEPROM.get(0, s);
  if (s.magic != EE_MAGIC) return;
  if (s.ovp >= 230 && s.ovp <= 280 && s.uvp >= 100 && s.uvp <= 210 &&
      s.ocp >= 1 && s.ocp <= 25 && s.delaySec >= 1 && s.delaySec <= 600) {
    OVP = s.ovp;  UVP = s.uvp;  OCP = s.ocp;  reconnectDelay = s.delaySec;
  }
}


/* ========================= 5. MEASUREMENT ================================ */
// Samples both channels for 20 ms (one 50 Hz cycle) and computes
// RMS voltage, RMS current and peak voltage. Blocks for 20 ms by design:
// this window is the sampling period.
void measure() {
  long n = 0;
  float sv = 0, sv2 = 0, si = 0, si2 = 0;   // sums for mean and mean-square
  int vmax = 0, vmin = 1023;
  unsigned long t0 = micros();

  while (micros() - t0 < 20000UL) {
    int v = analogRead(A0), i = analogRead(A1);
    sv += v;  sv2 += (float)v * v;
    si += i;  si2 += (float)i * i;
    n++;
    if (v > vmax) vmax = v;
    if (v < vmin) vmin = v;
  }

  float mv = sv / n, mi = si / n;            // DC offsets (mid-rail bias)
  // RMS of the AC part: sqrt(mean(x^2) - mean(x)^2)
  vrms = sqrt(max(0.0f, sv2 / n - mv * mv)) * V_SCALE;
  irms = sqrt(max(0.0f, si2 / n - mi * mi)) * I_SCALE;
  vpk  = (vmax - vmin) * 0.5 * V_SCALE;
}

/* ========================= 6. TRIP + LOGGING ============================= */
// Stores one trip record in the 5-entry log (shifts out the oldest if full)
void logTrip(const char* c) {
  Rec r = {c, (int)vrms, irms, millis() / 1000};
  if (logN < 5) {
    logBuf[logN++] = r;
  } else {
    for (int k = 0; k < 4; k++) logBuf[k] = logBuf[k + 1];
    logBuf[4] = r;
  }
}

// Sends a one-line JSON trip event to the PC
void sendTripEvent(const char* c) {
  Serial.print(F("{\"event\":\"TRIP\",\"cause\":\"")); Serial.print(c);
  Serial.print(F("\",\"v\":"));  Serial.print(vrms, 1);
  Serial.print(F(",\"i\":"));    Serial.print(irms, 2);
  Serial.print(F(",\"t\":"));    Serial.print(millis() / 1000);
  Serial.println('}');
}

// Disconnects the load, enters FAULT, logs and reports the event.
// latch = true -> stays off until reset button or RELAY ON
void trip(const char* c, bool latch) {
  digitalWrite(PIN_RELAY, LOW);
  state = FAULT;
  cause = c;
  goodSince = 0;
  if (latch) latched = true;
  logTrip(c);
  sendTripEvent(c);
}

/* ========================= 7. PROTECTION (relay ON) ====================== */
// Called every cycle while state == NORMAL. Checks run in priority order;
// the first one that trips returns immediately.
void protect() {
  // 1. Surge: instant trip, auto-recovers
  if (vpk > PEAK_V || vrms > SURGE_V) { trip("SURGE", false); return; }

  // 2. Over-voltage: 5 consecutive cycles (~100 ms), auto-recovers
  if (vrms > OVP) { if (++ovCnt >= 5)  { trip("OVERVOLTAGE", false); return; } }
  else ovCnt = 0;

  // 3. Under-voltage: 25 consecutive cycles (~500 ms), auto-recovers
  if (vrms < UVP) { if (++uvCnt >= 25) { trip("UNDERVOLTAGE", false); return; } }
  else uvCnt = 0;

  // 4. Short circuit: instant trip, LATCHED
  if (irms > SHORT_OC) { trip("SHORT_CIRCUIT", true); return; }

  // 5. Heavy overload: 5 consecutive cycles, LATCHED
  if (irms > HEAVY_OC) { if (++hvCnt >= 5) { trip("HEAVY_OVERLOAD", true); return; } }
  else hvCnt = 0;

  // 6. Inverse-time overload: heat builds with (I/OCP)^2, cools when OK
  if (irms > OCP) {
    thermal += (irms * irms) / (OCP * OCP);
    if (thermal >= 250) { trip("OVERLOAD", true); return; }
  } else {
    thermal *= 0.99;
  }
}

/* ========================= 8. RECOVERY (relay OFF) ======================= */
// Called every cycle while state != NORMAL. Reconnects the relay once the
// mains has stayed within 190-240 V for reconnectDelay seconds, and no
// latch / manual-off is active.
void recover() {
  bool ok = vrms >= 190 && vrms <= 240 && !latched && !manualOff;

  if (!ok) {
    goodSince = 0;                              // restart the good-mains timer
    if (state == WAITING) {                     // update the cause shown on LCD
      if (vrms > OVP)      cause = "OVERVOLTAGE";
      else if (vrms < UVP) cause = "UNDERVOLTAGE";
      else                 cause = "NONE";
    }
    return;
  }

  if (goodSince == 0) goodSince = millis();     // mains just became good
  if (millis() - goodSince >= reconnectDelay * 1000UL) {
    digitalWrite(PIN_RELAY, HIGH);              // reconnect the load
    state = NORMAL;
    cause = "NONE";
    ovCnt = uvCnt = hvCnt = 0;                  // fresh start
    thermal = 0;
  }
}

/* ========================= 9. SERIAL: OUTPUT ============================= */
// Sends the current status as one JSON line
void sendStatus() {
  Serial.print(F("{\"v\":"));      Serial.print(vrms, 1);
  Serial.print(F(",\"i\":"));      Serial.print(irms, 2);
  Serial.print(F(",\"p\":"));      Serial.print(vrms * irms, 0);
  Serial.print(F(",\"state\":\"")); Serial.print(stateName[state]);
  Serial.print(F("\",\"cause\":\"")); Serial.print(cause);
  Serial.println("\"}");
}

// Prints the stored trips, one per line, then END
void sendLog() {
  for (int k = 0; k < logN; k++) {
    Serial.print(logBuf[k].c);      Serial.print(',');
    Serial.print(logBuf[k].v);      Serial.print("V,");
    Serial.print(logBuf[k].i, 1);   Serial.print("A,t=");
    Serial.println(logBuf[k].t);
  }
  Serial.println(F("END"));
}

/* ========================= 10. SERIAL: COMMANDS ========================== */
// Range-checked setter for float settings. Prints OK or ERR RANGE.
void setFloatParam(float x, float lo, float hi, float &target) {
  if (x >= lo && x <= hi) { target = x; saveSettings(); Serial.println(F("OK")); }
  else Serial.println(F("ERR RANGE"));
}

// Executes one complete command line.
// Commands: GET STATUS | STREAM ON/OFF | SET OVP/UVP/OCP/DELAY <n>
//           RELAY ON/OFF | GET LOG | CLEAR LOG
void processCmd(String s) {
  s.trim();
  s.toUpperCase();

  // --- Status / streaming ---
  if (s == "GET STATUS")      sendStatus();
  else if (s == "STREAM ON")  { streamOn = true;  Serial.println(F("OK")); }
  else if (s == "STREAM OFF") { streamOn = false; Serial.println(F("OK")); }

  // --- Settings (with allowed ranges) ---
  else if (s.startsWith("SET OVP ")) setFloatParam(s.substring(8).toFloat(), 230, 280, OVP);
  else if (s.startsWith("SET UVP ")) setFloatParam(s.substring(8).toFloat(), 100, 210, UVP);
  else if (s.startsWith("SET OCP ")) setFloatParam(s.substring(8).toFloat(), 1, 25, OCP);
  else if (s.startsWith("SET DELAY ")) {
    long x = s.substring(10).toInt();
    if (x >= 1 && x <= 600) { reconnectDelay = x; saveSettings(); Serial.println(F("OK")); }
    else Serial.println(F("ERR RANGE"));
  }

  // --- Manual relay control ---
  else if (s == "RELAY ON") {            // clears flags; recover() reconnects
    manualOff = false;
    latched = false;
    Serial.println(F("OK"));
  }
  else if (s == "RELAY OFF") {           // immediate disconnect, logged
    if (!manualOff) logTrip("MANUAL");   // log once per manual-off
    manualOff = true;
    digitalWrite(PIN_RELAY, LOW);
    state = FAULT;
    cause = "MANUAL";
    goodSince = 0;
    Serial.println(F("OK"));
  }

  // --- Log ---
  else if (s == "GET LOG")   sendLog();
  else if (s == "CLEAR LOG") { logN = 0; Serial.println(F("OK")); }

  // --- Unknown command ---
  else {
    Serial.println(F("ERR CMD"));
    Serial.println(F("Please try again."));
  }
}

// Reads serial bytes into rxBuf; when a line ends (CR or LF) runs the command
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (rxN > 0) {
        rxBuf[rxN] = 0;
        rxN = 0;
        processCmd(String(rxBuf));
      }
    } else if (c >= 32 && c < 127 && rxN < sizeof(rxBuf) - 1) {
      rxBuf[rxN++] = c;          // printable ASCII only, junk bytes dropped
    }
  }
}

/* ========================= 11. INPUTS / OUTPUTS ========================== */
// Physical reset button (active LOW): clears latch and manual-off
void handleResetButton() {
  if (digitalRead(PIN_RESET) == LOW) {
    latched = false;
    manualOff = false;
  }
}

// LEDs and buzzer follow the state
void updateOutputs() {
  digitalWrite(PIN_GREEN, state == NORMAL);
  digitalWrite(PIN_RED,   state != NORMAL);
  // Buzzer only for real faults, not for a manual OFF
  digitalWrite(PIN_BUZ,   state == FAULT && strcmp(cause, "MANUAL") != 0);
}

/* ========================= 12. LCD ======================================= */
// Prints text on one row padded to 16 chars so old characters are erased
void lcdLine(byte row, const char* s) {
  lcd.setCursor(0, row);
  byte n = 0;
  while (*s && n < 16) { lcd.write(*s++); n++; }
  while (n < 16)       { lcd.write(' ');  n++; }
}

// Every 500 ms: stream status (if enabled) and refresh the LCD.
// Row 1: "V:230 I:1.5"   Row 2: fault cause, or state name if no cause
void updateDisplay() {
  if (millis() - lastTx < 500) return;
  lastTx = millis();

  if (streamOn) sendStatus();
  if (menuActive) return;                          // menu is using the LCD

  char iBuf[8], line[17];
  dtostrf(irms, 3, 1, iBuf);                       // AVR snprintf can't print floats
  snprintf(line, sizeof(line), "V:%d I:%s", (int)vrms, iBuf);
  lcdLine(0, line);
  lcdLine(1, strcmp(cause, "NONE") ? cause : stateName[state]);
}

// Shows the splash for 1 s without blocking, then clears the screen and
// discards serial junk received during boot (fixes first-command error).
// Returns true once the splash is finished.
bool handleSplash() {
  if (splashDone) return true;
  if (millis() - splashStart >= 1000) {
    splashDone = true;
    lcd.clear();
    while (Serial.available()) Serial.read();   // flush boot junk
    rxN = 0;                                     // clear partial command
    Serial.println(F("READY"));                  // tells the PC it can send now
  }
  return splashDone;
}

/* ========================= 12b. ON-DEVICE MENU =========================== */
// Reads the 3 buttons (non-blocking, debounced). Returns 0 = no key,
// 1 = SELECT, 2 = UP, 3 = DOWN. UP/DOWN auto-repeat while held.
byte readKey() {
  static bool prev[3] = {HIGH, HIGH, HIGH};
  static unsigned long changed[3]   = {0, 0, 0};
  static unsigned long pressedAt[3] = {0, 0, 0};
  static unsigned long nextRep[3]   = {0, 0, 0};
  const byte pins[3] = {PIN_SEL, PIN_UP, PIN_DOWN};
  unsigned long now = millis();

  for (byte k = 0; k < 3; k++) {
    bool lvl = digitalRead(pins[k]);
    if (lvl != prev[k]) {
      if (now - changed[k] > 30) {                 // 30 ms debounce
        prev[k] = lvl;
        changed[k] = now;
        if (lvl == LOW) {                          // new press
          pressedAt[k] = now;
          nextRep[k] = now + 500;                  // repeat starts after 0.5 s
          keyFast = false;
          return k + 1;
        }
      }
    } else if (lvl == LOW && k > 0 && now >= nextRep[k]) {
      nextRep[k] = now + 100;                      // repeat every 100 ms
      keyFast = (now - pressedAt[k] > 2000);       // held > 2 s = fast
      return k + 1;
    }
  }
  return 0;
}

// Changes the selected setting by one step (dir = +1 or -1), within limits
void menuAdjust(int dir) {
  switch (menuItem) {
    case 0: OVP = constrain(OVP + dir, 230, 280); break;
    case 1: UVP = constrain(UVP + dir, 100, 210); break;
    case 2: OCP = constrain(OCP + dir * 0.5, 1, 25); break;
    case 3: {
      long d = (long)reconnectDelay + dir * (keyFast ? 10 : 1);
      reconnectDelay = constrain(d, 1, 600);
      break;
    }
  }
}

// Draws the selected setting and its value on the LCD
void showMenu() {
  const char* names[] = {"OVP", "UVP", "OCP", "DELAY"};
  char top[17], val[17], buf[8];
  snprintf(top, sizeof(top), "SET %s", names[menuItem]);
  switch (menuItem) {
    case 0: snprintf(val, sizeof(val), "%d V", (int)OVP); break;
    case 1: snprintf(val, sizeof(val), "%d V", (int)UVP); break;
    case 2: dtostrf(OCP, 4, 1, buf);
            snprintf(val, sizeof(val), "%s A", buf); break;
    case 3: snprintf(val, sizeof(val), "%u s", reconnectDelay); break;
  }
  lcdLine(0, top);
  lcdLine(1, val);
}

// Closes the menu, saves the settings and forces an LCD refresh
void exitMenu() {
  menuActive = false;
  saveSettings();
  lastTx = 0;
}

// Called every loop: SELECT opens/steps/closes, UP/DOWN change the value
void handleMenu() {
  if (menuActive && millis() - menuLastKey > 10000UL) { exitMenu(); return; }

  byte key = readKey();
  if (key == 0) return;
  menuLastKey = millis();

  if (key == 1) {                                  // SELECT
    if (!menuActive) { menuActive = true; menuItem = 0; }
    else if (++menuItem > 3) { exitMenu(); return; }
  } else if (menuActive) {                         // UP / DOWN
    menuAdjust(key == 2 ? 1 : -1);
  }
  if (menuActive) showMenu();
}

/* ========================= 13. SETUP / LOOP ============================== */
void setup() {
  // Relay first, so the load is OFF at boot (fail-safe)
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_BUZ,   OUTPUT);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_SEL,   INPUT_PULLUP);
  pinMode(PIN_UP,    INPUT_PULLUP);
  pinMode(PIN_DOWN,  INPUT_PULLUP);

  loadSettings();                             // saved settings override defaults

  Serial.begin(9600);

  // Splash screen (cleared later by handleSplash(), no delay())
  lcd.begin(16, 2);
  lcd.setCursor(0, 0); lcd.print(F("Voltage"));
  lcd.setCursor(0, 1); lcd.print(F("Protector"));
  splashStart = millis();
}

void loop() {
  measure();                                  // 1. read voltage & current

  if (handleSplash()) handleSerial();         // 2. commands only after splash
  if (splashDone) handleMenu();               //    on-device menu buttons

  handleResetButton();                        // 3. physical reset button

  if (state == NORMAL) protect();             // 4. relay ON  -> watch for faults
  else                 recover();             //    relay OFF -> try to reconnect

  updateOutputs();                            // 5. LEDs + buzzer

  if (splashDone) updateDisplay();            // 6. LCD + serial stream
}
