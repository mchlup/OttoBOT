/*
  ESP32 DevKit – 6× kontinuální servo SG90 360°
  + WiFiManager + WebServer + LittleFS config + sekvence
  + OTA z Arduino IDE + OTA .bin z webu + správce souborů
  + kalibrace serv (čas 360° + "center")
  + dálkový ovladač (mapování tlačítek na sekvence / URL)

  Kontinuální SG90 360°:
  - Řídíme SMĚR a VÝKON, ne polohu.
  - Parametr "power" = -100…+100:
      0     = stop (cca 1500 µs)
      +100  = plná rychlost jedním směrem
      -100  = plná rychlost opačným směrem

  LittleFS:
  - /index.html      – web UI
  - /config.txt      – konfigurace:
        STEP=10
        PULSE=50
        PIN0=13
        ...
        DESC0=Prave chodidlo
        ...
        FT0=1000
        CENTER0=0
        REMOTE0=seq:chuze_vpred
        REMOTE1=url:/api/seq_run?name=chuze_zpet
  - /seq_<name>.txt  – sekvence příkazů pro serva:
        # komentář
        servo,power,duration_ms
        0,50,1000
        0,0,500
        1,-30,1500
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>   // https://github.com/tzapu/WiFiManager
#include <ESP32Servo.h>    // https://github.com/madhephaestus/ESP32Servo
#include <LittleFS.h>      // LittleFS pro ukládání konfigurace a souborů
#include <ArduinoOTA.h>    // OTA z Arduino IDE
#include <Update.h>        // OTA z webu (nahrání .bin)

// ====== KONFIGURACE SERV ======
const uint8_t SERVO_COUNT = 6;

// Výchozí (kompilované) mapování servo -> GPIO pro ESP32 DevKit
uint8_t SERVO_PINS[SERVO_COUNT] = {
  13, // Servo 0
  14, // Servo 1
  25, // Servo 2
  26, // Servo 3
  27, // Servo 4
  32  // Servo 5
};

// Použitelné GPIO pro serva na ESP32 DevKit (PWM schopné, ne flash/USB/UART0)
const uint8_t GPIO_OPTIONS[] = {
  2, 4, 5,
  12, 13, 14, 15,
  16, 17, 18, 19,
  21, 22, 23,
  25, 26, 27,
  32, 33
};
const uint8_t GPIO_OPTION_COUNT = sizeof(GPIO_OPTIONS) / sizeof(GPIO_OPTIONS[0]);

// Kontinuální SG90 360° – PWM v mikrosekundách
const int SERVO_MIN_US     = 1000;  // plná rychlost jedním směrem
const int SERVO_MAX_US     = 2000;  // plná rychlost druhým směrem
const int SERVO_NEUTRAL_US = 1500;  // střed (stop)

// ====== LittleFS KONFIG ======
const char *CONFIG_PATH = "/config.txt";

// Sekvence – soubory typu /seq_<name>.txt
// prefix BEZ lomítka – protože file.name() vrací "seq_xxx.txt"
const char *SEQ_PREFIX      = "seq_";
const int   SEQ_PREFIX_LEN  = 4;   // délka "seq_"
const size_t SEQ_EXT_LEN    = 4;

// ====== STAV SERV ======
Servo  servos[SERVO_COUNT];

// Výkon v procentech -100..+100 (interní reprezentace)
int  currentPower[SERVO_COUNT];   // -100..100, 0 = stop
bool servoAttached[SERVO_COUNT];  // jestli je servo připojeno (PWM aktivní)

// Krok výkonu v procentech (změna power při LEVO/PRAVO tlačítku)
int defaultPowerStep = 10;        // 10 % na jedno kliknutí

// Délka impulzu pro /api/move v ms (LEVO/PRAVO)
int pulseDurationMs = 50;

// Popisy serv (max 32 znaků)
String servoDesc[SERVO_COUNT];

// Kalibrační parametry:
// - servoFullTurnMs: čas pro otočení o 360° při 100% výkonu
// - servoCurrentDeg: odhad aktuálního úhlu vzhledem k poslednímu "nulování"
// - servoCenterDeg:  uložený "střed" v okamžiku kalibrace CENTER (pro informaci)
int  servoFullTurnMs[SERVO_COUNT];
long servoCurrentDeg[SERVO_COUNT];
long servoCenterDeg[SERVO_COUNT];

// Asynchronní plánovač pro časované pulzy a sekvence
bool          servoPulseActive[SERVO_COUNT];
unsigned long servoPulseEndTime[SERVO_COUNT];

struct SeqStep {
  uint8_t  servo;
  int16_t  power;
  uint32_t durationMs;
};

const uint16_t MAX_SEQ_STEPS = 256;
SeqStep       seqSteps[MAX_SEQ_STEPS];
uint16_t      seqStepCount = 0;
uint16_t      seqStepIndex = 0;
bool          seqRunning   = false;
bool          seqStepActive = false;
unsigned long seqStepEndTime = 0;

// Remote ovladač – 6 tlačítek
// 0: dopředu, 1: dozadu, 2: vlevo, 3: vpravo, 4: otočka vlevo, 5: otočka vpravo
const uint8_t REMOTE_BTN_COUNT = 6;
String remoteType[REMOTE_BTN_COUNT];   // "none" / "seq" / "url"
String remoteValue[REMOTE_BTN_COUNT];  // název sekvence nebo URL příkazu

WebServer   server(80);
WiFiManager wm;

// ====== POMOCNÉ FUNKCE ======

int clampInt(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// Je GPIO v seznamu povolených pinů pro serva?
bool isValidGpio(uint8_t gpio) {
  for (uint8_t i = 0; i < GPIO_OPTION_COUNT; i++) {
    if (GPIO_OPTIONS[i] == gpio) return true;
  }
  return false;
}

// Jednoduché "escapování" stringu do JSON (odstraníme problematické znaky)
String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\' || c == '\r' || c == '\n') {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

// ====== LittleFS: načtení / uložení configu ======
void loadOrInitConfig() {
  // výchozí hodnoty
  defaultPowerStep = 10;
  pulseDurationMs  = 50;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servoDesc[i]       = "";
    servoFullTurnMs[i] = 1000;   // default 1 s / 360° při 100 %
    servoCurrentDeg[i] = 0;
    servoCenterDeg[i]  = 0;
  }
  for (uint8_t i = 0; i < REMOTE_BTN_COUNT; i++) {
    remoteType[i]  = "none";
    remoteValue[i] = "";
  }

  if (!LittleFS.exists(CONFIG_PATH)) {
    Serial.println("Config: config.txt neexistuje, pouziji defaulty.");
    return;
  }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    Serial.println("Config: nelze otevrit config.txt, pouziji defaulty.");
    return;
  }

  Serial.println("Config: nacitam config.txt");

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;

    if (line.startsWith("STEP=")) {
      int val = line.substring(5).toInt();
      if (val >= 1 && val <= 100) {
        defaultPowerStep = val;
      }
    } else if (line.startsWith("PULSE=")) {
      int val = line.substring(6).toInt();
      val = clampInt(val, 10, 5000);
      pulseDurationMs = val;
    } else if (line.startsWith("PIN")) {
      // formát: PIN<index>=<gpio>, např. PIN0=13
      int eqPos = line.indexOf('=');
      if (eqPos > 3) {
        int idx  = line.substring(3, eqPos).toInt();
        int gpio = line.substring(eqPos + 1).toInt();
        if (idx >= 0 && idx < SERVO_COUNT && isValidGpio((uint8_t)gpio)) {
          SERVO_PINS[idx] = (uint8_t)gpio;
        }
      }
    } else if (line.startsWith("DESC")) {
      // formát: DESC<index>=text
      int eqPos = line.indexOf('=');
      if (eqPos > 4) {
        int idx = line.substring(4, eqPos).toInt();
        if (idx >= 0 && idx < SERVO_COUNT) {
          String d = line.substring(eqPos + 1);
          d.replace('\r', ' ');
          d.replace('\n', ' ');
          d.trim();
          if (d.length() > 32) d = d.substring(0, 32);
          servoDesc[idx] = d;
        }
      }
    } else if (line.startsWith("FT")) {
      // formát: FT<index>=ms_360 (čas jedné otáčky při 100 % výkonu)
      int eqPos = line.indexOf('=');
      if (eqPos > 2) {
        int idx = line.substring(2, eqPos).toInt();
        int ms  = line.substring(eqPos + 1).toInt();
        if (idx >= 0 && idx < SERVO_COUNT) {
          ms = clampInt(ms, 100, 60000); // 0,1–60 s
          servoFullTurnMs[idx] = ms;
        }
      }
    } else if (line.startsWith("CENTER")) {
      // formát: CENTER<index>=deg (uložený střed – informativní)
      int eqPos = line.indexOf('=');
      if (eqPos > 6) {
        int idx = line.substring(6, eqPos).toInt();
        long d  = line.substring(eqPos + 1).toInt();
        if (idx >= 0 && idx < SERVO_COUNT) {
          servoCenterDeg[idx] = d;
        }
      }
    } else if (line.startsWith("REMOTE")) {
      // REMOTE<idx>=<type>:<value>
      int eqPos = line.indexOf('=');
      if (eqPos > 6) {
        int idx = line.substring(6, eqPos).toInt();
        if (idx >= 0 && idx < REMOTE_BTN_COUNT) {
          String v = line.substring(eqPos + 1);
          v.trim();
          int colon = v.indexOf(':');
          String t = "none";
          String val = "";
          if (colon > 0) {
            t   = v.substring(0, colon);
            val = v.substring(colon + 1);
          } else {
            t = v;
          }
          t.toLowerCase();
          if (t != "seq" && t != "url" && t != "none") t = "none";
          val.replace('\r', ' ');
          val.replace('\n', ' ');
          val.trim();
          if (val.length() > 64) val = val.substring(0, 64);
          remoteType[idx]  = t;
          remoteValue[idx] = val;
        }
      }
    }
  }

  f.close();
}

void saveConfigToLittleFS() {
  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) {
    Serial.println("Config: nelze zapsat config.txt");
    return;
  }

  f.println("# Konfigurace servo testeru");
  f.print("STEP=");
  f.println(clampInt(defaultPowerStep, 1, 100));

  f.print("PULSE=");
  f.println(clampInt(pulseDurationMs, 10, 5000));

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    f.print("PIN");
    f.print(i);
    f.print("=");
    f.println(SERVO_PINS[i]);
  }

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    f.print("DESC");
    f.print(i);
    f.print("=");
    f.println(servoDesc[i]);
  }

  // kalibrační údaje
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    f.print("FT");
    f.print(i);
    f.print("=");
    f.println(servoFullTurnMs[i]);

    f.print("CENTER");
    f.print(i);
    f.print("=");
    f.println(servoCenterDeg[i]);
  }

  // Remote ovladač
  for (uint8_t i = 0; i < REMOTE_BTN_COUNT; i++) {
    f.print("REMOTE");
    f.print(i);
    f.print("=");
    String t = remoteType[i];
    if (t.length() == 0) t = "none";
    t.toLowerCase();
    if (t != "seq" && t != "url" && t != "none") t = "none";
    String val = remoteValue[i];
    val.replace('\r', ' ');
    val.replace('\n', ' ');
    val.trim();
    if (val.length() > 64) val = val.substring(0, 64);
    f.print(t);
    if (t != "none" && val.length() > 0) {
      f.print(":");
      f.print(val);
    }
    f.println();
  }

  f.close();
  Serial.println("Config: ulozeno do LittleFS (config.txt)");
}

// ====== Servo pomocné funkce ======

void ensureServoAttached(uint8_t index) {
  if (index >= SERVO_COUNT) return;
  if (!servoAttached[index]) {
    Servo &s = servos[index];
    s.setPeriodHertz(50); // 50 Hz pro serva
    s.attach(SERVO_PINS[index], SERVO_MIN_US, SERVO_MAX_US);
    servoAttached[index] = true;
  }
}

int powerToPulse(int power) {
  power = clampInt(power, -100, 100);
  long p = map(power, -100, 100, SERVO_MIN_US, SERVO_MAX_US);
  return (int)p;
}

void setServoPower(uint8_t index, int power) {
  if (index >= SERVO_COUNT) return;

  ensureServoAttached(index);

  power = clampInt(power, -100, 100);
  currentPower[index] = power;

  if (power == 0) {
    // jakýkoli ruční zápis power=0 ruší případný naplánovaný pulz
    servoPulseActive[index] = false;
  }

  int pulse = powerToPulse(power);
  servos[index].writeMicroseconds(pulse);
}

void stopServo(uint8_t index) {
  if (index >= SERVO_COUNT) return;
  if (servoAttached[index]) {
    servos[index].detach();
    servoAttached[index] = false;
  }
  currentPower[index] = 0;
  servoPulseActive[index] = false;
}

// Naplánuje zastavení serva po daném čase (ms) – neblokující
void scheduleServoStop(uint8_t index, int durationMs) {
  if (index >= SERVO_COUNT) return;
  durationMs = clampInt(durationMs, 0, 30000);
  if (durationMs <= 0) {
    servoPulseActive[index] = false;
    return;
  }
  servoPulseActive[index] = true;
  servoPulseEndTime[index] = millis() + (uint32_t)durationMs;
}

// Pravidelně volaná funkce z loop(), která kontroluje, zda už nemá
// nějaký naplánovaný pulz skončit, a případně servo vypne.
void processServoPulseScheduler() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (servoPulseActive[i]) {
      if ((long)(now - servoPulseEndTime[i]) >= 0) {
        servoPulseActive[i] = false;
        setServoPower(i, 0);
      }
    }
  }
}

// ===== Kalibrace – výpočet času pro daný úhel =====

// Vrátí dobu (ms), po kterou má běžet servo, aby se otočilo o daný úhel
// při daném absolutním výkonu (1–100 %), podle kalibrovaného času 360°.
int computeDurationForAngle(uint8_t index, int angleDeg, int absPower) {
  if (index >= SERVO_COUNT) return 0;
  if (absPower <= 0) return 0;
  absPower = clampInt(absPower, 1, 100);

  int ft = servoFullTurnMs[index];
  if (ft <= 0) ft = 1000;

  long a = abs(angleDeg);
  float base = (float)ft * ((float)a / 360.0f);   // čas pro daný úhel při 100 %
  float scale = 100.0f / (float)absPower;         // úprava pro výkon
  long dur = (long)(base * scale);

  dur = clampInt((int)dur, 10, 30000);
  return (int)dur;
}

// Otevřená smyčka – otočí servo o zadaný úhel (kladný = jeden směr, záporný = druhý)
// a aktualizuje odhad servoCurrentDeg.
void moveServoByAngle(uint8_t index, int angleDeg, int power) {
  if (index >= SERVO_COUNT) return;
  if (angleDeg == 0) return;

  int absPower = abs(power);
  if (absPower == 0) absPower = 50; // výchozí výkon
  absPower = clampInt(absPower, 1, 100);

  int dir = (angleDeg > 0) ? 1 : -1;
  int cmdPower = (dir > 0) ? absPower : -absPower;

  int dur = computeDurationForAngle(index, angleDeg, absPower);

  // Neblokující varianta – servo rozjedeme a časované vypnutí
  // zajistí plánovač v loop().
  setServoPower(index, cmdPower);
  scheduleServoStop(index, dur);

  servoCurrentDeg[index] += angleDeg;
}

// ====== SEKVENCE – pomocné funkce ======

// adresář, kde jsou sekvence
const char *SEQ_DIR    = "/";
// prefix a přípona sekvenčních souborů
//const char *SEQ_PREFIX = "seq_";
const char *SEQ_EXT    = ".txt";

// Vrátí čistý název souboru bez cesty
String seqJustName(const String &full) {
  int slash = full.lastIndexOf('/');
  if (slash >= 0 && slash + 1 < (int)full.length()) {
    return full.substring(slash + 1);
  }
  return full;
}

// Zpracuje název jako "seq_chuze.txt" → "chuze"
String seqBaseName(const String &just) {
  if (!just.startsWith(SEQ_PREFIX)) return "";
  if (!just.endsWith(SEQ_EXT))      return "";
  int pLen = strlen(SEQ_PREFIX);
  int eLen = strlen(SEQ_EXT);
  return just.substring(pLen, just.length() - eLen);
}

// Vytvoří cestu: "chuze_vpred" → "/seq_chuze_vpred.txt"
String seqPathFromName(const String &name) {
  String p = "/";
  p += SEQ_PREFIX;
  p += name;
  p += SEQ_EXT;
  return p;
}

String makeSeqPath(const String &nameRaw) {
  String name = nameRaw;
  name.trim();
  if (name.length() == 0) name = "noname";

  for (int i = 0; i < name.length(); i++) {
    char c = name[i];
    bool ok = (c >= '0' && c <= '9') ||
              (c >= 'A' && c <= 'Z') ||
              (c >= 'a' && c <= 'z') ||
              c == '_' || c == '-';
    if (!ok) name[i] = '_';
  }

  String path = String(SEQ_PREFIX) + name + String(SEQ_EXT);
  return path; // /seq_xxx.txt
}

// ====== HTTP HANDLERY – UI a základní API ======

void handleRoot() {
  if (!LittleFS.exists("/index.html")) {
    server.send(500, "text/plain", "index.html not found in LittleFS");
    return;
  }
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain", "Cannot open index.html");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}

// ====== OTA .bin přes web ======

void handleUpdateUpload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA upload start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA upload end, size: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

void handleUpdateFinished() {
  bool ok = !Update.hasError();
  if (ok) {
    server.send(200, "text/plain", "Update OK, rebooting...");
    Serial.println("Update OK, rebooting...");
    // okamžitý restart bez blokujícího delay()
    ESP.restart();
  } else {
    server.send(500, "text/plain", "Update FAILED");
    Serial.println("Update FAILED");
  }
}

// ====== Servo API ======

// /api/move?servo=0&dir=1&step=10
// dir: +1 / -1, step: změna výkonu v procentech
// Servo běží pulseDurationMs a pak se zastaví (power = 0)
void handleApiMove() {
  if (!server.hasArg("servo") || !server.hasArg("dir")) {
    server.send(400, "text/plain", "Missing servo or dir");
    return;
  }

  int index = server.arg("servo").toInt();
  int dir   = server.arg("dir").toInt();
  int stepP = server.hasArg("step") ? server.arg("step").toInt() : defaultPowerStep;

  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }
  if (dir == 0) {
    server.send(400, "text/plain", "Invalid dir");
    return;
  }

  stepP = clampInt(stepP, 1, 100);

  int newPower = currentPower[index] + (dir > 0 ? stepP : -stepP);
  newPower = clampInt(newPower, -100, 100);

  setServoPower(index, newPower);

  int dur = clampInt(pulseDurationMs, 10, 5000);
  // Neblokující pulz – servo se po daném čase vypne v plánovači.
  scheduleServoStop(index, dur);

  String resp = "Servo ";
  resp += index;
  resp += " pulse ";
  resp += newPower;
  resp += " % for ";
  resp += dur;
  resp += " ms, then 0 %";
  server.send(200, "text/plain", resp);
}

// /api/move_angle?servo=0&angle=90&power=50
// POZOR: blokující – během pohybu se neobsluhuje webserver ani OTA.
void handleApiMoveAngle() {
  if (!server.hasArg("servo") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Missing servo or angle");
    return;
  }

  int index = server.arg("servo").toInt();
  int angle = server.arg("angle").toInt();
  int power = server.hasArg("power") ? server.arg("power").toInt() : 50;

  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  moveServoByAngle(index, angle, power);

  String resp = "Servo ";
  resp += index;
  resp += " moved by ";
  resp += angle;
  resp += " deg (estimated current = ";
  resp += servoCurrentDeg[index];
  resp += " deg)";
  server.send(200, "text/plain", resp);
}

// /api/center_one?servo=0
void handleApiCenterOne() {
  if (!server.hasArg("servo")) {
    server.send(400, "text/plain", "Missing servo");
    return;
  }

  int index = server.arg("servo").toInt();
  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  setServoPower(index, 0);

  String resp = "Servo ";
  resp += index;
  resp += " centered (power = 0 %)";
  server.send(200, "text/plain", resp);
}

// /api/center_all
void handleApiCenterAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoPower(i, 0);
  }
  server.send(200, "text/plain", "All servos centered (power = 0 %)");
}

// /api/stop_one?servo=0
void handleApiStopOne() {
  if (!server.hasArg("servo")) {
    server.send(400, "text/plain", "Missing servo");
    return;
  }

  int index = server.arg("servo").toInt();
  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  stopServo(index);

  String resp = "Servo ";
  resp += index;
  resp += " stopped (detached, power = 0 %)";
  server.send(200, "text/plain", resp);
}

// /api/stop_all
void handleApiStopAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    stopServo(i);
  }
  server.send(200, "text/plain", "All servos stopped (detached, power = 0 %)");
}

// /api/set_pin?servo=0&gpio=13  (změna GPIO v RAM)
void handleApiSetPin() {
  if (!server.hasArg("servo") || !server.hasArg("gpio")) {
    server.send(400, "text/plain", "Missing servo or gpio");
    return;
  }

  int index = server.arg("servo").toInt();
  int gpio  = server.arg("gpio").toInt();

  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  if (!isValidGpio((uint8_t)gpio)) {
    server.send(400, "text/plain", "Invalid gpio");
    return;
  }

  stopServo(index);
  SERVO_PINS[index] = (uint8_t)gpio;

  String resp = "Servo ";
  resp += index;
  resp += " pin set to GPIO";
  resp += gpio;
  server.send(200, "text/plain", resp);
}

// /api/set_desc?servo=0&desc=Text
void handleApiSetDesc() {
  if (!server.hasArg("servo") || !server.hasArg("desc")) {
    server.send(400, "text/plain", "Missing servo or desc");
    return;
  }

  int index = server.arg("servo").toInt();
  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  String d = server.arg("desc");
  d.replace('\r', ' ');
  d.replace('\n', ' ');
  d.trim();
  if (d.length() > 32) d = d.substring(0, 32);

  servoDesc[index] = d;

  String resp = "Servo ";
  resp += index;
  resp += " desc set to '";
  resp += d;
  resp += "'";
  server.send(200, "text/plain", resp);
}

// /api/save_config?step=10&pulse=50
void handleApiSaveConfig() {
  if (server.hasArg("step")) {
    int step = server.arg("step").toInt();
    defaultPowerStep = clampInt(step, 1, 100);
  }

  if (server.hasArg("pulse")) {
    int pulse = server.arg("pulse").toInt();
    pulseDurationMs = clampInt(pulse, 10, 5000);
  }

  saveConfigToLittleFS();

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    stopServo(i);
  }

  server.send(200, "text/plain",
              "Konfigurace ulozena do LittleFS (config.txt), serva zastavena");
}

// /api/config – vrací step, pulse, pins, popisy a remote mapování
void handleApiConfig() {
  String json = "{\"step\":";
  json += clampInt(defaultPowerStep, 1, 100);
  json += ",\"pulse\":";
  json += clampInt(pulseDurationMs, 10, 5000);
  json += ",\"pins\":[";

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    json += SERVO_PINS[i];
    if (i < SERVO_COUNT - 1) json += ",";
  }
  json += "],\"descs\":[";
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    json += "\"";
    json += jsonEscape(servoDesc[i]);
    json += "\"";
    if (i < SERVO_COUNT - 1) json += ",";
  }
  json += "],\"remote\":[";
  for (uint8_t i = 0; i < REMOTE_BTN_COUNT; i++) {
    json += "{";
    json += "\"type\":\"" + jsonEscape(remoteType[i]) + "\",";
    json += "\"value\":\"" + jsonEscape(remoteValue[i]) + "\"";
    json += "}";
    if (i < REMOTE_BTN_COUNT - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// /api/remote_save?idx=0&type=seq&value=chuze_vpred
void handleApiRemoteSave() {
  if (!server.hasArg("idx") || !server.hasArg("type")) {
    server.send(400, "text/plain", "Missing idx or type");
    return;
  }

  int idx = server.arg("idx").toInt();
  if (idx < 0 || idx >= REMOTE_BTN_COUNT) {
    server.send(400, "text/plain", "Invalid idx");
    return;
  }

  String t = server.arg("type");
  t.toLowerCase();
  if (t != "seq" && t != "url" && t != "none") {
    server.send(400, "text/plain", "Invalid type");
    return;
  }

  String val = server.hasArg("value") ? server.arg("value") : "";
  val.replace('\r', ' ');
  val.replace('\n', ' ');
  val.trim();
  if (val.length() > 64) val = val.substring(0, 64);

  remoteType[idx]  = t;
  remoteValue[idx] = val;

  saveConfigToLittleFS();

  String resp = "Remote btn ";
  resp += idx;
  resp += " set to ";
  resp += t;
  resp += ":";
  resp += val;
  server.send(200, "text/plain", resp);
}

// ===== Kalibrační API =====

// /api/calib_set_fullturn?servo=0&ms=1200
void handleApiCalibSetFullTurn() {
  if (!server.hasArg("servo") || !server.hasArg("ms")) {
    server.send(400, "text/plain", "Missing servo or ms");
    return;
  }

  int index = server.arg("servo").toInt();
  int ms    = server.arg("ms").toInt();

  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  ms = clampInt(ms, 100, 60000);
  servoFullTurnMs[index] = ms;
  saveConfigToLittleFS();

  String resp = "Servo ";
  resp += index;
  resp += " full-turn time set to ";
  resp += ms;
  resp += " ms (100% power)";
  server.send(200, "text/plain", resp);
}

// /api/calib_center_here?servo=0
void handleApiCalibCenterHere() {
  if (!server.hasArg("servo")) {
    server.send(400, "text/plain", "Missing servo");
    return;
  }

  int index = server.arg("servo").toInt();
  if (index < 0 || index >= SERVO_COUNT) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  servoCenterDeg[index]  = servoCurrentDeg[index];
  servoCurrentDeg[index] = 0;
  saveConfigToLittleFS();

  String resp = "Servo ";
  resp += index;
  resp += " center calibrated (centerDeg=";
  resp += servoCenterDeg[index];
  resp += ", current reset to 0)";
  server.send(200, "text/plain", resp);
}

// /api/calib_info – vrátí kalibrační údaje pro všechna serva
void handleApiCalibInfo() {
  String json = "{\"calib\":[";
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"fullTurnMs\":" + String(servoFullTurnMs[i]) + ",";
    json += "\"centerDeg\":" + String(servoCenterDeg[i]) + ",";
    json += "\"currentDeg\":" + String(servoCurrentDeg[i]);
    json += "}";
    if (i < SERVO_COUNT - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// /api/status – stav všech serv
void handleApiStatus() {
  String json = "{\"servos\":[";
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"power\":" + String(currentPower[i]) + ",";
    json += "\"attached\":" + String(servoAttached[i] ? "true" : "false");
    json += "}";
    if (i < SERVO_COUNT - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// ====== Správce souborů LittleFS ======

// /api/fs_list
void handleApiFsList() {
  File root = LittleFS.open("/");
  if (!root) {
    server.send(500, "text/plain", "LittleFS root open failed");
    return;
  }

  String json = "{\"files\":[";
  bool first = true;

  File file = root.openNextFile();
  while (file) {
    if (!first) json += ",";
    first = false;

    json += "{\"name\":\"";
    json += file.name();
    json += "\",\"size\":";
    json += file.size();
    json += "}";

    file = root.openNextFile();
  }

  json += "]}";
  server.send(200, "application/json", json);
}

// /api/fs_get?path=/something
void handleApiFsGet() {
  if (!server.hasArg("path")) {
    server.send(400, "text/plain", "Missing path");
    return;
  }

  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;

  if (!LittleFS.exists(path)) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f) {
    server.send(500, "text/plain", "Cannot open file");
    return;
  }

  server.streamFile(f, "application/octet-stream");
  f.close();
}

// /api/fs_put?path=/something   (POST, text/plain)
void handleApiFsPut() {
  if (!server.hasArg("path")) {
    server.send(400, "text/plain", "Missing path");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;

  String body = server.arg("plain");

  File f = LittleFS.open(path, "w");
  if (!f) {
    server.send(500, "text/plain", "Cannot write file");
    return;
  }
  f.print(body);
  f.close();

  server.send(200, "text/plain", "File saved");
}

// /api/fs_delete?path=/something
void handleApiFsDelete() {
  if (!server.hasArg("path")) {
    server.send(400, "text/plain", "Missing path");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;

  if (LittleFS.exists(path)) {
    LittleFS.remove(path);
    server.send(200, "text/plain", "File deleted");
  } else {
    server.send(404, "text/plain", "File not found");
  }
}

// ====== Sekvence API ======

// /api/seq_list -> {"seq":["nazev1","nazev2",...]}
void handleApiSeqList() {
  File root = LittleFS.open("/");
  if (!root) {
    server.send(500, "text/plain", "LittleFS root open failed");
    return;
  }

  String json = "{\"seq\":[";
  bool first = true;

  File file = root.openNextFile();
  while (file) {
    String fname = file.name(); // může být "/seq_xxx.txt" nebo "seq_xxx.txt"

    // odstraníme případné počáteční "/" (rozdíl mezi výpisem v LittleFS a ukládáním)
    if (fname.startsWith("/")) {
      fname.remove(0, 1);
    }

    // hledáme soubory seq_<name>.txt v kořeni
    if (fname.startsWith("seq_") && fname.endsWith(SEQ_EXT)) {
      // base = část mezi "seq_" a ".txt"
      String base = fname.substring(4, fname.length() - SEQ_EXT_LEN);
      if (!first) json += ",";
      first = false;
      json += "\"" + base + "\"";
    }
    file = root.openNextFile();
  }

  json += "]}";
  server.send(200, "application/json", json);
}

// /api/seq_get?name=xxx
void handleApiSeqGet() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String name = server.arg("name");
  String path = makeSeqPath(name);

  if (!LittleFS.exists(path)) {
    server.send(404, "text/plain", "Sequence not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f) {
    server.send(500, "text/plain", "Cannot open sequence file");
    return;
  }

  String content = f.readString();
  f.close();

  server.send(200, "text/plain", content);
}

// /api/seq_save?name=xxx   (POST, text/plain body)
void handleApiSeqSave() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String name = server.arg("name");
  String path = makeSeqPath(name);
  String body = server.arg("plain");

  File f = LittleFS.open(path, "w");
  if (!f) {
    server.send(500, "text/plain", "Cannot write sequence file");
    return;
  }

  f.print(body);
  f.close();

  server.send(200, "text/plain", "Sequence saved");
}

// /api/seq_delete?name=xxx
void handleApiSeqDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String name = server.arg("name");
  String path = makeSeqPath(name);

  if (LittleFS.exists(path)) {
    LittleFS.remove(path);
    server.send(200, "text/plain", "Sequence deleted");
  } else {
    server.send(404, "text/plain", "Sequence not found");
  }
}

// /api/seq_run?name=xxx   (neblokující – sekvence běží na pozadí)
void handleApiSeqRun() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String name = server.arg("name");
  String path = makeSeqPath(name);

  if (!LittleFS.exists(path)) {
    server.send(404, "text/plain", "Sequence not found");
    return;
  }

  // zastavíme případnou předchozí sekvenci a vynulujeme serva
  seqRunning    = false;
  seqStepCount  = 0;
  seqStepIndex  = 0;
  seqStepActive = false;
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoPower(i, 0);
  }

  File f = LittleFS.open(path, "r");
  if (!f) {
    server.send(500, "text/plain", "Cannot open sequence file");
    return;
  }

  uint16_t count = 0;

  while (f.available() && count < MAX_SEQ_STEPS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) {
      Serial.print("Seq: invalid line: ");
      Serial.println(line);
      continue;
    }

    int idx   = line.substring(0, c1).toInt();
    int power = line.substring(c1 + 1, c2).toInt();
    int dur   = line.substring(c2 + 1).toInt();

    if (idx < 0 || idx >= SERVO_COUNT) {
      Serial.print("Seq: invalid servo index: ");
      Serial.println(line);
      continue;
    }

    power = clampInt(power, -100, 100);
    dur   = clampInt(dur, 0, 30000); // max 30 s

    Serial.print("Seq cmd: servo=");
    Serial.print(idx);
    Serial.print(" power=");
    Serial.print(power);
    Serial.print(" dur=");
    Serial.println(dur);

    SeqStep &st = seqSteps[count++];
    st.servo      = (uint8_t)idx;
    st.power      = (int16_t)power;
    st.durationMs = (uint32_t)dur;
  }

  f.close();

  if (count == 0) {
    server.send(400, "text/plain", "Sequence empty or invalid");
    return;
  }

  seqStepCount  = count;
  seqStepIndex  = 0;
  seqStepActive = false;
  seqRunning    = true;
  seqStepEndTime = millis();

  server.send(200, "text/plain",
              "Sequence scheduled (běží asynchronně na pozadí)");

  Serial.print("Sequence scheduled: ");
  Serial.print(path);
  Serial.print(" steps=");
  Serial.println(seqStepCount);
}

// Plánovač kroků sekvence – volat pravidelně z loop()
void processSequenceScheduler() {
  if (!seqRunning) return;

  unsigned long now = millis();

  if (!seqStepActive) {
    // žádný krok neběží – zkusíme spustit další
    if (seqStepIndex >= seqStepCount) {
      // konec sekvence – vypneme všechna serva
      for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        setServoPower(i, 0);
      }
      seqRunning = false;
      Serial.println("Sequence finished.");
      return;
    }

    SeqStep &st = seqSteps[seqStepIndex++];

    Serial.print("Seq step: servo=");
    Serial.print(st.servo);
    Serial.print(" power=");
    Serial.print(st.power);
    Serial.print(" dur=");
    Serial.println(st.durationMs);

    setServoPower(st.servo, st.power);

    if (st.durationMs > 0) {
      seqStepActive  = true;
      seqStepEndTime = now + st.durationMs;
    } else {
      // krok bez prodlevy – další krok se připraví v dalším průchodu
      seqStepActive  = false;
      seqStepEndTime = now;
    }
    return;
  }

  // nějaký krok právě běží – zkontrolujeme, zda nevypršel čas
  if ((long)(now - seqStepEndTime) >= 0) {
    seqStepActive = false;
  }
}

// 404
void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ====== SETUP & LOOP ======

void setupServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    currentPower[i]  = 0;
    servoAttached[i] = false;
  }
}

void setupWiFiAndServer() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("ESP32-ServoTester360");

  bool res = wm.autoConnect("ServoTesterAP");

  if (!res) {
    Serial.println("WiFi failed, restarting...");
    // okamžitý restart bez blokujícího čekání
    ESP.restart();
  } else {
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // ArduinoOTA pro nahrávání z Arduino IDE (síťový port)
  ArduinoOTA.setHostname("ESP32-ServoTester360");
  ArduinoOTA.onStart([]() {
    Serial.println("ArduinoOTA: Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("ArduinoOTA: End");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("ArduinoOTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("ArduinoOTA ready.");

  // HTTP routy
  server.on("/",               HTTP_GET, handleRoot);
  server.on("/api/move",       HTTP_GET, handleApiMove);
  server.on("/api/move_angle", HTTP_GET, handleApiMoveAngle);
  server.on("/api/center_one", HTTP_GET, handleApiCenterOne);
  server.on("/api/center_all", HTTP_GET, handleApiCenterAll);
  server.on("/api/stop_one",   HTTP_GET, handleApiStopOne);
  server.on("/api/stop_all",   HTTP_GET, handleApiStopAll);
  server.on("/api/set_pin",    HTTP_GET, handleApiSetPin);
  server.on("/api/set_desc",   HTTP_GET, handleApiSetDesc);
  server.on("/api/save_config",HTTP_GET, handleApiSaveConfig);
  server.on("/api/status",     HTTP_GET, handleApiStatus);
  server.on("/api/config",     HTTP_GET, handleApiConfig);

  // kalibrační API
  server.on("/api/calib_set_fullturn", HTTP_GET, handleApiCalibSetFullTurn);
  server.on("/api/calib_center_here",  HTTP_GET, handleApiCalibCenterHere);
  server.on("/api/calib_info",         HTTP_GET, handleApiCalibInfo);

  // remote ovladač
  server.on("/api/remote_save",HTTP_GET, handleApiRemoteSave);

  // sekvence
  server.on("/api/seq_list",   HTTP_GET, handleApiSeqList);
  server.on("/api/seq_get",    HTTP_GET, handleApiSeqGet);
  server.on("/api/seq_delete", HTTP_GET, handleApiSeqDelete);
  server.on("/api/seq_run",    HTTP_GET, handleApiSeqRun);
  server.on("/api/seq_save",   HTTP_POST, handleApiSeqSave);

  // LittleFS file manager
  server.on("/api/fs_list",    HTTP_GET,  handleApiFsList);
  server.on("/api/fs_get",     HTTP_GET,  handleApiFsGet);
  server.on("/api/fs_put",     HTTP_POST, handleApiFsPut);
  server.on("/api/fs_delete",  HTTP_GET,  handleApiFsDelete);

  // OTA .bin z webu
  server.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 Servo Tester – SG90 360° + LittleFS + sekvence startuje...");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed, pouziji se pouze default konfigurace");
  }

  loadOrInitConfig();
  setupServos();
  setupWiFiAndServer();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();

  // neblokující plánovače pro pulzy a sekvence
  processServoPulseScheduler();
  processSequenceScheduler();
}
