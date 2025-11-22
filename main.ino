/*
  ESP32 DevKit – 6× kontinuální servo SG90 360°
  + WiFiManager + WebServer + LittleFS config + sekvence

  Kontinuální SG90 360°:
  - Řídíme SMĚR a VÝKON, ne polohu.
  - Parametr "power" = -100…+100:
      0     = stop (cca 1500 µs)
      +100  = plná rychlost jedním směrem
      -100  = plná rychlost opačným směrem

  LittleFS:
  - /index.html        – web UI (nahraješ přes LittleFS uploader, složka /data)
  - /config.txt        – konfigurace kroku výkonu + mapování servo→GPIO
      STEP=10
      PIN0=13
      PIN1=14
      ...
      PIN5=32
  - /seq_<name>.txt    – sekvence příkazů pro serva (text, jeden příkaz na řádek)
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
const int SERVO_NEUTRAL_US = 1500;  // ideálně stop (střed) – zatím nepoužito per-servo

// ====== LittleFS KONFIG ======
const char *CONFIG_PATH = "/config.txt";

// Sekvence – soubory typu /seq_<name>.txt
const char *SEQ_PREFIX      = "/seq_";
const size_t SEQ_PREFIX_LEN = 5;     // délka "/seq_"
const char *SEQ_EXT         = ".txt";
const size_t SEQ_EXT_LEN    = 4;

// ====== STAV SERV ======
Servo  servos[SERVO_COUNT];

// Výkon v procentech -100..+100 (interní reprezentace)
int  currentPower[SERVO_COUNT];   // -100..100, 0 = stop
bool servoAttached[SERVO_COUNT];  // jestli je servo připojeno (PWM aktivní)

// Krok výkonu v procentech (změna power při LEVO/PRAVO tlačítku)
int defaultPowerStep = 10;        // 10 % na jedno kliknutí

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

// LittleFS: načtení nebo inicializace defaultů
void loadOrInitConfig() {
  // výchozí hodnoty
  defaultPowerStep = 10;

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
    }
  }

  f.close();
}

// LittleFS: uložení konfigurace do textového souboru
void saveConfigToLittleFS() {
  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) {
    Serial.println("Config: nelze zapsat config.txt");
    return;
  }

  f.println("# Konfigurace servo testeru");
  f.print("STEP=");
  f.println(clampInt(defaultPowerStep, 1, 100));

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    f.print("PIN");
    f.print(i);
    f.print("=");
    f.println(SERVO_PINS[i]);
  }

  f.close();
  Serial.println("Config: ulozeno do LittleFS (config.txt)");
}

// Zajistí, že servo je připojeno (PWM běží) – pokud ještě nebylo, připojí ho.
void ensureServoAttached(uint8_t index) {
  if (index >= SERVO_COUNT) return;
  if (!servoAttached[index]) {
    Servo &s = servos[index];
    s.setPeriodHertz(50); // 50 Hz pro serva
    s.attach(SERVO_PINS[index], SERVO_MIN_US, SERVO_MAX_US);
    servoAttached[index] = true;
  }
}

// Přepočet power -100..100 na puls v mikrosekundách
int powerToPulse(int power) {
  power = clampInt(power, -100, 100);
  long p = map(power, -100, 100, SERVO_MIN_US, SERVO_MAX_US);
  return (int)p;
}

// Nastaví výkon daného serva (a připojí ho, pokud není attach)
void setServoPower(uint8_t index, int power) {
  if (index >= SERVO_COUNT) return;

  ensureServoAttached(index);

  power = clampInt(power, -100, 100);
  currentPower[index] = power;

  int pulse = powerToPulse(power);
  servos[index].writeMicroseconds(pulse);
}

// Odpojí servo (PWM se vypne, servo je volné) + vynuluje power
void stopServo(uint8_t index) {
  if (index >= SERVO_COUNT) return;
  if (servoAttached[index]) {
    servos[index].detach();
    servoAttached[index] = false;
  }
  currentPower[index] = 0;  // logický stav = stop
}

// ====== SEKVENCE – pomocné funkce v LittleFS ======

String makeSeqPath(const String &nameRaw) {
  // jednoduché "čištění" jména: nepovolené znaky nahradíme podtržítkem
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
  return path; // např. /seq_walk.txt
}

// ====== HTTP HANDLERY – UI a základní API ======

// /  -> servíruje /index.html z LittleFS
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

// API: /api/move?servo=0&dir=1&step=10
// dir: +1 / -1, step: změna výkonu v procentech
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

  String resp = "Servo ";
  resp += index;
  resp += " power set to ";
  resp += currentPower[index];
  resp += " %";
  server.send(200, "text/plain", resp);
}

// API: /api/center_one?servo=0  -> power = 0 % (stop, ale attach)
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

// API: /api/center_all -> všem power = 0 %
void handleApiCenterAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoPower(i, 0);
  }
  server.send(200, "text/plain", "All servos centered (power = 0 %)");
}

// API: /api/stop_one?servo=0 -> detach + power = 0
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

// API: /api/stop_all -> detach + power = 0 u všech
void handleApiStopAll() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    stopServo(i);
  }
  server.send(200, "text/plain", "All servos stopped (detached, power = 0 %)");
}

// API: /api/set_pin?servo=0&gpio=13 -> změna GPIO pro servo
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

  // pro jistotu servo zastavíme a odpojíme
  stopServo(index);
  SERVO_PINS[index] = (uint8_t)gpio;

  String resp = "Servo ";
  resp += index;
  resp += " pin set to GPIO";
  resp += gpio;
  server.send(200, "text/plain", resp);
}

// API: /api/save_config?step=10
// Uloží SERVO_PINS[] + defaultPowerStep do LittleFS a zastaví všechna serva
void handleApiSaveConfig() {
  if (server.hasArg("step")) {
    int step = server.arg("step").toInt();
    defaultPowerStep = clampInt(step, 1, 100);
  }

  saveConfigToLittleFS();

  // po uložení zastavíme všechna serva
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    stopServo(i);
  }

  server.send(200, "text/plain",
              "Konfigurace ulozena do LittleFS (config.txt), serva zastavena");
}

// API: /api/status – JSON se stavem všech serv
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

// ====== HTTP HANDLERY – sekvence ======

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
    String fname = file.name(); // např. "/seq_walk.txt"
    if (fname.startsWith(SEQ_PREFIX) && fname.endsWith(SEQ_EXT)) {
      String base = fname.substring(SEQ_PREFIX_LEN,
                                    fname.length() - SEQ_EXT_LEN);
      if (!first) json += ",";
      first = false;
      json += "\"" + base + "\"";
    }
    file = root.openNextFile();
  }

  json += "]}";
  server.send(200, "application/json", json);
}

// /api/seq_get?name=nazev -> vrátí obsah sekvence (text)
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

// /api/seq_save?name=nazev (POST, body = obsah)
// uloží nebo přepíše sekvenci
void handleApiSeqSave() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String name = server.arg("name");
  String path = makeSeqPath(name);
  String body = server.arg("plain"); // raw body

  File f = LittleFS.open(path, "w");
  if (!f) {
    server.send(500, "text/plain", "Cannot write sequence file");
    return;
  }

  f.print(body);
  f.close();

  server.send(200, "text/plain", "Sequence saved");
}

// /api/seq_delete?name=nazev -> smaže soubor sekvence
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

// /api/seq_run?name=nazev -> blokující spuštění sekvence
// Formát sekvence (textový soubor):
//   # komentář
//   servo,power,duration_ms
//   0,50,1000
//   0,0,500
//   1,-30,1500
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

  File f = LittleFS.open(path, "r");
  if (!f) {
    server.send(500, "text/plain", "Cannot open sequence file");
    return;
  }

  // Odpověď pošleme hned, aby prohlížeč nečekal na konec sekvence.
  server.send(200, "text/plain", "Sequence running (blokuje ESP, UI chvilku nereaguje)");

  Serial.print("Sequence run: ");
  Serial.println(path);

  while (f.available()) {
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
    dur   = clampInt(dur, 0, 30000); // max 30 s na příkaz

    Serial.print("Seq cmd: servo=");
    Serial.print(idx);
    Serial.print(" power=");
    Serial.print(power);
    Serial.print(" dur=");
    Serial.println(dur);

    setServoPower(idx, power);
    if (dur > 0) {
      delay(dur);   // BLOKUJE – jednoduchá implementace pro testování
    }
  }

  f.close();
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
    Serial.println("WiFi failed, restarting in 5s...");
    delay(5000);
    ESP.restart();
  } else {
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // HTTP routy
  server.on("/",               HTTP_GET, handleRoot);
  server.on("/api/move",       HTTP_GET, handleApiMove);
  server.on("/api/center_one", HTTP_GET, handleApiCenterOne);
  server.on("/api/center_all", HTTP_GET, handleApiCenterAll);
  server.on("/api/stop_one",   HTTP_GET, handleApiStopOne);
  server.on("/api/stop_all",   HTTP_GET, handleApiStopAll);
  server.on("/api/set_pin",    HTTP_GET, handleApiSetPin);
  server.on("/api/save_config",HTTP_GET, handleApiSaveConfig);
  server.on("/api/status",     HTTP_GET, handleApiStatus);

  // sekvence
  server.on("/api/seq_list",   HTTP_GET, handleApiSeqList);
  server.on("/api/seq_get",    HTTP_GET, handleApiSeqGet);
  server.on("/api/seq_delete", HTTP_GET, handleApiSeqDelete);
  server.on("/api/seq_run",    HTTP_GET, handleApiSeqRun);
  server.on("/api/seq_save",   HTTP_POST, handleApiSeqSave);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("ESP32 Servo Tester – SG90 360° + LittleFS + sekvence startuje...");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed, pouziji se pouze default konfigurace");
  }

  loadOrInitConfig();   // načte STEP + mapování GPIO z config.txt (nebo nechá defaulty)
  setupServos();
  setupWiFiAndServer();
}

void loop() {
  server.handleClient();
}
