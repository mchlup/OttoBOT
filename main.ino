/*
  ESP32 DevKit – 6× kontinuální servo SG90 360° + WiFiManager + WebServer + EEPROM config

  Verze pro kontinuální serva:
  - POZOR: Tohle NENÍ polohové řízení (°), ale řízení SMĚRU a VÝKONU.
  - Kontinuální SG90 360° neumí držet úhel, jen se točit jako motorek s převodem.

  Logika:
  - Po spuštění se serva NEPŘESTAVUJÍ, jsou v klidu (neposílá se PWM).
  - Parametr "power" = -100…+100:
      - 0   = stop (cca 1500 µs)
      - +100 = plná rychlost doprava
      - -100 = plná rychlost doleva
  - CENTER = power = 0 (servo zastavené, ale attach = držíme neutrál pulzem)
  - STOP   = detach (servo volné, PWM se neposílá)

  WebUI:
  - Globální "Krok výkonu (%)" – o kolik % se změní výkon při LEVO/PRAVO.
  - U každého serva: LEVO / CENTER / PRAVO / STOP.
  - CENTER ALL, STOP ALL.
  - Zpětná vazba: výkon v %, stav AKTIVNÍ/STOP (aktualizace každé 2 s).
  - Konfigurace GPIO:
      - Pro každé servo výběr GPIO z nabídky.
      - Po změně se servo odpojí (STOP) a nastaví se nový pin.
      - Tlačítko „Uložit konfiguraci“ uloží GPIO + krok výkonu do EEPROM.

  Potřebné knihovny:
  - ESP32Servo (ESP32Servo.h)
  - WiFiManager (WiFiManager.h)
  - EEPROM (EEPROM.h) – emulovaná EEPROM v flashi ESP32
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>   // https://github.com/tzapu/WiFiManager
#include <ESP32Servo.h>    // https://github.com/madhephaestus/ESP32Servo
#include <SPIFFS.h>        // SPIFFS pro ukládání konfigurace

// ====== KONFIGURACE SERV ======
const uint8_t SERVO_COUNT = 6;

// Výchozí (kompilované) mapování servo -> GPIO pro ESP32 DevKit
// Reálné mapování se při startu přepíše z EEPROM, pokud je v ní platná konfigurace.
uint8_t SERVO_PINS[SERVO_COUNT] = {
  13, // Servo 0
  14, // Servo 1
  25, // Servo 2
  26, // Servo 3
  27, // Servo 4
  32  // Servo 5
};

// Použitelné GPIO pro serva na ESP32 DevKit (PWM schopné, ne flash/USB/UART0)
// Tohle je seznam, který nabízíme v UI pro výběr:
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
// Tyto hodnoty jsou typické, ale neutrál může být mírně posunutý.
const int SERVO_MIN_US     = 1000;  // plná rychlost jedním směrem
const int SERVO_MAX_US     = 2000;  // plná rychlost druhým směrem
const int SERVO_NEUTRAL_US = 1500;  // ideálně stop (střed)

// ====== SPIFFS KONFIG ======
const char *CONFIG_PATH = "/config.txt";

// ====== STAV SERV ======
Servo servos[SERVO_COUNT];

// Výkon v procentech -100..+100 (interní reprezentace)
int  currentPower[SERVO_COUNT];   // -100..100, 0 = stop
bool servoAttached[SERVO_COUNT];  // jestli je servo připojeno (PWM aktivní)

// Krok výkonu v procentech (změna power při LEVO/PRAVO tlačítku)
// Hodnota se načítá/ukládá do EEPROM
int defaultPowerStep = 10;        // 10 % na jedno kliknutí

WebServer server(80);
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

// SPIFFS: načtení nebo inicializace defaultů
void loadOrInitConfig() {
  // výchozí hodnoty
  defaultPowerStep = 10;

  if (!SPIFFS.exists(CONFIG_PATH)) {
    // žádná konfigurace – zůstanou kompilované defaulty SERVO_PINS + STEP=10
    return;
  }

  File f = SPIFFS.open(CONFIG_PATH, "r");
  if (!f) {
    Serial.println("Config: nelze otevrit config.txt, pouziji se defaulty");
    return;
  }

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
      // očekávaný formát: PIN<index>=<gpio>, např. PIN0=13
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

// SPIFFS: uložení konfigurace do textového souboru
void saveConfigToSPIFFS() {
  File f = SPIFFS.open(CONFIG_PATH, "w");
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
  Serial.println("Config: ulozeno do SPIFFS (config.txt)");
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
  // lineární mapování -100..100 -> SERVO_MIN_US..SERVO_MAX_US
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

// ====== HTTP HANDLERY ======

// Jednoduché HTML WebUI
void handleRoot() {
  String html;

  html += F(
    "<!DOCTYPE html>"
    "<html lang='cs'>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>ESP32 Servo Tester (kontinuální)</title>"
    "<style>"
      "body{font-family:Arial,sans-serif;margin:0;padding:1rem;background:#111;color:#eee;}"
      "h1{text-align:center;}"
      ".card{background:#222;border-radius:8px;padding:1rem;margin:0.5rem auto;max-width:800px;box-shadow:0 0 10px rgba(0,0,0,0.5);}"
      ".servo-row{display:flex;align-items:center;justify-content:space-between;margin:0.3rem 0;gap:0.5rem;flex-wrap:wrap;}"
      ".servo-label{min-width:150px;}"
      "button{padding:0.4rem 0.8rem;border:none;border-radius:4px;margin:0.1rem;cursor:pointer;background:#444;color:#eee;}"
      "button:hover{background:#666;}"
      "input,select{padding:0.2rem;border-radius:4px;border:1px solid #555;background:#111;color:#eee;}"
      "input{width:4rem;text-align:center;}"
      ".center-btn{background:#0066aa;}"
      ".center-btn:hover{background:#0099ff;}"
      ".stop-btn{background:#aa3300;}"
      ".stop-btn:hover{background:#ff4400;}"
      ".small{font-size:0.85rem;color:#aaa;}"
    "</style>"
    "</head>"
    "<body>"
    "<h1>ESP32 Servo Tester – SG90 360°</h1>"
    "<div class='card'>"
    "<h2>Globální nastavení</h2>"
    "<div>"
      "<label>Krok výkonu (%): <input id='stepPower' type='number' min='1' max='100' value='"
  );

  html += String(defaultPowerStep);
  html += F(
      "'></label><br>"
      "<p class='small'>LEVO/PRAVO mění výkon o daný počet procent. 0&nbsp;% = STOP, ±100&nbsp;% = plná rychlost.</p>"
      "<button class='center-btn' onclick='centerAll()'>CENTER ALL (STOP)</button>"
      "<button class='stop-btn' onclick='stopAll()'>STOP ALL (DETACH)</button>"
    "</div>"
    "</div>"
  );

  // Konfigurace GPIO
  html += F(
    "<div class='card'>"
    "<h2>Konfigurace GPIO</h2>"
    "<div>"
    "<p class='small'>Vyber GPIO pro jednotlivá serva. Po změně GPIO se dané servo zastaví (STOP). "
    "Pro trvalé uložení konfigurace (včetně kroku výkonu) klikni na &bdquo;Uložit konfiguraci&ldquo;.</p>"
  );

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    html += "<div class='servo-row'>"
              "<div class='servo-label'>Servo " + String(i) + " GPIO:</div>"
              "<div><select id='servo_pin_" + String(i) + "' onchange='setServoPin(" + String(i) + ")'>";

    for (uint8_t g = 0; g < GPIO_OPTION_COUNT; g++) {
      uint8_t gpio = GPIO_OPTIONS[g];
      html += "<option value='" + String(gpio) + "'";
      if (gpio == SERVO_PINS[i]) {
        html += " selected";
      }
      html += ">GPIO" + String(gpio) + "</option>";
    }

    html += "</select></div></div>";
  }

  html += F(
    "<button class='center-btn' onclick='saveConfig()'>Uložit konfiguraci</button>"
    "</div>"
    "</div>"
  );

  // Ovládání serv
  html += F(
    "<div class='card'>"
    "<h2>Serva</h2>"
  );

  // Dynamicky vytvořit řádky pro 6 serv
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    html += "<div class='servo-row'>"
              "<div class='servo-label'>Servo " + String(i) +
              " <span id='servo_state_" + String(i) + "' class='small'>(stav neznámý)</span></div>"
              "<div>"
                "<button onclick='moveServo(" + String(i) + ",-1)'>&#9664; LEVO</button>"
                "<button onclick='centerServo(" + String(i) + ")'>CENTER (0&nbsp;%)</button>"
                "<button onclick='moveServo(" + String(i) + ",1)'>PRAVO &#9654;</button>"
                "<button class='stop-btn' onclick='stopServoBtn(" + String(i) + ")'>STOP</button>"
              "</div>"
            "</div>";
  }

  html += F(
    "</div>"
    "<script>"
    "function getStep(){"
      "let v=parseInt(document.getElementById('stepPower').value);"
      "if(isNaN(v)) v=10;"
      "if(v<1) v=1;"
      "if(v>100) v=100;"
      "return v;"
    "}"
    "function moveServo(index,dir){"
      "let step=getStep();"
      "let url='/api/move?servo='+index+'&dir='+dir+'&step='+step;"
      "fetch(url).then(_=>refreshStatus()).catch(e=>console.log(e));"
    "}"
    "function centerServo(index){"
      "let url='/api/center_one?servo='+index;"
      "fetch(url).then(_=>refreshStatus()).catch(e=>console.log(e));"
    "}"
    "function centerAll(){"
      "let url='/api/center_all';"
      "fetch(url).then(_=>refreshStatus()).catch(e=>console.log(e));"
    "}"
    "function stopServoBtn(index){"
      "let url='/api/stop_one?servo='+index;"
      "fetch(url).then(_=>refreshStatus()).catch(e=>console.log(e));"
    "}"
    "function stopAll(){"
      "let url='/api/stop_all';"
      "fetch(url).then(_=>refreshStatus()).catch(e=>console.log(e));"
    "}"
    "function setServoPin(index){"
      "let sel=document.getElementById('servo_pin_'+index);"
      "if(!sel) return;"
      "let gpio=sel.value;"
      "let url='/api/set_pin?servo='+index+'&gpio='+gpio;"
      "fetch(url).then(_=>refreshStatus()).catch(e=>console.log(e));"
    "}"
    "function saveConfig(){"
      "let step=getStep();"
      "let url='/api/save_config?step='+step;"
      "fetch(url)"
        ".then(r=>r.text())"
        ".then(t=>{alert(t);})"
        ".catch(e=>console.log(e));"
    "}"
    "function refreshStatus(){"
      "fetch('/api/status')"
        ".then(r=>r.json())"
        ".then(data=>{"
          "if(!data.servos) return;"
          "data.servos.forEach(s=>{"
            "let el=document.getElementById('servo_state_'+s.index);"
            "if(el){"
              "let st=s.attached?'AKTIVNÍ':'STOP';"
              "el.textContent='(výkon '+s.power+' %, '+st+')';"
            "}"
          "});"
        "})"
        ".catch(e=>console.log(e));"
    "}"
    "window.addEventListener('load',()=>{"
      "refreshStatus();"
      "setInterval(refreshStatus,2000);"
    "});"
    "</script>"
    "</body>"
    "</html>"
  );

  server.send(200, "text/html", html);
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

// API: /api/set_pin?servo=0&gpio=13 -> přiřazení GPIO k servu
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

  // Při změně pinu servo zastavíme a odpojíme pro bezpečnost
  stopServo(index);

  SERVO_PINS[index] = (uint8_t)gpio;

  String resp = "Servo ";
  resp += index;
  resp += " pin set to GPIO";
  resp += gpio;
  server.send(200, "text/plain", resp);
}

// API: /api/save_config?step=10 -> uloží SERVO_PINS[] + defaultPowerStep do EEPROM
void handleApiSaveConfig() {
  if (server.hasArg("step")) {
    int step = server.arg("step").toInt();
    defaultPowerStep = clampInt(step, 1, 100);
  }

  saveConfigToSPIFFS();

  server.send(200, "text/plain", "Konfigurace uložena do SPIFFS (config.txt)");
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

// 404
void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ====== SETUP & LOOP ======

void setupServos() {
  // Připravit PWM timery
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Výchozí interní stav – serva v klidu, odpojena
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    currentPower[i]  = 0;     // 0 % = stop
    servoAttached[i] = false; // servo není připojeno -> klid
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
  server.on("/",            HTTP_GET, handleRoot);
  server.on("/api/move",    HTTP_GET, handleApiMove);
  server.on("/api/center_one", HTTP_GET, handleApiCenterOne);
  server.on("/api/center_all", HTTP_GET, handleApiCenterAll);
  server.on("/api/stop_one",   HTTP_GET, handleApiStopOne);
  server.on("/api/stop_all",   HTTP_GET, handleApiStopAll);
  server.on("/api/set_pin",    HTTP_GET, handleApiSetPin);
  server.on("/api/save_config",HTTP_GET, handleApiSaveConfig);
  server.on("/api/status",     HTTP_GET, handleApiStatus);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("ESP32 Servo Tester – SG90 360° startuje...");

  // inicializace SPIFFS (true = případně naformátuje při první chybě)
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed, pouziji se pouze default konfigurace");
  }

  loadOrInitConfig();   // načti mapování GPIO a krok výkonu ze SPIFFS (nebo dej defaulty)
  setupServos();        // serva zůstávají fyzicky v klidu (není attach)
  setupWiFiAndServer();
}

void loop() {
  server.handleClient();
}
