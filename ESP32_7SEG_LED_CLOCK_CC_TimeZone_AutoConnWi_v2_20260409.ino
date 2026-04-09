/*
  ESP32 Clock + Thermometer (DS18B20) + 4x7seg (CC) via 74HC595 + ULN2803
  - Display refresh in dedicated high-priority task on Core 0 (no blanking)
  - DS18B20 handled asynchronously (no blocking)
  - Time with DST: configTzTime + getLocalTime (no restart needed)
  - WiFi configuration: AutoConnect captive portal (retainPortal)
  - OTA update via AutoConnectOTA
  - Status endpoint: /status
  - mDNS: esp32-clock.local (best-effort)
  - Brightness: OE pin PWM + optional auto brightness from LDR (ADC)

*/

#include <Arduino.h>
// ===== WiFi / Portal / OTA =====
#include <WiFi.h>
#include <WebServer.h>
#include <AutoConnect.h>
#include <AutoConnectOTA.h>
#include <ESPmDNS.h>

// ===== Time =====
#include <time.h>

// ===== DS18B20 =====
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== Preferences =====
#include <Preferences.h>

// -----------------------------------------------------------------------------
// Pinout (YOUR PROVIDED PINS)
// -----------------------------------------------------------------------------
static const int PIN_595_CLK   = 12;   // SRCLK
static const int PIN_595_LATCH = 13;   // RCLK
static const int PIN_595_DATA  = 14;   // SER

static const int PIN_595_OE    = 27;   // OE (PWM brightness). NOTE: OE is active LOW on 74HC595.
static const int PIN_LDR_ADC   = 34;   // LDR analog input (ADC1)
static const int PIN_ONEWIRE   = 15;   // DS18B20 data
static const int PIN_LED       = 2;    // On-board LED

// Digit select pins (to ULN2803 inputs) for 4 digits (common cathode)
// Order: leftmost -> rightmost
static const int PIN_DIGIT_0 = 32;   // 1st digit (ULN2803 pin 1)
static const int PIN_DIGIT_1 = 33;   // 2nd digit (ULN2803 pin 2)
static const int PIN_DIGIT_2 = 25;   // 3rd digit (ULN2803 pin 3)
static const int PIN_DIGIT_3 = 26;   // 4th digit (ULN2803 pin 4)


// Your info: enabling a digit is HIGH
static const bool DIGIT_ENABLE_HIGH = true;

// -----------------------------------------------------------------------------
// Display configuration
// -----------------------------------------------------------------------------
static const uint8_t SEG_A  = 1 << 0;
static const uint8_t SEG_B  = 1 << 1;
static const uint8_t SEG_C  = 1 << 2;
static const uint8_t SEG_D  = 1 << 3;
static const uint8_t SEG_E  = 1 << 4;
static const uint8_t SEG_F  = 1 << 5;
static const uint8_t SEG_G  = 1 << 6;
static const uint8_t SEG_DP = 1 << 7;

// 7-seg font for digits 0-9 (A..G, no DP)
// Bit layout: 0bDPGFEDCBA
static const uint8_t FONT_HEX[16] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,         // 0
  SEG_B | SEG_C,                                         // 1
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                 // 2
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                 // 3
  SEG_B | SEG_C | SEG_F | SEG_G,                         // 4
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                 // 5
  SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,         // 6
  SEG_A | SEG_B | SEG_C,                                 // 7
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // 8
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,         // 9
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,         // A
  SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,                 // b
  SEG_A | SEG_D | SEG_E | SEG_F,                         // C
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,                 // d
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,                 // E
  SEG_A | SEG_E | SEG_F | SEG_G                          // F
};

// Znaki specjalne do temperatury (dokładne maski bitowe)
static const uint8_t FONT_MINUS  = SEG_G;
static const uint8_t FONT_BLANK  = 0;
static const uint8_t FONT_DEGREE = SEG_A | SEG_B | SEG_F | SEG_G; // °
static const uint8_t FONT_C      = SEG_A | SEG_D | SEG_E | SEG_F; // C

// -----------------------------------------------------------------------------
// Shared state (written by tasks, read by DisplayTask)
// -----------------------------------------------------------------------------
volatile uint8_t g_displaySeg[4] = {0, 0, 0, 0};  // raw segment bytes (incl DP)
volatile uint8_t g_activeDigit = 0;

volatile int   g_hour   = 0;
volatile int   g_minute = 0;
volatile float g_tempC  = NAN;

volatile bool  g_showTemp = false;
volatile bool g_showBootId = true;

volatile bool g_timeValid = false;
volatile bool g_tempValid = false;

uint8_t g_displayNext[4];

// Brightness control
Preferences prefs;
volatile bool  g_autoBrightness = true;   // can be persisted later if you want
volatile uint8_t g_brightness = 220;      // 0..255 (logical brightness)

// -----------------------------------------------------------------------------
// WiFi / Portal
// -----------------------------------------------------------------------------
WebServer server(80);
AutoConnect portal(server);
AutoConnectConfig portalConfig;
AutoConnectOTA ota;
String g_hostName;
String g_deviceId;
char id[5] = {0};   // 4 hex + '\0'

// -----------------------------------------------------------------------------
// DS18B20
// -----------------------------------------------------------------------------
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

// -----------------------------------------------------------------------------
// Helpers: 74HC595 write (bit-banged, stable timing)
// -----------------------------------------------------------------------------
static inline void shiftOutByte(uint8_t val) {
  for (int i = 7; i >= 0; --i) {
    digitalWrite(PIN_595_CLK, LOW);
    digitalWrite(PIN_595_DATA, (val >> i) & 0x01);
    digitalWrite(PIN_595_CLK, HIGH);
  }
}

static inline void write595(uint8_t segments) {
  digitalWrite(PIN_595_LATCH, LOW);
  shiftOutByte(segments);
  digitalWrite(PIN_595_LATCH, HIGH);
}

// -----------------------------------------------------------------------------
// Display low-level
// -----------------------------------------------------------------------------
static const int DIGIT_PINS[4] = { PIN_DIGIT_0, PIN_DIGIT_1, PIN_DIGIT_2, PIN_DIGIT_3 };

static inline void allDigitsOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(DIGIT_PINS[i], DIGIT_ENABLE_HIGH ? LOW : HIGH);
  }
}

static inline void digitOn(uint8_t idx) {
  digitalWrite(DIGIT_PINS[idx], DIGIT_ENABLE_HIGH ? HIGH : LOW);
}

void refreshDisplayOnce() {
  uint8_t d = g_activeDigit;
  g_activeDigit = (d + 1) & 0x03;

  allDigitsOff();
  write595(g_displaySeg[d]);
  digitOn(d);
}

// -----------------------------------------------------------------------------
// Formatting: write to g_displaySeg[]
// -----------------------------------------------------------------------------
static inline uint8_t segForDigit(int d) {
  if (d < 0 || d > 9) return FONT_BLANK;
  return FONT_HEX[d];
}

uint8_t segFromChar(char c) {
  // cyfry
  if (c >= '0' && c <= '9') return FONT_HEX[c - '0'];
  // hex
  if (c >= 'A' && c <= 'F') return FONT_HEX[c - 'A' + 10];
  if (c >= 'a' && c <= 'f') return FONT_HEX[c - 'a' + 10];
  // znaki specjalne
  if (c == '-') return FONT_MINUS;
  if (c == 'C') return FONT_C;
  if (c == ' ') return FONT_BLANK;

  return FONT_BLANK;
}

void showBootId4() {
  g_showBootId = true;
  // Ostatnie 4 znaki ID, np. "A1B2C3" -> "B2C3"
  g_displayNext[0] = segFromChar(id[0]);
  g_displayNext[1] = segFromChar(id[1]);
  g_displayNext[2] = segFromChar(id[2]);
  g_displayNext[3] = segFromChar(id[3]);

  commitDisplayBuffer();

  delay(5000);   // tylko raz przy starcie
  g_showBootId = false;
}

void setDisplayTime(int hh, int mm, bool colonOn) {
  uint8_t s0 = (hh >= 10) ? segForDigit(hh / 10) : FONT_BLANK;
  uint8_t s1 = segForDigit(hh % 10);
  uint8_t s2 = segForDigit(mm / 10);
  uint8_t s3 = segForDigit(mm % 10);

  // Colon simulation: DP on digit1 (adjust if you prefer other digit)
  if (colonOn) s1 |= SEG_DP;
  else         s1 &= ~SEG_DP;

  g_displayNext[0] = s0;
  g_displayNext[1] = s1;
  g_displayNext[2] = s2;
  g_displayNext[3] = s3;
  commitDisplayBuffer();
}

void setDisplayTemp(float tC) {
  if (isnan(tC) || tC < 0.0f || tC > 99.0f) {
    g_displayNext[0] = FONT_BLANK;
    g_displayNext[1] = FONT_BLANK;
    g_displayNext[2] = FONT_BLANK;
    g_displayNext[3] = FONT_BLANK;
    commitDisplayBuffer();
    return;
  }

  int temp = (int)roundf(tC);

  // Dziesiątki – bez nieznaczącego zera
  g_displayNext[0] = (temp >= 10) ? segForDigit(temp / 10) : FONT_BLANK;
  // Jedności
  g_displayNext[1] = segForDigit(temp % 10);
  // Znak stopni i litera C
  g_displayNext[2] = FONT_DEGREE;
  g_displayNext[3] = FONT_C;
  commitDisplayBuffer();
}

void setDisplayDashes() {
  g_displayNext[0] = FONT_MINUS;
  g_displayNext[1] = FONT_MINUS;
  g_displayNext[2] = FONT_MINUS;
  g_displayNext[3] = FONT_MINUS;
  commitDisplayBuffer();
}

void commitDisplayBuffer() {
  noInterrupts();
  g_displaySeg[0] = g_displayNext[0];
  g_displaySeg[1] = g_displayNext[1];
  g_displaySeg[2] = g_displayNext[2];
  g_displaySeg[3] = g_displayNext[3];
  interrupts();
}

// -----------------------------------------------------------------------------
// Brightness: OE PWM + optional LDR auto brightness
// -----------------------------------------------------------------------------
static const int PWM_CH = 0;
static const int PWM_FREQ = 20000;   // 20 kHz (inaudible, stable)
static const int PWM_RES  = 8;       // 0..255

// OE is active LOW: duty=0 -> always LOW -> full ON; duty=255 -> mostly HIGH -> dim/off
static inline void applyBrightness(uint8_t logical) {
  // Map logical brightness (0..255, where 255 = brightest) to OE duty (inverted)
  uint8_t oeDuty = 255 - logical;
  ledcWrite(PWM_CH, oeDuty);
}

uint8_t computeAutoBrightnessFromLDR() {
  // LDR is at the bottom (to GND), 10k at the top (to +3.3V)
  // → dark = high ADC value, bright = low ADC value
  int raw = analogRead(PIN_LDR_ADC);

  // Smooth the reading (EMA)
  static float ema = 0;
  ema = 0.9f * ema + 0.1f * raw;

  // Calibration points (ADC values)
  // DARK  → high ADC
  // BRIGHT → low ADC
  const float RAW_DARK   = 3500;  // adjust after measurements
  const float RAW_BRIGHT = 800;   // adjust after measurements

  // Normalize: 0 = dark, 1 = bright
  float x = (ema - RAW_BRIGHT) / (RAW_DARK - RAW_BRIGHT);
  if (x < 0) x = 0;
  if (x > 1) x = 1;

  // Bright room → higher brightness
  float b = x;

  // Clamp to comfortable range
  const int B_MIN = 5;    // darkest room
  const int B_MAX = 250;  // daylight

  int out = (int)(B_MIN + b * (B_MAX - B_MIN));
  if (out < 0) out = 0;
  if (out > 255) out = 255;

  Serial.printf("LDR raw=%d  ema=%.1f  norm=%.2f  brightness=%d\n", raw, ema, x, out);

  return (uint8_t)out;
}

// -----------------------------------------------------------------------------
// Tasks
// -----------------------------------------------------------------------------
void DisplayTask(void *pv) {
  // Highest priority, Core 0: guarantees no blanking.
  for (;;) {
    refreshDisplayOnce();
    vTaskDelay(3); // ~3ms tick; adjust if needed
  }
}

void TimeTask(void *pv) {
  struct tm ti;
  for (;;) {
    if (getLocalTime(&ti, 50)) {
      g_hour = ti.tm_hour;
      g_minute = ti.tm_min;
      g_timeValid = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TempTask(void *pv) {
  // DS18B20 conversion is slow; keep it away from display timing.
  for (;;) {
    sensors.requestTemperatures();          // start conversion (non-blocking due to setWaitForConversion(false))
    vTaskDelay(pdMS_TO_TICKS(800));         // wait conversion (does not affect display)
    float t = sensors.getTempCByIndex(0);
    if (!isnan(t)) {
      g_tempC = t;
      g_tempValid = true;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void LogicTask(void *pv) {
  // Prepares display buffer only.
  bool colon = false;

  for (;;) {
    // czekaj na koniec show boot ID
    if (g_showBootId) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    // --- STARTUP: czekamy aż czas i temperatura będą gotowe ---
    if (!g_timeValid || !g_tempValid) {
      setDisplayDashes();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;   // TERAZ jesteśmy wewnątrz pętli -> OK
    }
    uint32_t now = millis();

    uint32_t phase = now % 15000;   // 15‑sekundowy cykl

    if (phase < 5000) {
      g_showTemp = true;    // 0–5 s → temperatura
    } else {
      g_showTemp = false;   // 5–15 s → czas
    }

    colon = !colon;

    if (g_showTemp) {
      setDisplayTemp(g_tempC);
    } else {
      setDisplayTime(g_hour, g_minute, colon);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void BrightnessTask(void *pv) {
  for (;;) {
    if (g_autoBrightness) {
      g_brightness = computeAutoBrightnessFromLDR();
      applyBrightness(g_brightness);
    } else {
      applyBrightness(g_brightness);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.printf("LDR raw=%d  brightness=%d\n", raw, g_brightness);
  }
}

void WiFiTask(void *pv) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  // mac[3], mac[4], mac[5] = unikalna część
  snprintf(id, sizeof(id), "%02X%02X", mac[4], mac[5]);

  g_hostName = String("esp32-clock-") + id;
  g_deviceId = id;

  showBootId4();

  portalConfig.autoReconnect = true;
  portalConfig.retainPortal  = true;
  portalConfig.apid          = String("ESP32-Clock-") + id;
  portalConfig.psk           = "12345678";
  portalConfig.hostName      = g_hostName.c_str();
  portalConfig.menuItems     = portalConfig.menuItems | AC_MENUITEM_DELETESSID;  // enable the credentials removal feature in OpenSSIDs menu

  portal.config(portalConfig);
  
  // --- ROOT -> /status redirect (HOME button fix) ---
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Location", "/status", true);
    server.send(302, "text/plain", "");
  });
  
  // --- Status endpoint ---
  server.on("/status", []() {
    String s;
    s.reserve(256);
    s += "id=" + g_deviceId + "\n";
    s += "hostname=" + g_hostName + "\n";
    s += "time=" + String((int)g_hour) + ":" + String((int)g_minute) + "\n";
    s += "tempC=" + String((float)g_tempC, 1) + "\n";
    s += "brightness=" + String((int)g_brightness) + "\n";
    s += "autoBrightness=" + String(g_autoBrightness ? "1" : "0") + "\n";
    s += "wifi=" + String((WiFi.status() == WL_CONNECTED) ? "connected" : "not_connected") + "\n";
    if (WiFi.status() == WL_CONNECTED) {
      s += "ip=" + WiFi.localIP().toString() + "\n";
      s += "rssi=" + String(WiFi.RSSI()) + "\n";
      s += "mdns=http://" + g_hostName + ".local/\n";
    }
    server.send(200, "text/plain", s);
  });

  ota.attach(portal);
  portal.begin();

  if (MDNS.begin(g_hostName.c_str())) {
    MDNS.addService("http", "tcp", 80);
  }

  for (;;) {
    portal.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------
void initDisplayHardware() {
  pinMode(PIN_595_DATA, OUTPUT);
  pinMode(PIN_595_CLK, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(DIGIT_PINS[i], OUTPUT);
  }
  allDigitsOff();
  write595(0);
}

void initBrightnessHardware() {
  pinMode(PIN_595_OE, OUTPUT);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_595_OE, PWM_CH);

  // ADC
  pinMode(PIN_LDR_ADC, INPUT);
  analogReadResolution(12);

  applyBrightness(g_brightness);
}

void loadSettings() {
  prefs.begin("clock", false);
  g_brightness = prefs.getUChar("bright", 220);
  g_autoBrightness = prefs.getBool("autoB", true);
}

void setupTime() {
  configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "tempus1.gum.gov.pl", "pl.pool.ntp.org", "tempus2.gum.gov.pl");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  loadSettings();
  initDisplayHardware();
  pinMode(PIN_595_OE, OUTPUT);
  digitalWrite(PIN_595_OE, HIGH);   // OE aktywne LOW → HIGH = wyłączone
  // Ustawiamy wyświetlacz w stan stabilny PRZED startem tasków
  g_displayNext[0] = FONT_MINUS;
  g_displayNext[1] = FONT_MINUS;
  g_displayNext[2] = FONT_MINUS;
  g_displayNext[3] = FONT_MINUS;
  commitDisplayBuffer();
  g_activeDigit = 0;
  // Twarde wygaszenie wszystkich cyfr (ULN2803)
  allDigitsOff();
  write595(0);   // wyczyść 74HC595

  initBrightnessHardware();

  sensors.begin();
  sensors.setWaitForConversion(false);

  setupTime();

  // Tasks
  xTaskCreatePinnedToCore(DisplayTask,     "Display",    2048, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(TimeTask,        "Time",       4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TempTask,        "Temp",       4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(LogicTask,       "Logic",      4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(BrightnessTask,  "Brightness", 2048, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(WiFiTask,        "WiFi",       8192, nullptr, 1, nullptr, 1);

  // loop() intentionally unused
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
