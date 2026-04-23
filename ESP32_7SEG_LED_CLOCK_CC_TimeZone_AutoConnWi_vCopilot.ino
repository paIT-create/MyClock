/*
  ESP32 Clock + Thermometer (DS18B20) + 4x7seg (CC) via 74HC595 + ULN2803
  Ultra‑Stable Edition (2026‑04)

  - Display refresh in hardware timer ISR (no blanking)
  - DS18B20 handled asynchronously
  - Time with DST via configTzTime + getLocalTime
  - WiFi configuration via AutoConnect (retainPortal)
  - OTA via AutoConnectOTA
  - Status endpoint: /status
  - Config UI: /config
  - mDNS: esp32-clock-XXXX.local
  - Brightness: OE pin PWM + optional auto brightness from LDR (ADC)
  
  ✔ ESP32 core 2.0.17  
  ✔ AutoConnect 1.4.2
  ✔ PageBuilder 1.5.6
*/

#include <Arduino.h>

// WiFi / Portal / OTA
#include <WiFi.h>
#include <WebServer.h>
#include <AutoConnect.h>
#include <AutoConnectOTA.h>
#include <ESPmDNS.h>

// Time
#include <time.h>

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// Preferences
#include <Preferences.h>

// -----------------------------------------------------------------------------
// Pinout
// -----------------------------------------------------------------------------
static const int PIN_595_CLK = 12;    // SRCLK
static const int PIN_595_LATCH = 13;  // RCLK
static const int PIN_595_DATA = 14;   // SER

static const int PIN_595_OE = 27;   // OE (PWM brightness, active LOW)
static const int PIN_LDR_ADC = 34;  // LDR analog input (ADC1)
static const int PIN_ONEWIRE = 15;  // DS18B20 data
static const int PIN_LED = 2;       // On-board LED

// Digit select pins (to ULN2803 inputs) for 4 digits (common cathode)
static const int PIN_DIGIT_0 = 32;  // leftmost
static const int PIN_DIGIT_1 = 33;
static const int PIN_DIGIT_2 = 25;
static const int PIN_DIGIT_3 = 26;

static const bool DIGIT_ENABLE_HIGH = true;

// -----------------------------------------------------------------------------
// 7-seg segments and font
// -----------------------------------------------------------------------------
static const uint8_t SEG_A = 1 << 0;
static const uint8_t SEG_B = 1 << 1;
static const uint8_t SEG_C = 1 << 2;
static const uint8_t SEG_D = 1 << 3;
static const uint8_t SEG_E = 1 << 4;
static const uint8_t SEG_F = 1 << 5;
static const uint8_t SEG_G = 1 << 6;
static const uint8_t SEG_DP = 1 << 7;

// Bit layout: 0bDPGFEDCBA
static const uint8_t FONT_HEX[16] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          // 0
  SEG_B | SEG_C,                                          // 1
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                  // 2
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                  // 3
  SEG_B | SEG_C | SEG_F | SEG_G,                          // 4
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                  // 5
  SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,          // 6
  SEG_A | SEG_B | SEG_C,                                  // 7
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,  // 8
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,          // 9
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,          // A
  SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,                  // b
  SEG_A | SEG_D | SEG_E | SEG_F,                          // C
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,                  // d
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,                  // E
  SEG_A | SEG_E | SEG_F | SEG_G                           // F
};

static const uint8_t FONT_MINUS = SEG_G;
static const uint8_t FONT_BLANK = 0;
static const uint8_t FONT_DEGREE = SEG_A | SEG_B | SEG_F | SEG_G;
static const uint8_t FONT_C = SEG_A | SEG_D | SEG_E | SEG_F;

// -----------------------------------------------------------------------------
// Global state
// -----------------------------------------------------------------------------
volatile uint8_t g_displaySeg[4] = { 0, 0, 0, 0 };
uint8_t g_displayNext[4] = { 0, 0, 0, 0 };

volatile int g_hour = 0;
volatile int g_minute = 0;
volatile int g_second = 0;
volatile float g_tempC = NAN;

volatile bool g_showTemp = false;
volatile bool g_showBootId = true;

volatile bool g_timeValid = false;
volatile bool g_tempValid = false;

// WiFi watchdog
volatile bool g_forceWifiDot = false;
bool wifiWasConnected = false;

// OTA status
volatile bool g_otaActive = false;

// Brightness
Preferences prefs;
volatile bool g_autoBrightness = true;
volatile uint8_t g_brightness = 150;  // 0..255 logical brightness

// WiFi / portal
WebServer server(80);
AutoConnect portal(server);
AutoConnectConfig portalConfig;
AutoConnectOTA ota;

String g_hostName;
String g_deviceId;
char id[5] = { 0 };  // 4 hex + '\0'

// DS18B20
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

// -----------------------------------------------------------------------------
// 74HC595 helpers
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
static const int DIGIT_PINS[4] = {
  PIN_DIGIT_0, PIN_DIGIT_1, PIN_DIGIT_2, PIN_DIGIT_3
};

static inline void allDigitsOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(DIGIT_PINS[i], DIGIT_ENABLE_HIGH ? LOW : HIGH);
  }
}

static inline void digitOn(uint8_t idx) {
  digitalWrite(DIGIT_PINS[idx], DIGIT_ENABLE_HIGH ? HIGH : LOW);
}

// -----------------------------------------------------------------------------
// Display timer ISR (ultra-stable, no drift)
// -----------------------------------------------------------------------------
hw_timer_t *displayTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t currentDigit = 0;

static const uint16_t DIGIT_ON_US = 350;
static const uint16_t FRAME_US = 2000;

void IRAM_ATTR onDisplayTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  static uint32_t nextSwitch = 0;
  uint32_t now = micros();

  if ((int32_t)(now - nextSwitch) >= 0) {
    nextSwitch += DIGIT_ON_US;

    if (g_otaActive) {
      allDigitsOff();
      write595(FONT_HEX[10]);  // 'A'
      digitOn(0);
    } else {
      allDigitsOff();
      write595(g_displaySeg[currentDigit]);
      digitOn(currentDigit);
      currentDigit = (currentDigit + 1) & 0x03;
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// -----------------------------------------------------------------------------
// Formatting helpers
// -----------------------------------------------------------------------------
static inline uint8_t segForDigit(int d) {
  if (d < 0 || d > 9) return FONT_BLANK;
  return FONT_HEX[d];
}

uint8_t segFromChar(char c) {
  if (c >= '0' && c <= '9') return FONT_HEX[c - '0'];
  if (c >= 'A' && c <= 'F') return FONT_HEX[c - 'A' + 10];
  if (c >= 'a' && c <= 'f') return FONT_HEX[c - 'a' + 10];
  if (c == '-') return FONT_MINUS;
  if (c == 'C') return FONT_C;
  if (c == ' ') return FONT_BLANK;
  return FONT_BLANK;
}

void commitDisplayBuffer() {
  noInterrupts();
  g_displaySeg[0] = g_displayNext[0];
  g_displaySeg[1] = g_displayNext[1];
  g_displaySeg[2] = g_displayNext[2];
  g_displaySeg[3] = g_displayNext[3];
  interrupts();
}

void showBootId4() {
  g_showBootId = true;

  g_displayNext[0] = segFromChar(id[0]);
  g_displayNext[1] = segFromChar(id[1]);
  g_displayNext[2] = segFromChar(id[2]);
  g_displayNext[3] = segFromChar(id[3]);
  commitDisplayBuffer();

  delay(5000);
  g_showBootId = false;
}

void setDisplayTime(int hh, int mm, bool colonOn) {
  uint8_t s0 = (hh >= 10) ? segForDigit(hh / 10) : FONT_BLANK;
  uint8_t s1 = segForDigit(hh % 10);
  uint8_t s2 = segForDigit(mm / 10);
  uint8_t s3 = segForDigit(mm % 10);

  if (colonOn) s1 |= SEG_DP;
  else s1 &= ~SEG_DP;

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

  g_displayNext[0] = (temp >= 10) ? segForDigit(temp / 10) : FONT_BLANK;
  g_displayNext[1] = segForDigit(temp % 10);
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

int getDS18B20Resolution() {
  DeviceAddress addr;
  if (!sensors.getAddress(addr, 0)) {
    return -1;
  }
  return sensors.getResolution(addr);
}

// -----------------------------------------------------------------------------
// Brightness control
// -----------------------------------------------------------------------------
static const int PWM_CH = 0;
static const int PWM_FREQ = 20000;  // 20 kHz
static const int PWM_RES = 8;       // 0..255

// OE is active LOW: duty=0 -> full ON, duty=255 -> off
static inline void applyBrightness(uint8_t logical) {
  uint8_t oeDuty = 255 - logical;
  ledcWrite(PWM_CH, oeDuty);
}

uint8_t computeAutoBrightnessFromLDR() {
  int raw = analogRead(PIN_LDR_ADC);

  static float ema = 0;
  ema = 0.9f * ema + 0.1f * raw;

  // WOOD enclosure calibration
  const float RAW_DARK = 3900;
  const float RAW_BRIGHT = 900;
  // --- OBUDOWA PLA ---
  // const float RAW_DARK = 2500;   // adjust after measurements
  // const float RAW_BRIGHT = 50;  // adjust after measurements

  float x = (RAW_DARK - ema) / (RAW_DARK - RAW_BRIGHT);
  if (x < 0) x = 0;
  if (x > 1) x = 1;

  const int B_MIN = 5;
  const int B_MAX = 250;

  int out = (int)(B_MIN + x * (B_MAX - B_MIN));
  if (out < 0) out = 0;
  if (out > 255) out = 255;

  return (uint8_t)out;
}

void BrightnessTask(void *pv) {
  uint8_t lastApplied = g_brightness;

  for (;;) {
    uint8_t target = g_brightness;

    if (g_autoBrightness) {
      target = computeAutoBrightnessFromLDR();
      g_brightness = target;
    }

    if (abs((int)target - (int)lastApplied) >= 2) {
      applyBrightness(target);
      lastApplied = target;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// -----------------------------------------------------------------------------
// TimeTask
// -----------------------------------------------------------------------------
void TimeTask(void *pv) {
  struct tm ti;
  int lastSec = -1;

  for (;;) {
    if (getLocalTime(&ti, 50)) {
      if (ti.tm_sec != lastSec) {
        lastSec = ti.tm_sec;
        g_hour = ti.tm_hour;
        g_minute = ti.tm_min;
        g_second = ti.tm_sec;
        g_timeValid = true;
      }
    }
    vTaskDelay(1);
  }
}

// -----------------------------------------------------------------------------
// TempTask
// -----------------------------------------------------------------------------
void TempTask(void *pv) {
  for (;;) {
    sensors.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(800));

    float t = sensors.getTempCByIndex(0);
    if (!isnan(t)) {
      g_tempC = t;
      g_tempValid = true;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// -----------------------------------------------------------------------------
// LogicTask
// -----------------------------------------------------------------------------
void LogicTask(void *pv) {
  static int lastSec = -1;

  for (;;) {
    if (g_showBootId) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (!g_timeValid || !g_tempValid) {
      setDisplayDashes();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (g_second != lastSec) {
      lastSec = g_second;

      bool colon = (g_second % 2) == 0;

      uint32_t now = millis();
      uint32_t phase = now % 20000;  // 20 s cycle

      g_showTemp = (phase < 5000);  // 0–5 s temp, 5–20 s time

      if (g_showTemp) {
        setDisplayTemp(g_tempC);
      } else {
        setDisplayTime(g_hour, g_minute, colon);
      }

      if (g_forceWifiDot) {
        g_displayNext[3] |= SEG_DP;
      } else {
        g_displayNext[3] &= ~SEG_DP;
      }
      commitDisplayBuffer();
    }

    vTaskDelay(1);
  }
}
// Minimalny watchdog — tylko kropka przy utracie WiFi
void wifiWatchdog() {
  wl_status_t st = WiFi.status();

  if (st == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      g_forceWifiDot = false;
      Serial.println("WiFi OK");
    }
  } else {
    if (wifiWasConnected) {
      wifiWasConnected = false;
      g_forceWifiDot = true;
      Serial.println("WiFi LOST");

    }
  }
}
// -----------------------------------------------------------------------------
// Settings (Preferences)
// -----------------------------------------------------------------------------
void saveSettings() {
  prefs.begin("clock", false);
  prefs.putUChar("bright", g_brightness);
  prefs.putBool("autoB", g_autoBrightness);
  prefs.end();
}

void resetSettings() {
  const uint8_t DEFAULT_BRIGHT = 150;
  const bool DEFAULT_AUTO = true;

  prefs.begin("clock", false);
  prefs.putUChar("bright", DEFAULT_BRIGHT);
  prefs.putBool("autoB", DEFAULT_AUTO);
  prefs.end();

  g_brightness = DEFAULT_BRIGHT;
  g_autoBrightness = DEFAULT_AUTO;
}

void loadSettings() {
  prefs.begin("clock", false);
  g_brightness = prefs.getUChar("bright", 150);
  g_autoBrightness = prefs.getBool("autoB", true);
  prefs.end();
}

// -----------------------------------------------------------------------------
// Time setup (NTP + DST)
// -----------------------------------------------------------------------------
void setupTime() {
  configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3",
               "tempus1.gum.gov.pl",
               "pl.pool.ntp.org",
               "tempus2.gum.gov.pl");

  struct tm timeinfo;
  unsigned long start = millis();

  while (millis() - start < 3000) {
    if (getLocalTime(&timeinfo)) {
      if (timeinfo.tm_year + 1900 > 2020) {
        int lastSec = timeinfo.tm_sec;
        unsigned long secWaitStart = millis();

        while (millis() - secWaitStart < 1500) {
          getLocalTime(&timeinfo);
          if (timeinfo.tm_sec != lastSec) return;
          delay(1);
        }
        return;
      }
    }
    delay(10);
  }
  // If we get here: no NTP sync; LogicTask will show ----
}

// -----------------------------------------------------------------------------
// WiFiTask (AutoConnect + UI + status)
// -----------------------------------------------------------------------------
void WiFiTask(void *pv) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  snprintf(id, sizeof(id), "%02X%02X", mac[4], mac[5]);
  g_hostName = String("esp32-clock-") + id;
  g_deviceId = id;

  showBootId4();

  portalConfig.autoReconnect = true;
  portalConfig.reconnectInterval = 6; // Przerwa między próbami (w jednostkach 5s, tu: 30s)
  portalConfig.retainPortal = true;
  portalConfig.apid = String("ESP32-Clock-") + id;
  portalConfig.psk = "Al@m@kot@";
  portalConfig.hostName = g_hostName.c_str();
  portalConfig.menuItems |= AC_MENUITEM_DELETESSID;
  portal.config(portalConfig);

  // OTA callbacks
  ota.onStart([]() {
    g_otaActive = true;
  });
  ota.onEnd([]() {
    g_otaActive = false;
  });

  // Root redirect -> /config
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Location", "/config", true);
    server.send(302, "text/plain", "");
  });

  // /status endpoint
  server.on("/status", []() {
    String s;
    String mm = (g_minute < 10 ? "0" : "") + String(g_minute);
    String ss = (g_second < 10 ? "0" : "") + String(g_second);

    s.reserve(256);
    s += "id=" + g_deviceId + "\n";
    s += "hostname=" + g_hostName + "\n";
    s += "time=" + String(g_hour) + ":" + mm + ":" + ss + "\n";
    if (g_tempC > -100) {
      s += "tempC=" + String((float)g_tempC, 1) + "\n";
    }
    s += "ds18b20_resolution=" + String(getDS18B20Resolution()) + "\n";
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

  // /config UI
  server.on("/config", HTTP_GET, []() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Clock Config</title>

<style>
body{
  margin:0;padding:20px;background:#05060a;
  font-family:Segoe UI,Roboto,Arial,sans-serif;
  background:linear-gradient(135deg,#05060a,#0a0d14,#05060a);
  background-size:400% 400%;
  animation:bgmove 18s ease infinite;
  color:#d0d0d0;
}
@keyframes bgmove{
  0%{background-position:0% 50%;}
  50%{background-position:100% 50%;}
  100%{background-position:0% 50%;}
}
.card{
  background:#0f1117;padding:28px;border-radius:16px;
  max-width:500px;margin:25px auto;
  box-shadow:0 0 25px #0090ff55,0 0 60px #0050ff33,inset 0 0 20px #0030aa55;
}
h2{
  text-align:center;margin-top:0;color:#6ab8ff;font-size:26px;
  text-shadow:0 0 12px #0088ff,0 0 22px #0066ff;
}
label{
  display:block;margin-top:25px;font-weight:600;color:#9fc9ff;
  text-shadow:0 0 6px #0044aa;
}
.value{font-size:14px;color:#aaa;margin-top:6px;}
input[type=range]{
  width:100%;margin-top:12px;-webkit-appearance:none;height:6px;
  background:#222;border-radius:4px;outline:none;
  box-shadow:0 0 10px #0077ff88;
}
input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:22px;height:22px;
  background:#00aaff;border-radius:50%;cursor:pointer;
  box-shadow:0 0 12px #00aaff,0 0 22px #0088ff;
}
input[type=checkbox]{transform:scale(1.5);margin-top:12px;cursor:pointer;}
.btn{
  margin-top:30px;width:100%;padding:14px;border:none;border-radius:12px;
  font-size:18px;cursor:pointer;font-weight:600;transition:0.25s;
  letter-spacing:0.5px;
}
.save{
  background:#0078ff;color:white;
  box-shadow:0 0 18px #0078ffcc,0 0 30px #0050ff88;
}
.save:hover{
  background:#0a8bff;
  box-shadow:0 0 25px #0a8bffdd,0 0 40px #0070ffaa;
}
.reset{
  background:#333;color:#ccc;margin-top:12px;
  box-shadow:0 0 12px #444;
}
.reset:hover{
  background:#444;color:white;box-shadow:0 0 18px #666;
}
.statusBox{
  margin-top:35px;padding:18px;background:#0a0c12;border-radius:12px;
  box-shadow:inset 0 0 18px #0070ffaa,inset 0 0 35px #0030aa55;
}
.titleSmall{
  color:#6ab8ff;font-size:17px;margin-bottom:12px;
  text-shadow:0 0 10px #0088ff;
}
.statusLine{
  margin:6px 0;font-size:14px;color:#c0c0c0;font-family:Consolas,monospace;
  text-shadow:0 0 6px #0040aa;
}
.bigClockBox{text-align:center;margin-top:10px;margin-bottom:25px;}
.bigClock{
  font-size:48px;font-weight:700;color:#6ab8ff;
  text-shadow:0 0 12px #0088ff,0 0 25px #0066ff,0 0 40px #0044aa;
  letter-spacing:2px;margin-bottom:10px;
}
.bigTemp{
  font-size:32px;font-weight:600;color:#ffdd88;
  text-shadow:0 0 12px #ffaa00,0 0 25px #ff8800,0 0 40px #cc6600;
}
</style>

<script>
let hh="--", mm="--", ss="--";
let temp="--.-";
let firstStatus = true;

function setAuto() {
  let a = document.getElementById('auto').checked ? 1 : 0;
  fetch('/set?auto=' + a)
    .then(() => {
      console.log("Auto brightness updated immediately:", a);
    });
}
    
function setBright(v) {
  if (document.getElementById('auto').checked) return;

  fetch('/set?bright=' + v)
    .then(() => {
      document.getElementById('brightVal').textContent = "Aktualnie: " + v;
      console.log("Brightness updated immediately:", v);
    });
}
    
function save(){
  let b=document.getElementById('bright').value;
  let a=document.getElementById('auto').checked?1:0;
  fetch('/set?bright='+b+'&auto='+a).then(()=>{
    alert('Zapisano ustawienia');
  });
}

function reset(){
  fetch('/reset').then(()=>{
    alert('Przywrócono ustawienia domyślne');
    location.reload();
  });
}

function updateClock(){
  document.getElementById('bigClock').textContent =
    hh + ":" + mm + ":" + ss;
}

function updateTemp(){
  document.getElementById('bigTemp').textContent = temp + " °C";
}

function loadStatus(){
  fetch('/status').then(r=>r.text()).then(t=>{
    let lines=t.trim().split('\n');
    let box=document.getElementById('statusBox');
    box.innerHTML='';

    lines.forEach(l=>{
      let div=document.createElement('div');
      div.className='statusLine';
      div.textContent=l;
      box.appendChild(div);

      if(l.startsWith("time=")){
        let parts=l.substring(5).split(":");
        hh = parts[0];
        mm = parts[1];
        ss = parts[2];
        updateClock();
      }

      if(l.startsWith("tempC=")){
        let v = parseFloat(l.substring(6));
        if (v > -100) {
          temp = v.toFixed(1);
          updateTemp();
        }
      }
    
      if(l.startsWith("brightness=")){
        let v = l.substring(11);
        const bright = document.getElementById('bright');
        const auto = document.getElementById('auto').checked;

        if (firstStatus) {
          bright.value = v;
          document.getElementById('brightVal').textContent = "Aktualnie: " + v;
          firstStatus = false;
          return;
        }

        if (auto) {
          bright.value = v;
          document.getElementById('brightVal').textContent = "Aktualnie: " + v;
        }
      }

      if(l.startsWith("autoBrightness=")){
        let v = l.substring(15);
        document.getElementById('auto').checked = (v === "1");
      }
    });
  });
}

setInterval(loadStatus, 1000);
window.onload = loadStatus;
</script>

</head>
<body>

<div class="card">
<h2>Ustawienia Zegara</h2>

<div class="bigClockBox">
  <div id="bigClock" class="bigClock">--:--:--</div>
  <div id="bigTemp" class="bigTemp">--.- °C</div>
</div>

<label>Jasność</label>
<input type="range" id="bright" min="0" max="255" oninput="setBright(this.value)">
<div class="value" id="brightVal"></div>

<label>Auto jasność</label>
<input type="checkbox" id="auto" onchange="setAuto()">

<button class="btn save" onclick="save()">💾 Zapisz</button>
<button class="btn reset" onclick="reset()">↺ Reset</button>

<div class="statusBox">
  <div class="titleSmall">Status urządzenia</div>
  <div id="statusBox">Ładowanie...</div>
</div>

</div>
</body>
</html>
)rawliteral";

    server.send(200, "text/html", html);
  });

  // /set endpoint
  server.on("/set", []() {
    if (server.hasArg("bright")) {
      g_brightness = server.arg("bright").toInt();
    }
    if (server.hasArg("auto")) {
      g_autoBrightness = (server.arg("auto") == "1");
    }
    saveSettings();
    server.send(200, "text/plain", "OK");
  });

  // /reset endpoint
  server.on("/reset", []() {
    resetSettings();
    server.send(200, "text/plain", "OK: reset");
  });

  ota.attach(portal);
  portal.begin();

  if (MDNS.begin(g_hostName.c_str())) {
    MDNS.addService("http", "tcp", 80);
  }

  for (;;) {
    portal.handleClient();
    wifiWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// -----------------------------------------------------------------------------
// Hardware init
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
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_595_OE, PWM_CH);

  pinMode(PIN_LDR_ADC, INPUT);
  analogReadResolution(12);

  applyBrightness(g_brightness);
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  WiFi.setAutoReconnect(false);
  WiFi.persistent(true);

  loadSettings();
  initDisplayHardware();

  g_displayNext[0] = FONT_MINUS;
  g_displayNext[1] = FONT_MINUS;
  g_displayNext[2] = FONT_MINUS;
  g_displayNext[3] = FONT_MINUS;
  commitDisplayBuffer();

  allDigitsOff();
  write595(0);

  initBrightnessHardware();

  sensors.begin();
  sensors.setWaitForConversion(false);

  setupTime();

  displayTimer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(displayTimer, &onDisplayTimer, true);
  timerAlarmWrite(displayTimer, FRAME_US, true);
  timerAlarmEnable(displayTimer);

  xTaskCreatePinnedToCore(TimeTask, "Time", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TempTask, "Temp", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(LogicTask, "Logic", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(BrightnessTask, "Bright", 2048, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(WiFiTask, "WiFi", 8192, nullptr, 1, nullptr, 1);

}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
