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
  - Web UI - LDR Calibration and Temperature Offset
  - Web UI - Alarm handler
  
  ✔ ESP32 core 2.0.17  
  ✔ AutoConnect 1.4.2
  ✔ PageBuilder 1.5.6
*/
#include <Arduino.h>

// WiFi / Portal / OTA
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <AutoConnect.h>
#include <AutoConnectOTA.h>
#include <ESPmDNS.h>

// Time
#include <time.h>
#include "esp_sntp.h"  // Niezbędne do zmiany interwału i callbacka

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// Preferences
#include <Preferences.h>

// Watchdog Timer (WDT)
#include <esp_task_wdt.h>

// --- KOD HTML W PAMIĘCI FLASH ---
const char CONFIG_HTML[] PROGMEM = R"rawliteral(
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
  display:block;margin-top:10px;font-weight:600;color:#9fc9ff;
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
.bigDate {
  font-size: 20px;
  font-weight: 500;
  color: #a0c4ff; /* Jasnobłękitny */
  text-shadow: 0 0 8px #0078ff, 0 0 15px #0050ff;
  margin-top: 10px;
  letter-spacing: 1px;
  font-family: 'Segoe UI', sans-serif;
}
.calendar-icon {
  margin-right: 8px;
  font-size: 18px;
  vertical-align: middle;
  opacity: 0.8;
}
</style>

<script>
let hh="--", mm="--", ss="--";
let temp="--.-";
let firstStatus = true;
let debounceTimer; // Timer dla suwaka jasności
let localTime = new Date();
let lastSyncTime = 0;
let isEditing = false; // Flaga blokująca auto-odświeżanie pól podczas wpisywania

// Funkcja Debounce: wysyła żądanie dopiero 150ms po zakończeniu ruchu suwakiem
function setBright(v) {
  if (document.getElementById('auto').checked) return;
  
  // Aktualizujemy etykietę natychmiast dla płynności interfejsu
  document.getElementById('brightVal').textContent = "Aktualnie: " + v;

  clearTimeout(debounceTimer);
  debounceTimer = setTimeout(() => {
    fetch('/set?bright=' + v)
      .then(() => console.log("Brightness updated:", v))
      .catch(err => console.error("Update failed", err));
  }, 150); 
}

function setAuto() {
  let a = document.getElementById('auto').checked ? 1 : 0;
  // Przy checkboxie wysyłamy od razu, bo to pojedyncze kliknięcie
  fetch('/set?auto=' + a)
    .then(() => {
      console.log("Auto brightness toggled:", a);
      // Jeśli włączono auto, suwak staje się nieaktywny
      document.getElementById('bright').disabled = (a === 1);
    });
}
    
function save(){
  // ZMIANA: Strzelamy do nowego endpointu /save, który faktycznie robi prefs.put
  fetch('/save').then(r => {
    if(r.ok) alert('Ustawienia zapisane trwale w pamięci Flash');
    else alert('Błąd zapisu');
  });
}

function applyAdv() {
  let to = document.getElementById('tOff').value;
  let rd = document.getElementById('rDark').value;
  let rb = document.getElementById('rBright').value;
  fetch(`/set?tOff=${to}&rDark=${rd}&rBright=${rb}`).then(() => {
    alert('Zastosowano! Nie zapomnij zapisać na stałe.');
    console.log("Parametry kalibracji przesłane");
    isEditing = false; // Po zapisaniu pozwalamy na odświeżenie pól z urządzenia
    });
}

function reset(){
  if(confirm('Czy na pewno przywrócić ustawienia fabryczne?')){
    fetch('/reset').then(()=>{
      alert('Reset zakończony. Strona zostanie odświeżona.');
      location.reload();
    });
  }
}

function reboot() {
  if (confirm('Czy na pewno zrestartować urządzenie?')) {
    fetch('/reboot').then(() => {
      alert('Zegar restartuje się...');
      setTimeout(() => location.reload(), 5000);
    });
  }
}

function updateClock() {
  // Jeśli od ostatniej synchronizacji minęło więcej niż 1s, dodaj sekundę lokalnie
  let now = new Date();
  if (now - lastSyncTime >= 1000) {
    localTime.setSeconds(localTime.getSeconds() + 1);
    lastSyncTime = now;
  }

  let hh = String(localTime.getHours()).padStart(2, '0');
  let mm = String(localTime.getMinutes()).padStart(2, '0');
  let ss = String(localTime.getSeconds()).padStart(2, '0');
  
  document.getElementById('bigClock').textContent = hh + ":" + mm + ":" + ss;
}

// Uruchom płynne odliczanie co 100ms dla idealnej płynności
setInterval(updateClock, 100);

function updateTemp(){
  document.getElementById('bigTemp').textContent = temp + " °C";
}

function updateAlarm() {
  let t = document.getElementById('alTime').value;
  let on = document.getElementById('alActive').checked ? 1 : 0;
  fetch(`/set?alTime=${t}&alOn=${on}`);
}

function stopAlarm() {
  fetch('/set?stopAlarm=1');
}

function loadStatus(){
  // Dodajemy timestamp, aby zapobiec cachowaniu odpowiedzi przez przeglądarkę
  fetch('/status?t=' + Date.now()).then(r=>r.text()).then(t=>{
    let lines=t.trim().split('\n');
    let box=document.getElementById('statusBox');
    box.innerHTML='';

    lines.forEach(l=>{
      let div=document.createElement('div');
      div.className='statusLine';
      div.textContent=l;
      box.appendChild(div);

      if(l.startsWith("time=")){
        let parts = l.substring(5).split(":");
        if(parts.length === 3) {
          // Synchronizacja lokalnego czasu z tym z ESP32
          localTime.setHours(parseInt(parts[0]));
          localTime.setMinutes(parseInt(parts[1]));
          localTime.setSeconds(parseInt(parts[2]));
          lastSyncTime = new Date(); // Reset licznika płynności
        }
      }

      if(l.startsWith("date=")){
        let d = l.substring(5);
        let dayLine = lines.find(line => line.startsWith("day="));
        let dayName = dayLine ? dayLine.substring(4) : "";
        // Aktualizujemy tylko tekst, aby nie nadpisać ikonki spanem
        document.getElementById('dateText').textContent = dayName + ", " + d;
      }

      if(l.startsWith("lastSync=")){
        document.getElementById('lastSync').textContent = "Synchronizacja: " + l.substring(9);
      }

      if(l.startsWith("tempC=")){
        let v = l.substring(6).trim();
        if (v === "nan" || parseFloat(v) < -80) {
          document.getElementById('bigTemp').textContent = "--.- °C";
        } else {
          document.getElementById('bigTemp').textContent = parseFloat(v).toFixed(1) + " °C";
        }
      }
    
      if(l.startsWith("brightness=")){
        let v = l.substring(11);
        const brightInput = document.getElementById('bright');
        
        // Aktualizujemy suwak tylko jeśli nie jest właśnie przesuwany przez użytkownika
        if (firstStatus || document.getElementById('auto').checked) {
          brightInput.value = v;
          document.getElementById('brightVal').textContent = "Aktualnie: " + v;
        }
      }

      if(l.startsWith("autoBrightness=")){
        let isAuto = (l.substring(15) === "1");
        document.getElementById('auto').checked = isAuto;
        document.getElementById('bright').disabled = isAuto; // Blokada suwaka
      }
      // Obsługa pól kalibracji (tylko jeśli użytkownik nie klika w nie teraz)
      if (!isEditing) {
        if(l.startsWith("tOff=")) document.getElementById('tOff').value = l.substring(5);
        if(l.startsWith("rDark=")) document.getElementById('rDark').value = l.substring(6);
        if(l.startsWith("rBright=")) document.getElementById('rBright').value = l.substring(8);
      }

      if(l.startsWith("rawLDR=")) {
        let val = l.substring(7);
        document.getElementById('liveLDR').textContent = val;
  
        // Opcjonalnie: zmiana koloru jeśli wartość zbliża się do progów
        let rd = parseInt(document.getElementById('rDark').value);
        let rb = parseInt(document.getElementById('rBright').value);
        if(val >= rd || val <= rb) {
          document.getElementById('liveLDR').style.color = "#ff4444"; // Alarm - poza zakresem
        } else {
          document.getElementById('liveLDR').style.color = "#00ffaa"; // OK
        }
      }

      // Obsługa Alarmu
      if(l.startsWith("isAlarming=")){
        let isAlarming = (l.substring(11) === "1");
        // Jeśli alarm gra, pokazujemy przycisk STOP, w przeciwnym razie ukrywamy
        document.getElementById('stopAl').style.display = isAlarming ? "block" : "none";
      }
      if(l.startsWith("alTime=")) {
        // Aktualizujemy pole czasu w panelu tylko raz (przy ładowaniu) 
        // lub gdy użytkownik nie edytuje właśnie pola
        if (!isEditing) document.getElementById('alTime').value = l.substring(7);
      }
      if(l.startsWith("alActive=")) {
        document.getElementById('alActive').checked = (l.substring(9) === "1");
      }

    });
    firstStatus = false;
  }).catch(err => console.log("Status offline"));
}

// Obsługa flagi edycji - gdy użytkownik kliknie w pole, przestajemy nadpisywać mu tekst
function setEdit(state) {
  isEditing = state;
}

// Odświeżanie co 1 sekundę
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
  <div id="bigDate" class="bigDate"><span class="calendar-icon">📅</span><span id="dateText">-- --- ----</span></div>
  <div id="lastSync" style="font-size:16px; color:#555; margin-top:10px;">Ostatnia synch: --:--</div>
</div>

<div style="border-top:1px solid #333; margin-top:20px; padding-top:15px;">
  <label>⏰ Budzik</label>
  <input type="time" id="alTime" onfocus="setEdit(true)" onblur="setEdit(false)" onchange="updateAlarm()" style="width:100%; background:#111; color:#fff; border:1px solid #444; padding:8px; border-radius:8px;">
  <label><input type="checkbox" id="alActive" onchange="updateAlarm()"> Budzik włączony</label>
  <button id="stopAl" class="btn reset" style="display:none; background:#ff4444; color:#fff;" onclick="stopAlarm()">WYŁĄCZ ALARM</button>
</div>

<label>Jasność</label>
<input type="range" id="bright" min="0" max="255" oninput="setBright(this.value)">
<div class="value" id="brightVal"></div>

<label>Auto jasność</label>
<input type="checkbox" id="auto" onchange="setAuto()">

<button class="btn save" onclick="save()">💾 Zapisz</button>
<details style="margin-top:10px; text-align:left; color:#6ab8ff;">
  <summary style="cursor:pointer; font-weight:bold; padding:10px;">⚙️ Zaawansowana Kalibracja</summary>
  <div style="padding:15px; background:#0a0c12; border-radius:12px; margin-top:5px; border:1px solid #0070ff44;">
    
    <label style="font-size:13px;">Korekta Temp (°C)</label>
    <input type="number" id="tOff" step="0.1" onfocus="setEdit(true)" onblur="setEdit(false)" 
           style="width:90%; background:#05060a; color:#ffdd88; border:1px solid #333; padding:8px; margin:5px 0; border-radius:6px;">
    
    <div style="margin: 10px 0; padding: 8px; background: #1a1d26; border-radius: 6px; text-align: center; border: 1px solid #0070ff22;">
      <span style="font-size: 11px; color: #888; text-transform: uppercase;">Aktualny odczyt sensora:</span>
      <div id="liveLDR" style="font-size: 20px; color: #00ffaa; font-weight: bold; text-shadow: 0 0 10px #00ffaa66;">----</div>
    </div>
    <label style="font-size:13px; display:block; margin-top:10px;">LDR Dark (Raw ADC)</label>
    <div style="font-size:11px; color:#666; margin-bottom:5px;">Wartość przy całkowitej ciemności</div>
    <input type="number" id="rDark" onfocus="setEdit(true)" onblur="setEdit(false)"
           style="width:90%; background:#05060a; color:#6ab8ff; border:1px solid #333; padding:8px; margin:5px 0; border-radius:6px;">
    
    <label style="font-size:13px; display:block; margin-top:10px;">LDR Bright (Raw ADC)</label>
    <div style="font-size:11px; color:#666; margin-bottom:5px;">Wartość przy pełnym świetle</div>
    <input type="number" id="rBright" onfocus="setEdit(true)" onblur="setEdit(false)"
           style="width:90%; background:#05060a; color:#6ab8ff; border:1px solid #333; padding:8px; margin:5px 0; border-radius:6px;">
    
    <button class="btn save" style="margin-top:15px; padding:10px; font-size:14px;" onclick="applyAdv()">⚡ Zastosuj korekty</button>
    <div style="font-size:10px; color:#444; margin-top:8px; text-align:center;">Zmiany będą aktywne do restartu, chyba że klikniesz główny przycisk Zapisz.</div>
  </div>
</details>
<button class="btn reset" onclick="reset()">↺ Reset</button>
<button class="btn reset" onclick="location.href='/_ac'">🌐 Portal WiFi (AutoConnect)</button>

<div class="statusBox">
  <div class="titleSmall">Status urządzenia</div>
  <div id="statusBox">Ładowanie...</div>
</div>

<button class="btn reset" onclick="reboot()">🔄 Restartuj System</button>

</div>
</body>
</html>
)rawliteral";

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
static const int PIN_BUZZER = 4;    // BUZZER
static const int BUZZER_CH = 1;     // Kanał PWM 1 (0 mamy dla jasności)

// Digit select pins (to ULN2803 inputs) for 4 digits (common cathode)
static const int PIN_DIGIT_0 = 32;  // leftmost
static const int PIN_DIGIT_1 = 33;
static const int PIN_DIGIT_2 = 25;
static const int PIN_DIGIT_3 = 26;
// cyfry włączane stanem wysokim
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
char g_lastSyncTimeStr[32] = "Brak synchronizacji";
volatile int g_hour = 0;
volatile int g_minute = 0;
volatile int g_second = 0;
volatile float g_tempC = NAN;
float g_tempOffset = 0.0f;  // offest temperatury, wprowadzanie korekty w WebUI

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
volatile uint8_t g_brightness = 128;    // 0..255 logical brightness
volatile bool g_hardwareReady = false;  // flaga niewykorzystywane w tej wersji
// Kalibracja czujnika LDR - wartości "fabryczne" dla obudowy typu WOOD, możliowość korekty w WebUI
int g_rawDark = 3900;
int g_rawBright = 900;

// WiFi / portal / DNS
WebServer server(80);
AutoConnect portal(server);
AutoConnectConfig portalConfig;
AutoConnectOTA ota;
DNSServer dnsServer;
const byte DNS_PORT = 53;

String g_hostName;
String g_deviceId;
char id[5] = { 0 };  // 4 hex + '\0'

// DS18B20
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

// ALARM
int g_alarmH = 7, g_alarmM = 0;
bool g_alarmActive = false;
bool g_masterMute = false;           // Całkowite wyciszenie
int g_alarmMelody = 0;               // Wybór melodii (0 - klasyk, 1 - radosna, 2 - syrena
volatile bool g_isAlarming = false;  // Flaga, czy budzik aktualnie gra
uint8_t g_alarmDays = 127;           // Bity: 0-Niedz, 1-Pon... 6-Sob. 127 = wszystkie dni.

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
  digitalWrite(PIN_595_OE, HIGH);  // Wyłącz świecenie na czas przesyłu danych
  digitalWrite(PIN_595_LATCH, LOW);
  shiftOut(PIN_595_DATA, PIN_595_CLK, MSBFIRST, segments);
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

static const uint16_t FRAME_US = 4000;

void IRAM_ATTR onDisplayTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  allDigitsOff();  // Wygaszenie wszystkich cyfr

  if (g_otaActive) {
    write595(FONT_HEX[10]);  // Litera 'A' podczas aktualizacji
    digitOn(0);
  } else {
    write595(g_displaySeg[currentDigit]);
    digitOn(currentDigit);
    currentDigit = (currentDigit + 1) & 0x03;  // Przełącz na następną cyfrę
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
  // Używamy tego samego MUXa, co w przerwaniu timera
  portENTER_CRITICAL(&timerMux);
  g_displaySeg[0] = g_displayNext[0];
  g_displaySeg[1] = g_displayNext[1];
  g_displaySeg[2] = g_displayNext[2];
  g_displaySeg[3] = g_displayNext[3];
  portEXIT_CRITICAL(&timerMux);
}

void showBootId4() {
  g_showBootId = true;
  // Wpisujemy ID bezpośrednio do bufora, który czyta przerwanie (ISR)
  portENTER_CRITICAL(&timerMux);
  g_displaySeg[0] = segFromChar(id[0]);
  g_displaySeg[1] = segFromChar(id[1]);
  g_displaySeg[2] = segFromChar(id[2]);
  g_displaySeg[3] = segFromChar(id[3]);
  portEXIT_CRITICAL(&timerMux);

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
// BUZZER control
// -----------------------------------------------------------------------------
void initBuzzer() {
  // Jasność używa kanału 0. Spróbujmy użyć kanału 2,
  // który na pewno należy do innego "bloku" (Timer 1)
  ledcSetup(2, 2000, 8);
  ledcAttachPin(PIN_BUZZER, 2);
  ledcWrite(2, 0);
}
void beep(int freq = 2000, int duration = 100) {
  if (g_masterMute) return;  // Jeśli wyciszono, funkcja nic nie robi
  ledcWriteTone(2, freq);
  vTaskDelay(pdMS_TO_TICKS(duration));
  ledcWriteTone(2, 0);
  ledcWrite(2, 0);  // Po użyciu tonu warto wymusić powrót do 0, aby nie "brzęczało"
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

// --- DEFINICJA RODZAJU OBUDOWY ---
//  !!! KALIBRACJA PRZENIESIONA DO WEB UI !!!
//   Calibration points (ADC values)
//   DARK  → high ADC
//   BRIGHT → low ADC
//
// #ifdef CASE_WOOD
//   const float RAW_DARK = 3900;
//   const float RAW_BRIGHT = 900;
// #else  // Domyślnie lub jeśli wybrano CASE_PLA
//   const float RAW_DARK = 2500;
//   const float RAW_BRIGHT = 20;
// #endif
// ---------------------------------

uint8_t computeAutoBrightnessFromLDR() {
  // LDR is at the bottom (to GND), 10k at the top (to +3.3V)
  // → dark = high ADC value, bright = low ADC value
  int raw = analogRead(PIN_LDR_ADC);

  static float ema = 2000;  // Wstępna wartość średniej
  ema = 0.9f * ema + 0.1f * raw;

  // ZABEZPIECZENIE: jeśli wartości są identyczne, nie dzielimy przez zero
  if (abs(g_rawDark - g_rawBright) < 10) return 128;

  float x = (g_rawDark - ema) / (g_rawDark - g_rawBright);
  if (x < 0) x = 0;
  if (x > 1) x = 1;

  const int B_MIN = 5;
  const int B_MAX = 250;

  int out = (int)(B_MIN + x * (B_MAX - B_MIN));  // Zakres od 5 do 250

  // uncomment for measurements
  //Serial.printf("LDR raw=%d  ema=%.1f  norm=%.2f  brightness=%d\n", raw, ema, x, out);

  return (uint8_t)constrain(out, 0, 255);
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
    // Sprawdzamy czas rzadziej (co 100ms), co oszczędza energię i CPU
    if (getLocalTime(&ti, 0)) {  // 0 oznacza: sprawdź co masz w pamięci, nie czekaj (pierwotnie było 50)
      if (ti.tm_sec != lastSec) {
        lastSec = ti.tm_sec;
        g_hour = ti.tm_hour;
        g_minute = ti.tm_min;
        g_second = ti.tm_sec;
        g_timeValid = true;
      }
    }
    // 100ms to złoty środek dla zegara
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// -----------------------------------------------------------------------------
// TempTask
// -----------------------------------------------------------------------------
void TempTask(void *pv) {
  //sensors.setWaitForConversion(false); // aktywowane w setup()
  for (;;) {
    sensors.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(800));  // Czekamy na konwersję

    float t = NAN;
    int retryCount = 0;
    const int maxRetries = 3;

    while (retryCount < maxRetries) {
      t = sensors.getTempCByIndex(0);
      // Sprawdzamy czy odczyt jest poprawny (nie -127 i nie NAN)
      if (t > -80.0f && t < 150.0f) {
        break;  // Mamy poprawny odczyt, wychodzimy z pętli while
      }
      retryCount++;
      vTaskDelay(pdMS_TO_TICKS(150));  // Krótka przerwa przed ponowieniem
    }

    if (!isnan(t) && t > -80.0f) {
      g_tempC = t + g_tempOffset;  // Dodajemy offset
      g_tempValid = true;
    } else {
      Serial.printf("❌ Błąd DS18B20 po %d próbach!\n", maxRetries);
      // zmieniamy g_tempValid (plus zerujemy g_tempC), aby wyłączyć wyświetlanie temperatury
      g_tempValid = false;
      g_tempC = NAN;
    }

    vTaskDelay(pdMS_TO_TICKS(15000));
  }
}
// -----------------------------------------------------------------------------
// AlarmTask
// -----------------------------------------------------------------------------
// void AlarmTask(void *pv) {
//   for (;;) {
//     // Sprawdzanie co sekundę, czy czas alarmu nadszedł
//     if (g_alarmActive && g_hour == g_alarmH && g_minute == g_alarmM && g_second == 0) {
//       g_isAlarming = true;
//       Serial.println("⏰ ALARM URUCHOMIONY!");

//       // Graj przez 1 minutę lub do wyłączenia flagi g_isAlarming
//       for (int i = 0; i < 60 && g_isAlarming; i++) {
//         for (int j = 0; j < 3; j++) {  // Potrójne "pi-pi-pi"
//           beep(2500, 100);
//           vTaskDelay(pdMS_TO_TICKS(100));
//         }
//         vTaskDelay(pdMS_TO_TICKS(700));
//       }
//       g_isAlarming = false;
//     }
//     vTaskDelay(pdMS_TO_TICKS(1000));
//   }
// }
void AlarmTask(void *pv) {
  for (;;) {
    struct tm ti;
    if (g_alarmActive && getLocalTime(&ti, 0)) {
      // Sprawdzamy czy dzisiejszy dzień (ti.tm_wday) jest zaznaczony w masce bitowej
      bool dayMatch = (g_alarmDays & (1 << ti.tm_wday));

      if (dayMatch && g_hour == g_alarmH && g_minute == g_alarmM && g_second == 0) {
        g_isAlarming = true;
        Serial.printf("⏰ ALARM! Melodia: %d\n", g_alarmMelody);

        for (int i = 0; i < 60 && g_isAlarming; i++) {
          if (g_masterMute) {
            g_isAlarming = false;
            break;
          }  // Master Mute przerywa granie

          switch (g_alarmMelody) {
            case 1:  // Radosna (narastająca)
              beep(1500, 100);
              beep(2000, 100);
              beep(2500, 100);
              vTaskDelay(pdMS_TO_TICKS(700));
              break;
            case 2:  // Syrena (wysoki/niski)
              beep(3000, 400);
              beep(1500, 400);
              vTaskDelay(pdMS_TO_TICKS(200));
              break;
            default:  // 0 - Klasyk pi-pi-pi
              for (int j = 0; j < 3; j++) {
                beep(2500, 100);
                vTaskDelay(pdMS_TO_TICKS(100));
              }
              vTaskDelay(pdMS_TO_TICKS(700));
              break;
          }
        }
        g_isAlarming = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
// -----------------------------------------------------------------------------
// LogicTask
// -----------------------------------------------------------------------------
void LogicTask(void *pv) {
  Serial.println("LogicTask: Uruchomiony");
  static int lastSec = -1;
  static bool colon = false;

  for (;;) {
    // BEZPIECZNIK: Jeśli mamy czas, a flaga showID nadal wisi - wymuś start zegara
    if (g_timeValid && g_showBootId) {
      g_showBootId = false;
    }

    // Bezpieczne kopiowanie czasu w sekcji krytycznej
    int h, m, s;
    portENTER_CRITICAL(&timerMux);
    h = g_hour;
    m = g_minute;
    s = g_second;
    portEXIT_CRITICAL(&timerMux);

    if (g_showBootId) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    uint8_t localBuf[4] = { FONT_BLANK, FONT_BLANK, FONT_BLANK, FONT_BLANK };

    if (!g_timeValid) {
      for (int i = 0; i < 4; i++) localBuf[i] = FONT_MINUS;
    } else {
      uint32_t now = millis();
      bool showTempNow = (now % 20000 < 5000);

      if (g_second != lastSec) {
        lastSec = g_second;
        colon = (g_second % 2) == 0;
      }

      if (showTempNow && g_tempValid && !isnan(g_tempC)) {
        int temp = (int)roundf(g_tempC);
        // Dodatkowe zabezpieczenie zakresu dla 4 cyfr
        if (temp > -9 && temp < 100) {
          localBuf[0] = (temp >= 10) ? segForDigit(temp / 10) : FONT_BLANK;
          localBuf[1] = segForDigit(temp % 10);
          localBuf[2] = FONT_DEGREE;
          localBuf[3] = FONT_C;
        } else {
          // Jeśli temp poza zakresem, pokaż kreski
          for (int i = 0; i < 4; i++) localBuf[i] = FONT_MINUS;
        }
      } else {
        localBuf[0] = (h >= 10) ? segForDigit(h / 10) : FONT_BLANK;
        localBuf[1] = segForDigit(h % 10);
        if (colon) localBuf[1] |= SEG_DP;
        localBuf[2] = segForDigit(m / 10);
        localBuf[3] = segForDigit(m % 10);
      }
      if (g_forceWifiDot) localBuf[3] |= SEG_DP;
    }

    // KOPIOWANIE
    portENTER_CRITICAL(&timerMux);
    g_displaySeg[0] = localBuf[0];
    g_displaySeg[1] = localBuf[1];
    g_displaySeg[2] = localBuf[2];
    g_displaySeg[3] = localBuf[3];
    portEXIT_CRITICAL(&timerMux);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// Minimalny watchdog — tylko kropka czwartej cyfry przy utracie WiFi
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
  prefs.putFloat("tOffset", g_tempOffset);
  prefs.putInt("rDark", g_rawDark);
  prefs.putInt("rBright", g_rawBright);
  prefs.putInt("alH", g_alarmH);
  prefs.putInt("alM", g_alarmM);
  prefs.putBool("alOn", g_alarmActive);
  prefs.putBool("mMute", g_masterMute);
  prefs.putInt("alMel", g_alarmMelody);
  prefs.putUChar("alDays", g_alarmDays);
  prefs.end();
}

void resetSettings() {
  const uint8_t DEFAULT_BRIGHT = 128;
  const bool DEFAULT_AUTO = true;
  const float DEFAULT_TEMP_OFFSET = 0.0f;
  const int DEFAULT_RAW_DARK = 3900;
  const int DEFAULT_RAW_BRIGHT = 900;
  const int DEFAULT_AL_H = 7;
  const int DEFAULT_AL_M = 0;
  const bool DEFAULT_AL_ON = false;
  const bool DEFAULT_M_MUTE = false;
  const int DEFAULT_AL_MELODY = 0;
  const uint8_t DEFAULT_AL_DAYS = 127;

  prefs.begin("clock", false);
  prefs.putUChar("bright", DEFAULT_BRIGHT);
  prefs.putBool("autoB", DEFAULT_AUTO);
  prefs.putFloat("tOffset", DEFAULT_TEMP_OFFSET);
  prefs.putInt("rDark", DEFAULT_RAW_DARK);
  prefs.putInt("rBright", DEFAULT_RAW_BRIGHT);
  prefs.putInt("alH", DEFAULT_AL_H);
  prefs.putInt("alM", DEFAULT_AL_M);
  prefs.putBool("alOn", DEFAULT_AL_ON);
  prefs.putBool("mMute", DEFAULT_M_MUTE);
  prefs.putInt("alMel", DEFAULT_AL_MELODY);
  prefs.putUChar("alDays", DEFAULT_AL_DAYS);
  prefs.end();

  g_brightness = DEFAULT_BRIGHT;
  g_autoBrightness = DEFAULT_AUTO;
  g_tempOffset = DEFAULT_TEMP_OFFSET;
  g_rawDark = DEFAULT_RAW_DARK;
  g_rawBright = DEFAULT_RAW_BRIGHT;
  g_alarmH = DEFAULT_AL_H;
  g_alarmM = DEFAULT_AL_M;
  g_alarmActive = DEFAULT_AL_ON;
  g_masterMute = DEFAULT_M_MUTE;
  g_alarmMelody = DEFAULT_AL_MELODY;
  g_alarmDays = DEFAULT_AL_DAYS;
}

void loadSettings() {
  prefs.begin("clock", false);
  g_brightness = prefs.getUChar("bright", 150);
  g_autoBrightness = prefs.getBool("autoB", true);
  g_tempOffset = prefs.getFloat("tOffset", 0.0f);
  g_rawDark = prefs.getInt("rDark", 3900);
  g_rawBright = prefs.getInt("rBright", 900);
  g_alarmH = prefs.getInt("alH", 7);
  g_alarmM = prefs.getInt("alM", 0);
  g_alarmActive = prefs.getBool("alOn", false);
  g_masterMute = prefs.getBool("mMute", false);
  g_alarmMelody = prefs.getInt("alMel", 0);
  g_alarmDays = prefs.getUChar("alDays", 127);
  prefs.end();
}

// -----------------------------------------------------------------------------
// Time setup (NTP + DST)
// -----------------------------------------------------------------------------
/* TESTY ZMIANY CZASU
  //configTzTime("CET-1CEST,M3.5.0/2,M4.3.1/9", "tempus1.gum.gov.pl", "pl.pool.ntp.org", "tempus2.gum.gov.pl");
  M3.5.0/2 — początek czasu letniego
  Format: M<miesiąc>.<tydzień>.<dzień tygodnia>/<godzina>
  M3 – marzec
  5 – piąty tydzień miesiąca (czyli ostatnia niedziela marca)
  0 – niedziela
  /2 – o godzinie 2:00 czasu lokalnego
  Start DST: ostatnia niedziela marca o 02:00

  M10.5.0/3 — koniec czasu letniego
  M10 – październik
  5 – piąty tydzień (ostatnia niedziela października)
  0 – niedziela
  /3 – o godzinie 3:00 czasu lokalnego
  Koniec DST: ostatnia niedziela października o 03:00
  */
void setupTime() {
  configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3",
               "tempus1.gum.gov.pl",
               "tempus2.gum.gov.pl",
               "pl.pool.ntp.org");

  struct tm timeinfo;
  unsigned long start = millis();

  while (millis() - start < 3000) {
    esp_task_wdt_reset();  // WDT na wypadek jeśli router jest wyłączony, pętla while w setupTime może blokować start zegara
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
// Funkcja wywoływana automatycznie po udanej synchronizacji
void timeSyncCallback(struct timeval *tv) {
  Serial.println("----------------------------------------------");
  Serial.println("🔔 SUKCES: Czas został zsynchronizowany z NTP!");

  struct tm ti;
  getLocalTime(&ti);
  // Formatujemy datę i godzinę sukcesu
  snprintf(g_lastSyncTimeStr, sizeof(g_lastSyncTimeStr), "%02d.%02d %02d:%02d:%02d",
           ti.tm_mday, ti.tm_mon + 1, ti.tm_hour, ti.tm_min, ti.tm_sec);
  //Serial.println(g_lastSyncTimeStr);
  Serial.print("Aktualny czas: ");
  Serial.println(&ti, "%A, %B %d %Y %H:%M:%S");
  Serial.println("----------------------------------------------");
  beep(3500, 25);
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
  portalConfig.reconnectInterval = 6;  // Przerwa między próbami (w jednostkach 5s, tu: 30s)
  portalConfig.retainPortal = true;
  portalConfig.apid = String("ESP32-Clock-") + id;
  portalConfig.psk = "Al@m@kot@";
  portalConfig.hostName = g_hostName.c_str();
  portalConfig.menuItems |= AC_MENUITEM_DELETESSID;
  portalConfig.boundaryOffset = 64;  // Zwiększa stabilność bufora przy dużym HTML
  portalConfig.immediateStart = false;
  portalConfig.homeUri = "/config";  // Wymusza stronę główną portalu
  portalConfig.bootUri = AC_ONBOOTURI_HOME;
  portal.config(portalConfig);

  // OTA callbacks
  ota.onStart([]() {
    g_otaActive = true;
  });
  ota.onEnd([]() {
    g_otaActive = false;
    Serial.println("OTA Zakończone. Restart za 5 sekund...");

    // Tworzymy zadanie restartu, aby dać czas na wysłanie odpowiedzi do przeglądarki
    xTaskCreate([](void *pv) {
      vTaskDelay(pdMS_TO_TICKS(5000));  // 5 sekund zwłoki
      ESP.restart();
    },
                "reboot", 2048, NULL, 5, NULL);
  });

  // Root redirect -> /config
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Location", "/config", true);
    server.send(302, "text/plain", "");
  });

  // /status endpoint
  server.on("/status", []() {
    char s[1024];
    int out = 0;
    struct tm ti;
    char dateBuf[32] = "--.--.----";
    const char *days[] = { "Niedziela", "Poniedziałek", "Wtorek", "Środa", "Czwartek", "Piątek", "Sobota" };
    const char *dayName = "---";

    if (getLocalTime(&ti, 0)) {
      snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", ti.tm_mday, ti.tm_mon + 1, ti.tm_year + 1900);
      dayName = days[ti.tm_wday];
    }

    // Blok 1: Tożsamość i czas
    out += snprintf(s + out, sizeof(s) - out, "id=%s\nhostname=%s\ntime=%02d:%02d:%02d\ndate=%s\nday=%s\n",
                    g_deviceId.c_str(), g_hostName.c_str(), g_hour, g_minute, g_second, dateBuf, dayName);

    // Blok 2: NTP i Temperatura
    out += snprintf(s + out, sizeof(s) - out, "lastSync=%s\ntempC=%.1f\nd18b20_res=%d\ntOff=%.1f\n",
                    g_lastSyncTimeStr, g_tempC, getDS18B20Resolution(), g_tempOffset);

    // Blok 3: LDR i Jasność
    int rawLDR = analogRead(PIN_LDR_ADC);
    out += snprintf(s + out, sizeof(s) - out, "rDark=%d\nrBright=%d\nrawLDR=%d\nbrightness=%d\nautoBrightness=%d\n",
                    g_rawDark, g_rawBright, rawLDR, g_brightness, (g_autoBrightness ? 1 : 0));

    // Blok 4: Budzik
    out += snprintf(s + out, sizeof(s) - out, "isAlarming=%d\nalTime=%02d:%02d\nalActive=%d\n",
                    (g_isAlarming ? 1 : 0), g_alarmH, g_alarmM, (g_alarmActive ? 1 : 0));

    // Blok 5: Sieć
    out += snprintf(s + out, sizeof(s) - out, "wifi=%s\n", (WiFi.status() == WL_CONNECTED ? "connected" : "not_connected"));

    if (WiFi.status() == WL_CONNECTED) {
      out += snprintf(s + out, sizeof(s) - out, "ip=%s\nrssi=%d\nmdns=http://%s.local/\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI(), g_hostName.c_str());
    }

    server.send(200, "text/plain", s);
  });

  // /config UI
  // server.on("/config", HTTP_GET, []() {
  //   // Flag_P mówi serwerowi, że tablica CONFIG_HTML znajduje się w PROGMEM
  //   server.send_P(200, "text/html", CONFIG_HTML);
  // });
  // /config UI w wersji wysyłającej dane w porcjach po 1KB
  server.on("/config", HTTP_GET, []() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");  // Wyślij nagłówek

    const char *ptr = CONFIG_HTML;
    size_t fullLen = strlen_P(CONFIG_HTML);
    size_t sentLen = 0;
    const size_t chunkSize = 1024;  // Porcje po 1KB

    while (sentLen < fullLen) {
      size_t currentChunk = (fullLen - sentLen > chunkSize) ? chunkSize : (fullLen - sentLen);
      server.sendContent_P(ptr + sentLen, currentChunk);
      sentLen += currentChunk;
      // Bardzo ważne: oddaj na chwilę procesor systemowi WiFi
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    server.sendContent("");  // Zakończ transmisję
  });

  // /set endpoint
  // --- ZMIENIONY ENDPOINT /set ---
  server.on("/set", []() {
    if (server.hasArg("bright")) {
      g_brightness = server.arg("bright").toInt();
    }
    if (server.hasArg("auto")) {
      g_autoBrightness = (server.arg("auto") == "1");
    }
    if (server.hasArg("tOff")) g_tempOffset = server.arg("tOff").toFloat();
    if (server.hasArg("rDark")) g_rawDark = server.arg("rDark").toInt();
    if (server.hasArg("rBright")) g_rawBright = server.arg("rBright").toInt();

    applyBrightness(g_brightness);  // Reaguje od razu, ale nie zapisuje do Flash!

    if (server.hasArg("alTime")) {
      String t = server.arg("alTime");
      g_alarmH = t.substring(0, 2).toInt();
      g_alarmM = t.substring(3, 5).toInt();
    }
    if (server.hasArg("alOn")) g_alarmActive = (server.arg("alOn") == "1");
    if (server.hasArg("stopAlarm")) g_isAlarming = false;

    server.send(200, "text/plain", "OK");
  });

  // --- NOWY ENDPOINT /save ---
  server.on("/save", []() {
    saveSettings();  // Zapisuje do Flash tylko po kliknięciu "Zapisz"
    server.send(200, "text/plain", "Zapisano");
  });

  // /reset endpoint
  server.on("/reset", []() {
    resetSettings();
    server.send(200, "text/plain", "OK: reset");
  });

  // /reboot endpoint
  server.on("/reboot", []() {
    server.send(200, "text/plain", "Restartowanie...");
    xTaskCreate([](void *pv) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      ESP.restart();
    },
                "reboot", 2048, NULL, 5, NULL);
  });

  server.onNotFound([]() {
    // Jeśli ktoś zapyta o cokolwiek innego (np. test internetu),
    // przekieruj go na naszą stronę konfiguracji
    server.sendHeader("Location", "/config", true);
    server.send(302, "text/plain", "");
  });

  ota.attach(portal);
  portal.begin();
  // START DNS SERVER
  vTaskDelay(pdMS_TO_TICKS(500));  // Daj mu pół sekundy na wstanie interfejsu AP
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  Serial.print("DNS Server started on IP: ");
  Serial.println(WiFi.softAPIP());

  if (MDNS.begin(g_hostName.c_str())) {
    MDNS.addService("http", "tcp", 80);
  }

  bool firstSyncDone = false;

  for (;;) {

    // Synchronizuj czas tylko jeśli mamy WiFi i jeszcze tego nie zrobiliśmy
    if (!firstSyncDone && WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi połączone, uruchamiam NTP...");
      setupTime();
      firstSyncDone = true;
    }

    wifiWatchdog();

    if (WiFi.getMode() & WIFI_AP) {
      // dnsServer potrzebuje jak najczęstszego wywoływania processNextRequest
      dnsServer.processNextRequest();
      portal.handleClient();
      vTaskDelay(1);  // Minimalny "oddech" dla systemu
    } else {
      portal.handleClient();
      vTaskDelay(pdMS_TO_TICKS(50));
    }
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
  pinMode(PIN_595_OE, OUTPUT);
  digitalWrite(PIN_595_OE, HIGH);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_595_OE, PWM_CH);

  pinMode(PIN_LDR_ADC, INPUT);
  analogReadResolution(12);

  applyBrightness(g_brightness);
  // flaga niewykorzystywana w tej wersji
  g_hardwareReady = true;  // <--- TUTAJ odblokowujemy sterowanie jasnością
}
// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  initBuzzer();

  WiFi.setAutoReconnect(false);  // true: Pozwól ESP32 dbać o połączenie - koliduje z AC ; false: nie przeszkadza AutoConnect
  WiFi.persistent(false);        // NIE zapisuj danych WiFi przy każdym połączeniu (oszczędza Flash)

  // Skrócenie czasu między odpytaniami SNTP (DLA TESTÓW)
  // Domyślnie jest są to 3 godziny ! - zweryfikowane doświadczalnie (3x3600000 ms = 10 800 000 ms).
  // Ustawiamy np. na 20 sekund (20000 ms), aby szybko zobaczyć efekty.
  // UWAGA: Minimalny zalecany czas dla serwerów publicznych to 15s.

  // sntp_set_sync_interval(60000); // 1 minuta TYLKO DLA TESTÓW

  // Rejestracja powiadomienia (musi być przed configTzTime)
  sntp_set_time_sync_notification_cb(timeSyncCallback);

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

  esp_task_wdt_init(30, true);  // 30 sekund timeoutu (przy 10s i braku zasięgu znanej WiFi wpadał w pętlę restartów; można zwiększyć do 60 sek)
  esp_task_wdt_add(NULL);       // Dodaj główny wątek do monitorowania

  displayTimer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(displayTimer, &onDisplayTimer, true);
  timerAlarmWrite(displayTimer, FRAME_US, true);
  timerAlarmEnable(displayTimer);

  xTaskCreatePinnedToCore(TimeTask, "Time", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TempTask, "Temp", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(LogicTask, "Logic", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(BrightnessTask, "Bright", 2048, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(WiFiTask, "WiFi", 8192, nullptr, 1, nullptr, 0);  // przeniesienie obsługi WiFi na dedykowany rdzeń 0
  xTaskCreate(AlarmTask, "Alarm", 2048, NULL, 1, NULL);
}

void loop() {
  esp_task_wdt_reset();  // "Karmienie" psa - jeśli to nie nastąpi, nastąpi reset
  vTaskDelay(pdMS_TO_TICKS(1000));
}
