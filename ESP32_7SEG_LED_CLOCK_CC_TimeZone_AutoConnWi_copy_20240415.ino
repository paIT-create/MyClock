/*
  Prosty zegar z modułem ESP32, wyświetlaczem LED 4x7seg ("CC" wsp.Katoda, 74HC595, ULN2803, regulacja jasnosci), synchronizowany z serwera NTP przez sieć WiFi.
  Opcja termometru DS18B20
  TESTY
  paIT 22.01.2022
  27.03.2022 Dodana obsluga strefy czasowej / DST - bibloteka ezTime.h
  15.04.2022 Dodana obsługa konfiguracji WiFi przez AutoConnect
  31.10.2022 Poprawki zwiazane ze zmiana czasu - automatycza aktualizacja DST bez restartu programu
*/
#include <Arduino.h>
//#include <WiFi.h>
#include <WebServer.h>
#include <AutoConnect.h>
//#include <ezTime.h>
#include "time.h"
#include "sntp.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Insert your network credentials
/*
const char* ssid     = "FunBox2-367E";
const char* password = "2F4FE5F7729614C6311A2CEF6D";
*/
WebServer Server;
AutoConnect Portal(Server);
AutoConnectConfig Config;

void rootPage() {
  String  content =
    "<html>"
    "<head>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<script type=\"text/javascript\">"
    "setTimeout(\"location.reload()\", 2000);"
    "</script>"
    "</head>"
    "<body>"
    "<h3 align=\"center\" style=\"color:grey;margin:10px;\">{{HostName}}</h3>"
    "<h2 align=\"center\" style=\"color:green;margin:20px;\">Witaj !</h2>"
    "<h2 align=\"center\" style=\"color:red;margin:20px;\">{{DateTime}}</h2>"
    "<p style=\"text-align:center;\">Reload the page to update the time.</p>"
    "<p></p><p style=\"padding-top:15px;text-align:center\">" AUTOCONNECT_LINK(COG_32) "</p>"
    "</body>"
    "</html>";
  static const char *wd[7] = { "Sun","Mon","Tue","Wed","Thr","Fri","Sat" };
  struct tm *tm;
  time_t  t;
  char    dateTime[26];

  t = time(NULL);
  tm = localtime(&t);
  sprintf(dateTime, "%04d/%02d/%02d %s %02d:%02d:%02d",
    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
    wd[tm->tm_wday],
    tm->tm_hour, tm->tm_min, tm->tm_sec);
  content.replace("{{DateTime}}", String(dateTime));
  content.replace("{{HostName}}", String(WiFi.getHostname()));
  Server.send(200, "text/html", content);
}

// NTP Server Details
//const char* ntpServer           = "pl.pool.ntp.org";               //"pl.pool.ntp.org"; "tempus1.gum.gov.pl";
const char *ntpServer1 = "pl.pool.ntp.org";
const char *ntpServer2 = "tempus1.gum.gov.pl";
const char *ntpServer3 = "tempus2.gum.gov.pl";
long        gmtOffset_sec       = 0;                                  // bylo 3600, ustawione na 0 - obslugiwane przez ezTime i Timezone
long        daylightOffset_sec  = 0;                                  // obslugiwane przez ezTime i Timezone
#define     tzRule              "CET-1CEST,M3.5.0/2,M10.5.0/3"        // kod Posix strefy czasowej dla PL
//#define     ntpIntervalTime     1801                                  // interval in sec for NTP query
//Timezone myTZ;                                                        // tworzy obiekt myTZ dla typu Timezone

// LED Display
#define clockPin      12          //74HC595 Pin 11
#define latchPin      13          //74HC595 Pin 12
#define dataPin       14          //74HC595 Pin 14
#define oePin         27          //74HC595 Pin 13 (+PWM brightness control)
#define ldr           34          // LDR (Light Dependent Resistor)
#define ONE_WIRE_BUS  15          // Data wire is plugged into port 15 ESP32
#define ledPin         2          // OnBoard LED pin

// LDR config for PLA and WOOD type covers
// uncoment one for transparent PLA or WOOD cover
const char* coverTyp = "PLA";
//const char* coverTyp = "WOOD";

long ldrInMin = 0;
long ldrInMax = 4095;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

const int digitPins[4] =
{
  32,                             // element 0 - katoda pierwsza cyfra  - ULN2803 pin 1
  33,                             // element 1 - katoda druga cyfra     - ULN2803 pin 2
  25,                             // element 2 - katoda trzecia cyfra   - ULN2803 pin 3
  26                              // element 3 - katoda czwarta cyfra   - ULN2803 pin 4
};

const byte digit[13] =            //seven segment digits in bits and marks for temp display
{
  B00111111,                      //0
  B00000110,                      //1
  B01011011,                      //2
  B01001111,                      //3
  B01100110,                      //4
  B01101101,                      //5
  B01111101,                      //6
  B00000111,                      //7
  B01111111,                      //8
  B01101111,                      //9
  B00000000,                      //Blank
  B01100011,                      // st. "o"
  B00111001                       // "C"
};

const byte splash[4] =            //seven segment digits in bits and marks for temp display
{
  B00010000,                      //i
  B01010100,                      //n
  B00010000,                      //i
  B01111000                       //t
};

// setting PWM properties
const int freq        = 5000;
const int ledChannel  = 0;
const int resolution  = 8;
byte      pwmValue    = 100;

int digitBuffer[4] = {0, 1, 2, 3};
byte digitScan = 0;

byte last_second, second_, minute_, minute_2, minute_3, hour_, hour_0, hour_1, day_, month_;
int year_;
boolean clockready = false;

// timers for temperature display
unsigned long lastTimer = 0;
unsigned long timerDelay = 15000;
boolean showTemp = false;

int dsResolution = 12;               // resolution for DS sensor: 9,10,11 or 12 bits = sensor accuracy
/*
// konfiguracja czasu, sprawdzenie DST
void configTimeSet() {
  if (!myTZ.isDST()) {
    daylightOffset_sec = 0;
  }
  else if (myTZ.isDST() && (gmtOffset_sec == 3600)) {
    daylightOffset_sec = 3600;
  }
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}
*/
// pobieranie i dekodowanie czasu
void readTime() {

  struct tm timeinfo;

  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    splashScreenDisp();
  }

  second_ = timeinfo.tm_sec;

  if (last_second != second_) {
    //configTimeSet();
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S ");
//    Serial.println(getTimezoneName());
    //Serial.print("Time Offset: ");
    //Serial.print(gmtOffset_sec);
    //Serial.println(" sec");
    //Serial.print("DST Offset: ");
    //Serial.print(daylightOffset_sec);
    //Serial.println(" sec");

    minute_ = timeinfo.tm_min;
    hour_   = timeinfo.tm_hour;
    //    day_    = timeinfo.tm_mday;
    //    month_  = timeinfo.tm_mon + 1;
    //    year_   = timeinfo.tm_year + 1900;

    minute_3 = minute_ % 10;
    minute_2 = minute_ / 10;
    hour_1   = hour_   % 10;
    hour_0   = hour_   / 10;

    if (hour_ < 10) {           // dla godziny 00 do 09 w miejsce dziesiatek
      hour_0 = 10;              // wstawiam znak Blank aby wyczyscic pole dziesiatek godzin - nieznaczace zero
    }

    digitBuffer[3] = minute_3;
    digitBuffer[2] = minute_2;
    digitBuffer[1] = hour_1;
    digitBuffer[0] = hour_0;

    last_second = second_;
    displaysBrightSet();
  }
  updateDisp();
}

// odczyt temperatury i zapis do bufora cyfr
void readTemp() {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
    int res = sensors.getResolution(tempDeviceAddress);
    Serial.print("Device resolution is ");
    Serial.println(res);
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);
  int tempCC = tempC;

  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    digitalWrite(ledPin, HIGH);
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.print(tempC);
    int roundTemp = round(tempC);
    Serial.print("\t-> round temp = ");
    Serial.println(roundTemp);
    digitBuffer[3] = 12;
    digitBuffer[2] = 11;
//    digitBuffer[1] = roundTemp % 10;
//    digitBuffer[0] = roundTemp / 10;
    digitBuffer[1] = tempCC % 10;
    digitBuffer[0] = tempCC / 10;
    digitalWrite(ledPin, LOW);
    showTemp = true;
    lastTimer = millis();
    while ((millis() - lastTimer) < (timerDelay / 3)) {
      updateDisp();
      displaysBrightSet();
    }
    showTemp = false;
  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }
}

// wyswietlanie danych na wyswietlaczu
void updateDisp() {

  for (byte j = 0; j < 4; j++)
  {
    digitalWrite(digitPins[j], LOW);                       // all digits OFF
  }

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000);         // all segments OFF
  digitalWrite(latchPin, HIGH);

  delayMicroseconds(100);
  digitalWrite(digitPins[digitScan], HIGH);                  // digit ON

  digitalWrite(latchPin, LOW);                              // start sending data to register
  if (digitScan == 1)                                       // for 2nd digit
  {
    if (((second_ % 2) == 0) & (showTemp == false))
    {
      shiftOut(dataPin, clockPin, MSBFIRST, (digit[digitBuffer[digitScan]] | B10000000));        //print the decimal point on the 2nd digit for odd seconds
    }
    else
    {
      shiftOut(dataPin, clockPin, MSBFIRST, digit[digitBuffer[digitScan]]);
    }
  }
  else
  {
    shiftOut(dataPin, clockPin, MSBFIRST, digit[digitBuffer[digitScan]]);
  }
  digitalWrite(latchPin, HIGH);                           // end writind data

  delayMicroseconds(2000);                                // pause for display digit
  digitalWrite(digitPins[digitScan], LOW);               // digit OFF
  digitScan++;                                            // next digit
  if (digitScan > 3) digitScan = 0;                       // 4 digits range 0 - 3
}

// wyswietlanie "init" na wyswietlaczu
void splashScreenDisp() {

  for (byte j = 0; j < 4; j++)
  {
    digitalWrite(digitPins[j], LOW);                       // all digits OFF
  }

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000);         // all segments OFF
  digitalWrite(latchPin, HIGH);

  delayMicroseconds(100);
  digitalWrite(digitPins[digitScan], HIGH);                  // digit ON

  digitalWrite(latchPin, LOW);                                              // start sending data to register
  shiftOut(dataPin, clockPin, MSBFIRST, splash[digitBuffer[digitScan]]);
  digitalWrite(latchPin, HIGH);                                             // end writind data

  delayMicroseconds(2000);                                // pause for display digit
  digitalWrite(digitPins[digitScan], LOW);               // digit OFF
  digitScan++;                                            // next digit
  if (digitScan > 3) digitScan = 0;                       // 4 digits range 0 - 3
}

// Regulacja jasnosci wyswietlaczy LED w zaleznosci od natezenia oswietlenia
void displaysBrightSet() {
  int luxLED = map(analogRead(ldr), ldrInMin, ldrInMax, 0, 250);    // 2200 dla transparentnej plyty czolowej, 4000 dla WOOD
  //Serial.print(analogRead(ldr));
  //Serial.print("\t-> LuxLED ");
  //Serial.println(luxLED);
  if (luxLED < 0) luxLED = 0;
  if (luxLED > 250) luxLED = 250;
  pwmValue = luxLED;
  ledcWrite(ledChannel, pwmValue);
}

// statyczne wyswietlanie segmentow "d" na wyswietlaczu
void staticDsegDisp() {
    for (int i = 0; i < 4; i++)
  {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], HIGH);                        // all digits ON
  }

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00001000);         // all "d" segments ON
  digitalWrite(latchPin, HIGH);
/*
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000);         // all segments OFF
  digitalWrite(latchPin, HIGH);
*/  
}
// statyczne wyswietlanie segmentow "a" na wyswietlaczu
void staticAsegDisp() {
    for (int i = 0; i < 4; i++)
  {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], HIGH);                        // all digits ON
  }

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000001);         // all "a" segments ON
  digitalWrite(latchPin, HIGH);
/*
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000);         // all segments OFF
  digitalWrite(latchPin, HIGH);
*/  
}
// statyczne wyswietlanie segmentow "g" na wyswietlaczu
void staticGsegDisp() {
    for (int i = 0; i < 4; i++)
  {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], HIGH);                        // all digits ON
  }

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B01000000);         // all "g" segments ON
  digitalWrite(latchPin, HIGH);
/*
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000);         // all segments OFF
  digitalWrite(latchPin, HIGH);
*/  
}

// handle WiFi events
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case SYSTEM_EVENT_WIFI_READY: 
      Serial.println("WiFi interface ready.");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points.");
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi client started.");
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi clients stopped.");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to access point.");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point.");
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println("Authentication mode of access point has changed.");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("Lost IP address and IP address is reset to 0");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode.");
      staticAsegDisp();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode.");
      staticDsegDisp();
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("!!! Restarting in 30 seconds !!!");
        staticGsegDisp();
        delay(30000);
        ESP.restart();
      }
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode.");
      staticDsegDisp();
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("!!! Restarting in 60 seconds !!!");
        staticGsegDisp();
        delay(60000);
        ESP.restart();
      }
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode.");
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.println("WiFi access point started.");
      staticAsegDisp();
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("WiFi access point stopped.");
      staticDsegDisp();
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("!!! Restarting in 10 seconds !!!");
        staticGsegDisp();
        delay(10000);
        ESP.restart();
      }
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Client connected.");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("Client disconnected.");
      staticAsegDisp();
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.println("Assigned IP address to client.");
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println("Received probe request.");
      break;
    case SYSTEM_EVENT_GOT_IP6:
      Serial.println("IPv6 is preferred.");
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("Ethernet started.");
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped.");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected.");
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected.");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address.");
      break;
    default: break;
}}
/* AVAILABLE EVENTS:
    SYSTEM_EVENT_WIFI_READY = 0,           //< ESP32 WiFi ready
    SYSTEM_EVENT_SCAN_DONE,                //< ESP32 finish scanning AP
    SYSTEM_EVENT_STA_START,                //< ESP32 station start
    SYSTEM_EVENT_STA_STOP,                 //< ESP32 station stop
    SYSTEM_EVENT_STA_CONNECTED,            //< ESP32 station connected to AP
    SYSTEM_EVENT_STA_DISCONNECTED,         //< ESP32 station disconnected from AP
    SYSTEM_EVENT_STA_AUTHMODE_CHANGE,      //< the auth mode of AP connected by ESP32 station changed
    SYSTEM_EVENT_STA_GOT_IP,               //< ESP32 station got IP from connected AP
    SYSTEM_EVENT_STA_LOST_IP,              //< ESP32 station lost IP and the IP is reset to 0
    SYSTEM_EVENT_STA_WPS_ER_SUCCESS,       //< ESP32 station wps succeeds in enrollee mode
    SYSTEM_EVENT_STA_WPS_ER_FAILED,        //< ESP32 station wps fails in enrollee mode
    SYSTEM_EVENT_STA_WPS_ER_TIMEOUT,       //< ESP32 station wps timeout in enrollee mode
    SYSTEM_EVENT_STA_WPS_ER_PIN,           //< ESP32 station wps pin code in enrollee mode
    SYSTEM_EVENT_AP_START,                 //< ESP32 soft-AP start
    SYSTEM_EVENT_AP_STOP,                  //< ESP32 soft-AP stop
    SYSTEM_EVENT_AP_STACONNECTED,          //< a station connected to ESP32 soft-AP
    SYSTEM_EVENT_AP_STADISCONNECTED,       //< a station disconnected from ESP32 soft-AP
    SYSTEM_EVENT_AP_STAIPASSIGNED,         //< ESP32 soft-AP assign an IP to a connected station
    SYSTEM_EVENT_AP_PROBEREQRECVED,        //< Receive probe request packet in soft-AP interface
    SYSTEM_EVENT_GOT_IP6,                  //< ESP32 station or ap or ethernet interface v6IP addr is preferred
    SYSTEM_EVENT_ETH_START,                //< ESP32 ethernet start
    SYSTEM_EVENT_ETH_STOP,                 //< ESP32 ethernet stop
    SYSTEM_EVENT_ETH_CONNECTED,            //< ESP32 ethernet phy link up
    SYSTEM_EVENT_ETH_DISCONNECTED,         //< ESP32 ethernet phy link down
    SYSTEM_EVENT_ETH_GOT_IP,               //< ESP32 ethernet got IP from connected AP
*/

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
  Serial.println("Got time adjustment from NTP!");
  readTime();
  clockready = true;
}

void setup() {
  Serial.begin(115200);

  WiFi.onEvent(WiFiEvent);
  
  // set variable for brightnes config dependind on PLA / WOOD type covers
  if (coverTyp == "PLA") {
  ldrInMin = 50;
  ldrInMax = 2200;
  }
  else if (coverTyp == "WOOD") {
  ldrInMin = 50;
  ldrInMax = 4000;
  }
  
  sensors.begin();                                        // Start up the library for temp sensor
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, dsResolution); // set resolution for DS sensor
  sensors.setWaitForConversion(false);                    // Async reading of Dallas Temperature Sensors in order not to wait constant time for read temperature
  sensors.requestTemperatures();

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(oePin, OUTPUT);
  digitalWrite(oePin, LOW);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  ledcSetup(ledChannel, freq, resolution);                // configure LED PWM functionalitites
  ledcAttachPin(oePin, ledChannel);                       // attach the channel to the GPIO to be controlled
//  ledcWrite(ledChannel, pwmValue);                        // poczatkowe ustawienie jasnosci wyswietlacza - zmienna pwmValue lub...
  displaysBrightSet();                                    // poczatkowe ustawienie jasnosci wyswietlacza - automatyczne

  staticDsegDisp();

   // set notification call-back function
  sntp_set_time_sync_notification_cb(timeavailable);

  /**
   * NTP server address could be aquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE aquired NTP server address
   */
  sntp_servermode_dhcp(1);  // (optional)

  /**
   * This will set configured ntp servers and constant TimeZone/daylightOffset
   * should be OK if your time zone does not need to adjust daylightOffset twice a year,
   * in such a case time adjustment won't be handled automagicaly.
   */
  //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  /**
   * A more convenient approach to handle TimeZones with daylightOffset 
   * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
   * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
   */
  configTzTime(tzRule, ntpServer1, ntpServer2, ntpServer3);
  
  // Setup for AutoConnect Wi-Fi
  Config.apid = "AP-ESP32Clock-" + String((uint32_t)(ESP.getEfuseMac() >> 32), HEX);      // Change default SSID for SoftAP
  Config.psk  = "Al@m@kot@!";                                                             // Change default Password for SoftAP
  Config.hostName = "MyClock-" + String((uint32_t)(ESP.getEfuseMac() >> 32), HEX);        // Change default Host Name for station
  Config.beginTimeout = 90000;                                                            // timeout [ms] value used when trying to connect to the specified AP.
  Config.portalTimeout = 120000;                                                          // Specify the timeout value of the captive portal in [ms] units.
  Config.autoReconnect = true;                                                            // Attempt automatic reconnection.
  Config.reconnectInterval = 10;                                                          // Seek interval time is 5[min] (10 x 30s); 180[s] for value of 6 (x 30s).
  Config.menuItems = Config.menuItems | AC_MENUITEM_DELETESSID;                           // enable the credentials removal feature in OpenSSIDs menu
  
  // The AutoConnect ticker indicates the WiFi connection status in the following three flicker patterns:
  // Short blink: The ESP module stays in AP_STA mode.
  // Short-on and long-off: No STA connection state. (i.e. WiFi.status != WL_CONNECTED)
  // No blink: WiFi connection with access point established and data link enabled. (i.e. WiFi.status = WL_CONNECTED)
  Config.ticker = true;
  Config.tickerPort = ledPin;
  Config.tickerOn = HIGH;
  // store AC config
  Portal.config(Config);

  // Behavior a root path of WebServer.
  Server.on("/", rootPage);

  // Connect to Wi-Fi
      if (Portal.begin()) {
        Serial.println("WiFi connected: " + WiFi.localIP().toString());
        Serial.print("Host name: ");
        Serial.println(WiFi.getHostname());
      }
      else {
        Serial.println("Connection failed.");
        while (true) { yield(); }
      }

  // Uncomment the line below to see what it does behind the scenes ezTime
  //setDebug(INFO);

  //myTZ.setLocation(F("PL"));
  //myTZ.setPosix(tzRule);
  //myTZ.setDefault();

  //setServer(ntpServer);
  //setInterval(ntpIntervalTime);                   // interval in sec for NTP query
  
  //waitForSync();

  //gmtOffset_sec = -(getOffset() * 60);

  //Serial.print("Time Offset is ");
  //Serial.print(gmtOffset_sec);
  //Serial.println(" sec");

  // Init and get the time
  //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //configTimeSet();
}

void loop() {
  // for events from ezTime library
  //events();

  if (!clockready) {staticGsegDisp();}

  readTime();

  // odczyt temp co 15 sec (zmienna timerDelay)
  if (clockready){
    if ((millis() - lastTimer) > timerDelay) {
      readTemp();
      lastTimer = millis();
    }
  }
  // for WiFi AutoConnect portal
  Portal.handleClient();
}
