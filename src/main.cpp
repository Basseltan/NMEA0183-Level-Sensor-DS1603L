#define SKETCH_NAME    __FILE__
#define SKETCH_VERSION __DATE__ " " __TIME__
#define SKETCH_ABOUT   "ESP8266 DS1603L Sensor with WiFi Manager, Telnet and NMEA broadcast sender"
/*****************************************************************************************************
 * DS1603L Ultrasonic Level Sensor with NMEA 0183 UDP sender, using Tzapu´s WiFiManager
 * Hans-Jürgen Schliepkorte
 * 05.03.2023
 * 
 * The Tx Pin of the DS1603L has to be connected to D2 (max 3.3V!).
 * 
 * For using a 0-10V converter for the tank level a PWM is configured for D5 (GPIO14). In the two 
 * minutes startup phase the PWM is set to half (128) for calibration of the 0-10V adapter.
 * 
 * In addition a used 12V voltage could be measured at Pin D5 (analog input A0). A voltage devider must 
 * be used for A0 of ESP8266. I calculated the resistors (R1=5k6, R2=1k) for a maximum voltage of 
 * 20.5 V so that the voltage at A0 is 3.3 V.
 * 
 * For sending the NMEA data a UDP broadcast on port 10110 is used.
 * The sentecnce sent is:
 * $IIXDR,V,[0-100],P,FUEL,E,[00.0],P,FUEL,U,[00.00],V,BAT#0*[cs]
 * 
 * At each startup, WiFi Manager opens an access point (SSID "DS1603L-xxxxxx") to open the configuration 
 * page for 120 sec. The Wifi network can be configured there. Under the menu item "Setup" the setup 
 * parameters for the tank (height and size), the interval for sending out the NMEA string, the UDP port 
 * for transmitting the NMEA data and the password for the access point can be entered. 
 * The initial password is "12345678".
 * [x] A Telnet server on port 23 could be enabled via checkbox (due to security reasons).
 * [x] A test mode could be enabled via checkbox. In test mode the tank level value is swiped between 0 and
 * tank height.
 * The entered parameters will be stored in flash using the EEPROM routines.
 * 
 * The onboard LED indicates the following:
 * 1.) continious on: the access point is open
 * 2.) flashing (100ms on, every [interval_set] sec): sensor data was read
 * 3.) fast blinking (200ms interval): no connection to Wifi
 * 
 * OTA is possible with hostname DS1603L-xxxxxx, passwd = access point passwd (set in portal)
 * 
 * This project was inspired by norbert-walter https://github.com/norbert-walter/NMEA0183-Level-Sensor-DS1603L
*/

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266mDNS.h>          //Dynamik name server
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>
#include <ESPTelnet.h>            //https://github.com/LennartHennigs/ESPTelnet
#include "DS1603L.h"              //https://github.com/wvmarle/Arduino_DS1603L
#include <SoftwareSerial.h>       //https://docs.arduino.cc/learn/built-in-libraries/software-serial
#include <movingAvg.h>            //https://github.com/JChristensen/movingAvg
#include <timer.h>                //https://github.com/brunocalou/Timer
#include <timerManager.h>         //https://github.com/brunocalou/Timer
#include <EEPROM.h>
// ------------------------------------------------------------------------------------------
// WiFi Manager definitions
// ------------------------------------------------------------------------------------------
//WiFi AP settings
#define WIFI_MANAGER_TIMEOUT    120                  // Timeout of wifi Manager in sec  
#define WIFI_MANAGER_AP_PASSWD  "12345678"           // default, could be changed in portal 
// ------------------------------------------------------------------------------------------
// instancing WiFi classes
// ------------------------------------------------------------------------------------------
WiFiEventHandler gotIpEventHandler;
ESPTelnet        telnet;
WiFiManager      wifiManager; // defined global because of usage in callbacks
// ------------------------------------------------------------------------------------------
// WiFi definitions
// ------------------------------------------------------------------------------------------
#define TELNET_PORT             23
#define TELNET_ENABLED          false;  // dfault: Telnet is disabled
#define UDP_PORT                10110   // Default UDP Port for transmitting NMEA data.
                                        // Port 10110: NMEA 0183 Navigational Data.  
                                        // Transport of NMEA 0183 sentences over TCP or UDP
IPAddress ip;
char      hostname[40];
bool      telnetConnected       = false;
bool      shouldSaveParameter   = false;
// ------------------------------------------------------------------------------------------
// DS1603L Sensor definitions
// ------------------------------------------------------------------------------------------
#define FUEL_TANK_HEIGHT          400   // default fuel tank height (max. measuring distance) in mm
#define FUEL_TANK_CAPACITY        30    // default fuel tank capacity in l
#define SEND_NMEA_STRING_INTERVAL 2     // default interval time in s to send NMEA string 
#define MIN_NMEA_STRING_INTERVAL  2     // minimum interval time in s to send NMEA string 
#define NMEA_STRING_LENGTH        128
#define FILL_LEVEL_TEST_MODE      false // testmode swipes fill level from 0 to 100%
Timer sendNMEAstringTimer;

#define LED_FLASH_TIME            100
Timer LEDflashTimer;
Timer LEDblinkTimer;
#define PASSWD_STRING_LENGTH      20    

// UART pins for DS1603L connection, txPin is not connected!
const byte txPin = -1;                  // not connected!!!
const byte rxPin = D2;                  // rx of the Wemos connected to tx of the sensor
SoftwareSerial  sensorSerial(rxPin, txPin);
DS1603L         sensor(sensorSerial);

// config parameters
struct __attribute__ ((packed))configParam_st {
  int  tankhgt;
  int  tankcap;
  int  interval;
  long udpport;
  char passwd[PASSWD_STRING_LENGTH];
  bool entelnet;
  bool entestmode;
  byte chksum;
} configParam;

// Indicator LED
const byte LED = D4;
// PWM-Pin
const byte PWM_PIN = D5;
#define PWMRANGE 255

//WiFiUDP
WiFiUDP Udp;

movingAvg filter(10);                     // define the moving average object

// ==========================================================================================
// functions
// ==========================================================================================
bool LEDisOn = false;
// ------------------------------------------------------------------------------------------
void switchLEDon (void) {
  LEDisOn = true;
  digitalWrite(LED, LOW);   // LED on
}
// ------------------------------------------------------------------------------------------
void switchLEDoff (void) {
  LEDisOn = false;
  digitalWrite(LED, HIGH);   // LED off
}
// ------------------------------------------------------------------------------------------
void switchLEDtoggle (void) {
  LEDisOn = !LEDisOn;
  digitalWrite(LED, LEDisOn);   // LED off
}
// ------------------------------------------------------------------------------------------
void flashLED (void) {
    switchLEDon();
    LEDflashTimer.start();
}
// ------------------------------------------------------------------------------------------
byte checksum(char *data, size_t len) {
// ------------------------------------------------------------------------------------------
    byte crc = 0;
    unsigned int i;
    
    for (i = 0; i < len; i ++) {
        crc ^= data[i];
    }
    return crc;
}
// ------------------------------------------------------------------------------------------
byte EEPROMconfigChksum() {
// ------------------------------------------------------------------------------------------
  return checksum((char *)&configParam, sizeof(configParam)- sizeof(configParam.chksum));
}
// ------------------------------------------------------------------------------------------
void EEPROMsaveConfig() {
// ------------------------------------------------------------------------------------------
  configParam.chksum = EEPROMconfigChksum();
  EEPROM.put(0, configParam);
  EEPROM.commit();
}
// ------------------------------------------------------------------------------------------
bool EEPROMreadConfig() {
// ------------------------------------------------------------------------------------------
  configParam = {};
  EEPROM.get(0, configParam);
  if (configParam.chksum != EEPROMconfigChksum())
    return false;
  return true;
}
// ------------------------------------------------------------------------------------------
void setConfigParamDefauls() {
// ------------------------------------------------------------------------------------------
  configParam.tankhgt     = FUEL_TANK_HEIGHT;
  configParam.tankcap     = FUEL_TANK_CAPACITY;
  configParam.interval    = SEND_NMEA_STRING_INTERVAL;
  configParam.udpport     = UDP_PORT;
  configParam.entelnet    = TELNET_ENABLED;
  configParam.entestmode  = FILL_LEVEL_TEST_MODE;
  strcpy(configParam.passwd,WIFI_MANAGER_AP_PASSWD);
  EEPROMsaveConfig();
}
// ------------------------------------------------------------------------------------------
bool wifiConnected() {
// ------------------------------------------------------------------------------------------
  return (WiFi.status() == WL_CONNECTED);
}
// ------------------------------------------------------------------------------------------
void wifiInfo(){
// ------------------------------------------------------------------------------------------
  // can contain gargbage on esp32 if wifi is not ready yet
  Serial.println("[WIFI] Saved    " + (String)(wifiManager.getWiFiIsSaved() ? "YES" : "NO"));
  Serial.println("[WIFI] SSID     " + (String)wifiManager.getWiFiSSID());
  //Serial.println("[WIFI] Password " + (String)wifiManager.getWiFiPass());
  Serial.println("[WIFI] Hostname " + (String)WiFi.getHostname());
  Serial.println("[WIFI] Channel  " + (String)WiFi.channel());
  Serial.println("[WIFI] RSSI     " + (String)WiFi.RSSI());
  Serial.println("[WIFI] MAC-Adr. " + (String)WiFi.macAddress());
  Serial.println("[WIFI] IP-Adr.  " + WiFi.localIP().toString());
}
// ------------------------------------------------------------------------------------------
// ESP 8266 Wifi Event Handler
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
void onWiFigotIPAdress(const WiFiEventStationModeGotIP& evt){
// ------------------------------------------------------------------------------------------
  Serial.println("[WIFI] WIFI is connected!");
  wifiInfo();
}
// ------------------------------------------------------------------------------------------
// WifiManager callback
void saveParamCallback() {
// ------------------------------------------------------------------------------------------
  Serial.println("[CB] saveParamCallback fired");
  shouldSaveParameter = true;
  wifiManager.stopConfigPortal();
}
// ------------------------------------------------------------------------------------------
void setupSerial(long speed, String msg = "") {
// ------------------------------------------------------------------------------------------
  Serial.begin(speed);
  while (!Serial) {}
  delay(200);  
  Serial.println("\n" + (String)SKETCH_NAME + "\n" + (String)SKETCH_VERSION + "\n" + (String)SKETCH_ABOUT);
  if (msg != "") 
    Serial.println(msg);
  if (configParam.entestmode)
    Serial.println("!!! TESTMODE ACTIVE !!!");
}

// ------------------------------------------------------------------------------------------
// (optional) callback functions for telnet events
// ------------------------------------------------------------------------------------------
void onTelnetConnect(String ip) {
// ------------------------------------------------------------------------------------------
  char cstr[20];
  Serial.println("[Telnet] " + ip + " connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("commands: [bye], [restart]");
  telnet.println(SKETCH_NAME);
  telnet.println(SKETCH_VERSION);
  telnet.println(SKETCH_ABOUT);
  if (configParam.entestmode)
    telnet.println("!!! TESTMODE ACTIVE !!!");
  telnet.print("WiFi RSSI "); telnet.println((String)WiFi.RSSI());
  telnet.println("Configured values:");
  telnet.print("tank height:   "); telnet.println(itoa(configParam.tankhgt, cstr, 10));
  telnet.print("tank capacity: "); telnet.println(itoa(configParam.tankcap, cstr, 10));
  telnet.print("send interval: "); telnet.println(itoa(configParam.interval, cstr, 10));
  telnet.print("UDP port:      "); telnet.println(ltoa(configParam.udpport, cstr, 10));
  telnetConnected = true;
}
// ------------------------------------------------------------------------------------------
void onTelnetDisconnect(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.println("[Telnet] " + ip + " disconnected");
  telnetConnected = false;
}
// ------------------------------------------------------------------------------------------
void onTelnetReconnect(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.println("[Telnet] " + ip + " reconnected");
  telnetConnected = true;
}
// ------------------------------------------------------------------------------------------
void onTelnetConnectionAttempt(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.println("[Telnet] " + ip + " tried to connect");
}
// ------------------------------------------------------------------------------------------
void setupTelnet() {  
// ------------------------------------------------------------------------------------------
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  // passing a lambda function
  telnet.onInputReceived([](String str) {
    // checks for a certain command
    if (str == "restart") {
      telnet.println("> restarting");
      Serial.println("[Telnet] restart requested");
      ESP.restart();
      delay(5000);
    // disconnect the client
    } else if (str == "bye") {
      telnet.println("> disconnecting you...");
      telnet.disconnectClient();
      }
  });
  Serial.print("[Telnet] ");
  if (telnet.begin(TELNET_PORT)) {
    Serial.print("running: " + ip.toString() + " [");
    Serial.print(TELNET_PORT);
    Serial.println("]");
  } else {
    Serial.println("unable to start");
  }
}
// ------------------------------------------------------------------------------------------
// debugOutOnTelnet (height, filledPerc, filledLiter, batVoltage, analogIn, NMEAString);
void debugOutOnTelnet (int height, int avgHeight, int filledPerc, double filledLiter, double batVoltage, int analogIn, char* outputStr) {
// ------------------------------------------------------------------------------------------
  if (telnetConnected){
    char cstr[20];
    telnet.print("height: ");
    telnet.print(itoa(height, cstr, 10));
    telnet.print(" mm, avgHeight: ");
    telnet.print(itoa(avgHeight, cstr, 10));
    telnet.print(" mm, analogIn: ");
    telnet.println(itoa(analogIn, cstr, 10));
    telnet.print("filledPerc: ");
    telnet.print(itoa(filledPerc, cstr, 10));
    telnet.print(" %, filledLiter: ");
    telnet.print(dtostrf(filledLiter, 0, 1, cstr));
    telnet.print(" l, batVoltage: ");
    telnet.println(dtostrf(batVoltage, 0, 1, cstr));
    telnet.print("UDP=> ");
    telnet.println(outputStr);
  }
}
// ------------------------------------------------------------------------------------------
void setupWifiManager (void) {
// ------------------------------------------------------------------------------------------
  //reset settings - for testing
  //wifiManager.resetSettings();
  //wifiManager.erase();  

  wifiManager.setWiFiAutoReconnect(true);                     // enable WiFi auto reconnecting (important!!!)
  wifiManager.setShowPassword(false);                         // show password publicly in form
  wifiManager.setDebugOutput(true);
  //wifiManager.debugPlatformInfo();
  
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setConfigPortalBlocking(true);               
  wifiManager.setConfigPortalTimeout(WIFI_MANAGER_TIMEOUT);   // set configportal timeout
  wifiManager.setCleanConnect(true); // disconnect before connect, clean connect
  
  // callbacks
  wifiManager.setSaveParamsCallback(saveParamCallback);       

  wifiManager.setParamsPage(true);                            // move params to seperate page
  
  wifiManager.setHostname(hostname);
}
// ------------------------------------------------------------------------------------------
//Subroutine um Sensorwerte zu bekommen. Sensorwerte werden in ein Schieberegister geschrieben um aus 10 Werten den gleitenden Durchschnitt zu bekommen
int ReadDS1603L () {
// ------------------------------------------------------------------------------------------
  int  sensorValue  = sensor.readSensor();          // Call this as often or as little as you want - the sensor transmits every 1-2 seconds.
  byte sensorStatus = sensor.getStatus();           // Check the status of the sensor (not detected; checksum failed; reading success).

  switch (sensorStatus) {                           // For possible values see DS1603L.h
    case DS1603L_NO_SENSOR_DETECTED:                // No sensor detected: no valid transmission received for >10 seconds.
      Serial.println(F("[DS1603L] No sensor detected"));
      telnet.println(F("[DS1603L] No sensor detected"));
      sensorValue = -1;
      break;

    case DS1603L_READING_CHECKSUM_FAIL:             // Checksum of the latest transmission failed.
      Serial.print(F("[DS1603L] Data received; checksum failed. Latest level reading: "));
      Serial.print(sensorValue);
      Serial.println(F(" mm"));
      telnet.println(F("[DS1603L] Data received; checksum failed"));
      //sensorValue = -1;
      break;

    case DS1603L_READING_SUCCESS:                   // Latest reading was valid and received successfully.
     break;
  }
  return sensorValue;
}
// ------------------------------------------------------------------------------------------
// XDR Transducer Values 
//        1 2   3 4     x      n
//        | |   | |     |      | 
// $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF> 
// Field Number:
// 1) Transducer Type
// 2) Measurement Data
// 3) Units of measurement
// 4) Name of transducer
// x) More of the same
// n) Checksum
// ------------------------------------------------------------------------------------------
void NMEA_XDR_start(char *str) {
  strcpy(str,"$IIXDR");
}
// ------------------------------------------------------------------------------------------
void NMEA_XDR_add(char *str, char type, char *data, char unit, char const *name) {
  int len = strlen(str);
  sprintf(&str[len], ",%c,%s,%c,%s", type, data, unit, name );
}
// ------------------------------------------------------------------------------------------
void NMEA_XDR_finish(char *str) {
  int len = strlen(str);
  sprintf(&str[len], "*%X\r\n", checksum(&str[1],len-1)); // chksum w/o '$' and '*'
}
// ------------------------------------------------------------------------------------------
void sendNMEA_XDR_sentence (void) {
// ------------------------------------------------------------------------------------------
  int height      = ReadDS1603L(); 
  if (height < 0)        
    return;                                      // e.g. no sensor connected

  if (configParam.entestmode) {
    static int swipe = 0;
    height = swipe++;                              // only for Testing!!!
    if (swipe > configParam.tankhgt) swipe = 0;    // only for Testing!!!
  }

  unsigned int avgHeight  = filter.reading(height);
  int          filledPerc = int((avgHeight * 100) / configParam.tankhgt); // tanklevel in %
  double       filledLiter= (configParam.tankcap * filledPerc) / 100.0;
  // read analog input
  // voltage devider 5k6 and 1k used => Uin = 20.5 V Ua = 3,3V
  int         analogIn   = analogRead(A0);
  double      batVoltage = (((long)analogIn * 205) >> 10)/10.0; // Vmax=20.5 V, voltage=a*20.5/1024
  
  char        NMEAString[NMEA_STRING_LENGTH];
  char        cstr[10];

  NMEA_XDR_start(NMEAString);                               
  NMEA_XDR_add  (NMEAString, 'V',                           // Type  = volume
                             dtostrf(filledLiter, 0, 1, cstr),    // Measurement Data
                             'P',                           // Units = Percent
                             "FUEL#0");                     // Name  = FUEL#0
  NMEA_XDR_add  (NMEAString, 'E',                           // Type  = fluid level
                             itoa(filledPerc, cstr, 10),       // Measurement Data
                             'P',                           // Units = Percent
                             "FUEL#0");                     // Name  = FUEL#0
  NMEA_XDR_add  (NMEAString, 'U',                           // Type  = voltage
                             dtostrf(batVoltage, 0, 1, cstr),  // Measurement Data
                             'V',                           // Unit  = Volts
                             "BAT#0");                      // Name  = BAT#0
  NMEA_XDR_finish(NMEAString);

  //Setze Broadcastadresse
  IPAddress broadCastAdr = WiFi.localIP();
  broadCastAdr[3] = 255;

  if (wifiConnected()) {                     // WiFi connection established?
    Udp.beginPacket(broadCastAdr, UDP_PORT); // send UDP to Port X and BroadcastIP
    Udp.write(NMEAString);
    Udp.endPacket();
  }

  debugOutOnTelnet (height, avgHeight, filledPerc, filledLiter, batVoltage, analogIn, NMEAString);

  if (filledPerc > 100) filledPerc = 100;
  int pwmValue = (filledPerc * PWMRANGE) / 100;
  analogWrite(PWM_PIN, pwmValue);

  flashLED();
}
//###########################################################################################
// ==========================================================================================
// setup function
// ==========================================================================================
void setup() {
// ------------------------------------------------------------------------------------------
  pinMode(    LED,      OUTPUT);      // Define LED output
  switchLEDoff();
  pinMode(    PWM_PIN,  OUTPUT);      // Define PWM output
  analogWriteRange(PWMRANGE);
  analogWrite(PWM_PIN, 128);          // during onboarding set PWM to 50% for calibration

  //analogWriteFreq(new_frequency);   // should be set to 1 kHz as default
  
  setupSerial(115200, "");

  sensorSerial.begin(9600);                         // Sensor transmits its data at 9600 bps.
  sensor.begin();                                   // Initialise the sensor library.

  // Average filter
  filter.begin();

  EEPROM.begin(sizeof(configParam));
  if (!EEPROMreadConfig()) {
    Serial.println ("[EEPROM] wrong checksum, setting default values");
    setConfigParamDefauls();
  }

  WiFi.mode(WIFI_AP_STA);

  // set hostname to ESP8266-XXXXXX  
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(hostname, "DS1603L-%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFi.setHostname(hostname);

  gotIpEventHandler     = WiFi.onStationModeGotIP         (onWiFigotIPAdress);

  // setup some WifiManager parameters
  // !!! as default WIFI_MANAGER_MAX_PARAMS is set to 10 in WifiManager.h !!!
  // id/name, placeholder/prompt, default, length
  char buf[10];
  const char *customHtml;
  const char *customHtml_2;
  WiFiManagerParameter custom_tankhgt ("tankhgt",  "Tank Height [mm] (max. fill level)",itoa(configParam.tankhgt, buf, 10), 4);
  WiFiManagerParameter custom_tankcap ("tankcap",  "Tank Capacity [l]",                 itoa(configParam.tankcap, buf, 10), 4);
  WiFiManagerParameter custom_interval("interval", "Send Interval [s] (UDP interval)",  itoa(configParam.interval, buf, 10), 4);
  WiFiManagerParameter custom_udpport ("udpport",  "UDP port for NMEA transmission",    ltoa(configParam.udpport, buf, 10), 5);
  WiFiManagerParameter custom_passwd  ("passwd",   "Access Point Password (remember!)",configParam.passwd, PASSWD_STRING_LENGTH-1);
  customHtml = configParam.entelnet ? "type=\"checkbox\" checked" : "type=\"checkbox\"";
  WiFiManagerParameter custom_checkbox("entelnet", "enable Telnet on port 23", "T", 2, customHtml, WFM_LABEL_AFTER); 
  customHtml_2 = configParam.entestmode ? "type=\"checkbox\" checked" : "type=\"checkbox\"";
  WiFiManagerParameter custom_checkbox_2("entestmode", "enable test mode", "T", 2, customHtml_2, WFM_LABEL_AFTER); 
                                                                               // The "T" isn't really important, but if the
                                                                               // box is checked the value for this field will
                                                                               // be "T", so we can check for that.
  setupWifiManager();

    // add. parameters
  wifiManager.addParameter(&custom_tankhgt);                  
  wifiManager.addParameter(&custom_tankcap);                  
  wifiManager.addParameter(&custom_interval);                 
  wifiManager.addParameter(&custom_udpport);                 
  wifiManager.addParameter(&custom_passwd);                 
  wifiManager.addParameter(&custom_checkbox);                 
  wifiManager.addParameter(&custom_checkbox_2);                 
  
  switchLEDon();    // indicates open access point
  // start configPortal every time after startup
  if(!wifiManager.startConfigPortal (hostname,configParam.passwd)) {
    switchLEDoff();
    // wait Max. 10 s for connection established
    if (WiFi.waitForConnectResult(10000) != WL_CONNECTED)
      Serial.println("Not connected to WiFi but continuing anyway.");
  } else {
     Serial.println("Wifi connected!)");
     WiFi.mode(WIFI_STA);
  }  
  switchLEDoff();
    
  // read the custom parameters 
  configParam.tankhgt = atoi(custom_tankhgt.getValue());
  configParam.tankcap = atoi(custom_tankcap.getValue());
  configParam.interval = (atoi(custom_interval.getValue()) < MIN_NMEA_STRING_INTERVAL) ? MIN_NMEA_STRING_INTERVAL : atoi(custom_interval.getValue());
  configParam.udpport = atol(custom_udpport.getValue());
  configParam.entelnet = (strncmp(custom_checkbox.getValue(), "T", 1) == 0);
  configParam.entestmode = (strncmp(custom_checkbox_2.getValue(), "T", 1) == 0);
  strcpy(configParam.passwd,custom_passwd.getValue());

  if (shouldSaveParameter) {
    EEPROMsaveConfig();
    Serial.println("Parameter saved");
  } else
    EEPROMreadConfig();

  if (wifiConnected()) {
    ip = WiFi.localIP();
    if (configParam.entelnet) 
      setupTelnet();
  } else 
    Serial.println("Error connecting to WiFi");

  if (!MDNS.begin(hostname))              // Start the mDNS responder for [hostname].local
    Serial.println("Error setting up MDNS responder!");
  else
    Serial.println("mDNS responder started");

  // timer setup
  sendNMEAstringTimer.setInterval(1000*configParam.interval); // interval is in s
  sendNMEAstringTimer.setCallback(sendNMEA_XDR_sentence);
  sendNMEAstringTimer.start();
  LEDflashTimer.setTimeout(LED_FLASH_TIME);
  LEDflashTimer.setCallback(switchLEDoff);
  LEDblinkTimer.setInterval(LED_FLASH_TIME);
  LEDblinkTimer.setCallback(switchLEDtoggle);
  
  // OTA setup
  // ------------------------------------------------------------------------------------------
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
    sendNMEAstringTimer.stop();
    sensorSerial.end();                               // ESP8266 OTA crashes if sensor connected!?
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
    sensorSerial.begin(9600);                         // Sensor transmits its data at 9600 bps.
    sensor.begin();                                   // Initialise the sensor library.
    sendNMEAstringTimer.start();
  });
  /*
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  */
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.setHostname(WiFi.getHostname());
  ArduinoOTA.setPassword(configParam.passwd);
  ArduinoOTA.begin();
  // ------------------------------------------------------------------------------------------
  switchLEDoff();
}
//*******************************************************************************************
// ==========================================================================================
// loop function
// ==========================================================================================
void loop() {
  static unsigned long lastTime = millis();

  ArduinoOTA.handle();
  TimerManager::instance().update();   //Update all the timers at once
  if (configParam.entelnet)
    telnet.loop();

  if (wifiConnected()) {
    LEDblinkTimer.stop();
    lastTime = millis();
  } else {
    if ((millis() - lastTime) > 5000) {
      LEDblinkTimer.start();
      lastTime = millis();
    }
  }
}