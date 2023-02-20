/*
  Es werden Sensorwerte vom Ultraschallsensor DS1603L erfasst und mittels WiFi als NMEA Stream per UDP in das Netzwerk gesendet.
  Die Übertragung erfolgt auf die örtliche Broadcastadresse xxx.xxx.xxx.255 an den Port 50000.

  Nach dem erstmaligen Start oder bei Start in einem Netzwerk welches noch nicht bekannt ist erfolgt der
  Start mit einer Konfigurationsseite und Timeout
  Nach Ablauf von Timeout wird das WiFi-Modem für 3 Minuten abgeschaltet und dann erfolgt reset Wemos und Neustart mit Konfigurationsseite

  Geht die WiFi-Verbindung verloren wird die sichere Anmeldeseite aufgerufen, nach Timeout erfolgt reset Wemos und die normale Anmeldeschleife wie zuvor beschrieben startet.

  Automatische Wiederverbindung bei Wiederkehr WiFi ohne Neuverbindung

  Die Übertragung der Daten erfolgt über NMEA per UDP an BroadcastIP port 50000.
  Es werden 4 Pakete gesendet damit die auch wirklich nach dem aufwachen ankommen.

  Sensoreinbindung
  Die Sensorwerte werden in der Unterroutine alle 5 Sekunden auslesen.
  Die Sensorwerte werden in ein Schieberegister zum gleitenden Durchschnitt gegeben, jeweils 10 Werte bilden Durchschnitt, Register als FIFO (first In / first out)
  Es wird ein einfacher gleitender Durchschnitt gebildet.

  Der Datenstrom kann in OpenCPN eingelesen werden.
  Dazu unter Verbindungen eine neue Netzwerkverbindung einrichten, die Adresse ist die Broadcastadresse des Netzwerks (letzten drei Stellen .255) und der Port ist 50000
  Es wird eine XDR-Sequenz ausgegeben, die dann mit dem Enginedashboard-Plugin in Opencpn ausgelesen werden kann. Verschiedene Tanktypen können durch Anpassen von "FUEL" in der Unterroutine erfasst werden.
  Dazu bitte die Dokumentation vom EngineDashboard-Plugin lesen.


  adapted from Ethernet library examples
*/
#define SKETCH_NAME    __FILE__
#define SKETCH_VERSION __DATE__ " " __TIME__
#define SKETCH_ABOUT   "ESP8266 DS1603L Sensor with WiFi Manager, Telnet and NMEA broadcast sender"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>
#include <ESPTelnet.h>
#include "DS1603L.h"              //https://github.com/wvmarle/Arduino_DS1603L
#include <SoftwareSerial.h>
#include <movingAvg.h>   
#include <timer.h>
#include <timerManager.h>
// ------------------------------------------------------------------------------------------
// WiFi Manager definitions
// ------------------------------------------------------------------------------------------
//WiFi AP settings
#define WIFI_MANAGER_TIMEOUT    600                   // Timeout of wifi Manager in sec  
//#define WIFI_MANAGER_AP_NAME    "BasselESP32AP" => hostname is used!!!
#define WIFI_MANAGER_AP_PASSWD  "keins001"

//Einstellungen für broadcasting
unsigned int portBroadcast = 8888;      // localer port an den gesendet wird
unsigned int broadCast = 0;

WiFiManagerParameter custom_hostname("hostname", "Hostname", "", 40);
IPAddress ip;
char hostname[20];
// ------------------------------------------------------------------------------------------
// instancing WiFi classes
// ------------------------------------------------------------------------------------------
WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;
WiFiEventHandler probeRequestPrintHandler;
WiFiEventHandler probeRequestBlinkHandler;
ESPTelnet        telnet;
// ------------------------------------------------------------------------------------------
// WiFi definitions
// ------------------------------------------------------------------------------------------
const int        telnetPort      = 23;
// ------------------------------------------------------------------------------------------
// DS1603L Sensor definitions
// ------------------------------------------------------------------------------------------
//**************************
#define FUEL_TANK_HEIGHT 400            // Height of fuel tank (max. measuring distance) in mm
//**************************
#define SEND_NMEA_STRING_INTERVAL 2000  // interval time in ms to send NMEA string 
Timer sendNMEAstringTimer;
#define LED_FLASH_TIME            100
Timer LEDflashTimer;
#define NMEA_STRING_LENGTH        128

// UART pins for DS1603L connection, txPin is not connected!
const byte txPin = D2; // not connected!!!
const byte rxPin = D3;                               // rx of the Wemos to tx of the sensor
SoftwareSerial sensorSerial(rxPin, txPin);

// If your sensor is connected to Serial, Serial1, Serial2, AltSoftSerial, etc. pass that object to the sensor constructor.
DS1603L sensor(sensorSerial);
// ------------------------------------------------------------------------------------------

// Indicator LED
const byte LED = D4;
// PWM-Pin
const byte PWM_PIN = D5;
#define PWMRANGE 255

int swipe = 0;

//WiFiUDP
WiFiUDP Udp;

//WiFiManager
WiFiManager wifiManager;

// Definition eines Arrays von 10 Feldern für gleitenden Durchschnitt
movingAvg filter(10);

// ==========================================================================================
// functions
// ==========================================================================================
// ------------------------------------------------------------------------------------------
String macToString(const unsigned char* mac) {
// ------------------------------------------------------------------------------------------
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}
// ------------------------------------------------------------------------------------------
bool wifiConnected() {
// ------------------------------------------------------------------------------------------
  return (WiFi.status() == WL_CONNECTED);
}
// ------------------------------------------------------------------------------------------
void wifiInfo(){
  // can contain gargbage on esp32 if wifi is not ready yet
  Serial.println("[WIFI] WIFI INFO DEBUG");
  //WiFi.printDiag(Serial);
  Serial.println("[WIFI] Saved    " + (String)(wifiManager.getWiFiIsSaved() ? "YES" : "NO"));
  Serial.println("[WIFI] SSID     " + (String)wifiManager.getWiFiSSID());
  Serial.println("[WIFI] Password " + (String)wifiManager.getWiFiPass());
  Serial.println("[WIFI] Hostname " + (String)WiFi.getHostname());
  Serial.println("[WIFI] Channel  " + (String)WiFi.channel());
  Serial.println("[WIFI] RSSI     " + (String)WiFi.RSSI());
  Serial.println("[WIFI] MAC-Adr. " + (String)WiFi.macAddress());
  Serial.print("[WIFI] IP-Adr.  "); Serial.println(WiFi.localIP());
}
// ------------------------------------------------------------------------------------------
void onStationConnected(const WiFiEventSoftAPModeStationConnected& evt) {
// ------------------------------------------------------------------------------------------
  Serial.print("Station connected: ");
  Serial.println(macToString(evt.mac));
}
// ------------------------------------------------------------------------------------------
void onStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
// ------------------------------------------------------------------------------------------
  Serial.print("Station disconnected: ");
  Serial.println(macToString(evt.mac));
}
// ------------------------------------------------------------------------------------------
// WifiManager callback
void saveParamCallback(){
// ------------------------------------------------------------------------------------------
  Serial.println("[CB] saveParamCallback fired");
  Serial.print("[CB] custom_hostname: "); Serial.println(custom_hostname.getValue());
  if (strlen(custom_hostname.getValue()) != 0) {
    Serial.print("[CB] Change hostname from: "); Serial.print(WiFi.getHostname()); 
    Serial.print(" to: "); Serial.println(custom_hostname.getValue());
    WiFi.setHostname(custom_hostname.getValue());
  }
}
// ------------------------------------------------------------------------------------------
void setupSerial(long speed, String msg = "") {
// ------------------------------------------------------------------------------------------
  Serial.begin(speed);
  while (!Serial) {
  }
  delay(200);  
  Serial.println();
  Serial.println();
  if (msg != "") Serial.println(msg);
}

// ------------------------------------------------------------------------------------------
// (optional) callback functions for telnet events
// ------------------------------------------------------------------------------------------
void onTelnetConnect(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.print("[Telnet] ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");
}
// ------------------------------------------------------------------------------------------
void onTelnetDisconnect(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.print("[Telnet] ");
  Serial.print(ip);
  Serial.println(" disconnected");
}
// ------------------------------------------------------------------------------------------
void onTelnetReconnect(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.print("[Telnet] ");
  Serial.print(ip);
  Serial.println(" reconnected");
}
// ------------------------------------------------------------------------------------------
void onTelnetConnectionAttempt(String ip) {
// ------------------------------------------------------------------------------------------
  Serial.print("[Telnet] ");
  Serial.print(ip);
  Serial.println(" tried to connected");
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
    if (str == "ping") {
      telnet.println("> pong");
      Serial.println("[Telnet] pong");
    // disconnect the client
    } else if (str == "bye") {
      telnet.println("> disconnecting you...");
      telnet.disconnectClient();
      }
  });

  Serial.print("[Telnet] ");
  if (telnet.begin(telnetPort)) {
    Serial.print("running: ");
    Serial.print(ip); Serial.print(" ["); Serial.print(telnetPort); Serial.println("]");
  } else {
    Serial.println("unable to start");
  }
}
// ------------------------------------------------------------------------------------------
void flashLED (void) {
  digitalWrite(LED, LOW);   // LED on
  LEDflashTimer.start();
}
// ------------------------------------------------------------------------------------------
void switchLEDoff (void) {
  digitalWrite(LED, HIGH);   // LED off
}
// ------------------------------------------------------------------------------------------
//Subroutine um Sensorwerte zu bekommen. Sensorwerte werden in ein Schieberegister geschrieben um aus 10 Werten den gleitenden Durchschnitt zu bekommen
unsigned int ReadDS1603L () {
// ------------------------------------------------------------------------------------------
  unsigned int  sensorValue  = sensor.readSensor(); // Call this as often or as little as you want - the sensor transmits every 1-2 seconds.
  byte          sensorStatus = sensor.getStatus();  // Check the status of the sensor (not detected; checksum failed; reading success).

  switch (sensorStatus) {                           // For possible values see DS1603L.h
    case DS1603L_NO_SENSOR_DETECTED:                // No sensor detected: no valid transmission received for >10 seconds.
      Serial.println(F("[DS1603L] No sensor detected"));
      sensorValue = 0;
      break;

    case DS1603L_READING_CHECKSUM_FAIL:             // Checksum of the latest transmission failed.
      Serial.print(F("[DS1603L] Data received; checksum failed. Latest level reading: "));
      Serial.print(sensorValue);
      Serial.println(F(" mm"));
      break;

    case DS1603L_READING_SUCCESS:                   // Latest reading was valid and received successfully.
      Serial.print(F("[DS1603L] value: "));
      Serial.print(sensorValue);
      Serial.println(F(" mm"));
      break;
  }
  return sensorValue;
}
// ------------------------------------------------------------------------------------------
//Calculates the checksum for the NMEA String
unsigned char NMEAchecksum(char *string){
// ------------------------------------------------------------------------------------------
  unsigned char value=0;
  while(*string != '*' ){
    value ^= *(string++);
  }
  return value;
}
// ------------------------------------------------------------------------------------------
//Create NMEA String XDR
char* NMEA_XDR(char* NMEAString, int percent, double liter) {
// ------------------------------------------------------------------------------------------
  char cstr[10];
  strcpy(NMEAString,"$IIXDR");
  //1st quadruple ",V,[0-100],P,FUEL"
  strcat(NMEAString,",V,");
  strcat(NMEAString, itoa(percent, cstr, 10));
  strcat(NMEAString, ",P,FUEL"); 
  //2nd quadruple ",V,[00.0],M,FUEL"
  //strcat(NMEAString,",V,");
  //strcat(NMEAString, dtostrf(liter, 4, 1, cstr)); // returns char "000.0"
  //strcat(NMEAString, ",M,FUEL"); 
  
  strcat(NMEAString, "*"); 
  //add checksum
  int currStrLen = strlen(NMEAString);
  sprintf(&NMEAString[currStrLen], "%x", NMEAchecksum(&NMEAString[1]));
  strcat(NMEAString, "\r");
  strcat(NMEAString, "\n");
  return NMEAString;
}
// ------------------------------------------------------------------------------------------
void sendSensorValues (void) {
// ------------------------------------------------------------------------------------------
  unsigned int hight      = ReadDS1603L(); 

  hight = swipe++;                              // only for Testing!!!
  if (swipe > FUEL_TANK_HEIGHT) swipe = 0;      // only for Testing!!!

  unsigned int avgHight   = filter.reading(hight);
  int          filledPerc = int((avgHight * 100) / FUEL_TANK_HEIGHT); // tanklevel in %
  double       filledLiter= 13.8;
  char         NMEAString[NMEA_STRING_LENGTH];

  NMEA_XDR(NMEAString, filledPerc, filledLiter);

  //Setze Broadcastadresse
  IPAddress broadCast = WiFi.localIP();
  broadCast[3] = 255;

  Serial.print("Hight[mm]: ");
  Serial.print(hight);
  Serial.print(" -> avg Hight[mm]: ");
  Serial.print(avgHight);
  Serial.print(" -> Level[%]: ");
  Serial.println(filledPerc);
  Serial.print("send to: ");
  Serial.print(broadCast);
  Serial.print(" [");
  Serial.print(portBroadcast);
  Serial.print("]: ");
  Serial.println(NMEAString);

  Udp.beginPacket(broadCast, portBroadcast); // send UDP to Port 50000 and BroadcastIP
  Udp.write(NMEAString);
  Udp.endPacket();

  if (filledPerc > 100) filledPerc = 100;
  //int pwmValue = (filledPerc * PWMRANGE) / 100;
  int pwmValue = 10;
  analogWrite(PWM_PIN, pwmValue);
  Serial.print("PWM :");
  Serial.println(pwmValue);

  flashLED();
}
//###########################################################################################
// ==========================================================================================
// setup function
// ==========================================================================================
void setup() {
// ------------------------------------------------------------------------------------------

  pinMode(      LED,      OUTPUT);    // Define LED output
  pinMode(      PWM_PIN,  OUTPUT);    // Define LED output
  digitalWrite( LED,      LOW);       // Set LED on (low activ)
  analogWriteRange(PWMRANGE);
  //analogWriteFreq(new_frequency);   // should be set to 1 kHz as default
  
  setupSerial(115200, "DS1603L started");

  sensorSerial.begin(9600);                         // Sensor transmits its data at 9600 bps.
  sensor.begin();                                   // Initialise the sensor library.

  //Timeout in sek., nach Ablauf wird die Setup-Seite ausgeschaltet
  wifiManager.setTimeout(600);

  // Average filter
  filter.begin();

  // Print sketch intro ---------------------------
  Serial.println();
  int i = strlen(SKETCH_ABOUT);
  while(i--) Serial.print("="); 
  Serial.println();
  Serial.println(SKETCH_NAME);
  Serial.println(SKETCH_VERSION);
  Serial.println(SKETCH_ABOUT);
  i = strlen(SKETCH_ABOUT);
  while(i--) Serial.print("="); 
  Serial.println();

  Serial.print("Tank height set to: ");
  Serial.print(FUEL_TANK_HEIGHT);
  Serial.println(" mm");

  WiFi.mode(WIFI_AP_STA);

  // set hostname to ESP32-XXXXXX  
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(hostname, "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFi.setHostname(hostname);

  // Register event handlers.
  // Callback functions will be called as long as these handler objects exist.
  // Call "onStationConnected" each time a station connects
  stationConnectedHandler = WiFi.onSoftAPModeStationConnected(&onStationConnected);
  // Call "onStationDisconnected" each time a station disconnects
  stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&onStationDisconnected);

  //reset settings - for testing
  //wifiManager.resetSettings();
  //wifiManager.erase();  

  wifiManager.setDebugOutput(false);
  wifiManager.debugPlatformInfo();

  // callbacks
  wifiManager.setSaveParamsCallback(saveParamCallback);

  // add all your parameters here
  wifiManager.addParameter(&custom_hostname);

  // ConfigPortal is set to blocking
  wifiManager.setConfigPortalBlocking(true);
  
  // set configportal timeout
  wifiManager.setConfigPortalTimeout(WIFI_MANAGER_TIMEOUT);

  if(!wifiManager.autoConnect(hostname,WIFI_MANAGER_AP_PASSWD)) {
      Serial.println("wifiManager failed to connect and hit timeout");
  } else {
    //if you get here you have connected to the WiFi
     Serial.println("connected...yeey :)");
  }  
  
  wifiInfo();

  if (wifiConnected()) {
    ip = WiFi.localIP();
    setupTelnet();
  } else {
    Serial.println();    
    Serial.println("Error connecting to WiFi");
  }

  // timer setup
  sendNMEAstringTimer.setInterval(SEND_NMEA_STRING_INTERVAL);
  sendNMEAstringTimer.setCallback(sendSensorValues);
  sendNMEAstringTimer.start();
  LEDflashTimer.setTimeout(LED_FLASH_TIME);
  LEDflashTimer.setCallback(switchLEDoff);

  ArduinoOTA.setHostname(WiFi.getHostname());
  ArduinoOTA.begin();
}
//*******************************************************************************************
// ==========================================================================================
// loop function
// ==========================================================================================
void loop() {
  ArduinoOTA.handle();
  telnet.loop();
  //Update all the timers at once
  TimerManager::instance().update();
}