# NMEA0183-Level-Sensor-DS1603L
NMEA0183 Level Sensor DS1603L

 DS1603L Ultrasonic Level Sensor with NMEA 0183 UDP sender, using Tzapu´s WiFiManager
  
 The Tx Pin of the DS1603L has to be connected to D2 (max 3.3V!).
  
 For using a 0-10V converter for the tank level a PWM is configured for D5 (GPIO14). In the two 
 minutes startup phase the PWM is set to half (128) for calibration of the 0-10V adapter.
  
 In addition a used 12V voltage could be measured at Pin D5 (analog input A0). A voltage devider must 
 be used for A0 of ESP8266. I calculated the resistors (R1=5k6, R2=1k) for a maximum voltage of 
 20.5 V so that the voltage at A0 is 3.3 V.
  
 For sending the NMEA data a UDP broadcast on port 10110 is used.
 The sentecnce sent is:
 $IIXDR,V,[0-100],P,FUEL,E,[00.0],P,FUEL,U,[00.00],V,BAT#0*[cs]
  
 At each startup, WiFi Manager opens an access point (SSID "DS1603L-xxxxxx") to open the configuration 
 page for 120 sec. The Wifi network can be configured there. Under the menu item "Setup" the setup 
 parameters for the tank (height and size), the interval for sending out the NMEA string, the UDP port 
 for transmitting the NMEA data and the password for the access point can be entered. 
 The initial password is "12345678".
 A Telnet server on port 23 could be enabled via checkbox (due to security reasons).
 A test mode could be enabled via checkbox. In test mode the tank level value is swiped between 0 and
 tank height.
 The entered parameters will be stored in flash using the EEPROM routines.
  
 The onboard LED indicates the following:
 1.) continious on: the access point is open
 2.) flashing (100ms on, every [interval_set] sec): sensor data was read
 3.) fast blinking (200ms interval): no connection to Wifi
  
 OTA is possible with hostname DS1603L-xxxxxx, passwd = access point passwd (set in portal)
  
 This project was inspired by norbert-walter https://github.com/norbert-walter/NMEA0183-Level-Sensor-DS1603L
 
 ![IMG_0899_small](https://user-images.githubusercontent.com/67091578/222971708-06b80d79-7483-4d83-97f7-ee9dfd0ea5f8.png)
 
 ![NMEAremote_small](https://user-images.githubusercontent.com/67091578/222971718-160819ef-eff3-4e15-9cc0-b0820134d168.png)

 
