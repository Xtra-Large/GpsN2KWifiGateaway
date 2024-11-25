/*
Copyright (c) 2021-2025 Aurelien TRICAULT - XtraLarge
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/**Ce programme utilise les librairies suivantes :
 * https://github.com/ttlappalainen/NMEA0183
 * https://github.com/ttlappalainen/NMEA2000
 * https://github.com/ttlappalainen/NMEA2000_esp32
 * https://github.com/ronzeiller/NMEA0183-AIS
 * https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
 * 
 * 
 * Il faut aussi installer manuellement la librairie BlinkControl, celle fournie par les IDE Arduino n'etant plus MAJ
 * https://github.com/Xtra-Large/BlinkControl
 * 
 */

/**
 * TODO
 *  - Ajout UDP sur IP locale?
 *  - Ajout du serveur de configuration Wifi pour configuration sur un routeur externe
 *  - TCP NMEA2K (voir si intéret)
 *  - UDP NMEA2K (voir si intéret)
 *  - Ajout de fonction AJAX/JSON pour rechargement de la page de statut
 *    -> Mise en une seule page pour intégrer la fonction de MAJ du firmware
 *    -> Ajout d'option pour activer certaines redirections vers les ports et le reseau (a stocker en NVS)
 *    -> Mise en page et logo
 *    -> Configuration des ports de sortie serie : Msg NMEA, vitesse, activation...
 *    -> Configuration du SSID externe + PWD
 *  - TCP Broadcast NMEA2K (voir si intéret)
 *  - TCP Broadcast NMEA0183
 *  
 *  20210213 
 *    - ADD HOSTNAME + services DNS
 *    - Sortie NMEA0183 sur les 3 ports serie (pour ajout éventuel d'un pilote sur ce port)
 *    - Ajout port serie 2
 *    - UDP NMEA0183 broadcast
 *    - TCP NMEA1083

 *  20241125
 *    - MAJ des librairies pour compilation
 */


#pragma once
/*
  * Module de conversion GPS Ublox (M8N) vers NMEA 2000 à 10Hz
  * Copyright Dronotique - Aurélien TRICAULT
 */

extern "C" {
#include "esp_wifi.h"
#include "driver/uart.h"
}
 
#define DEBUG_ON false

#define VERSION 0.12
#define VERSION_STR "0.12"
#include <cstdint>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BlinkControl.h>
//OTA
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <Update.h>

//Manipulation des dates/times
#include <TimeLib.h>

//Stockage de l'adresse sur le bus N2K
#include <Preferences.h>
int NodeAddress;  // To store last Node Address
Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress
bool updateEnCours = false;

#include "configuration.h"

#include <Wire.h> //Needed for I2C to GPS


//gestion des pins
#define RX2_PIN GPIO_NUM_25
#define TX2_PIN GPIO_NUM_26
//#define RX1_PIN GPIO_NUM_9 //Non utilisable
//#define TX1_PIN GPIO_NUM_10 //Non utilisable
#define RX1_PIN GPIO_NUM_18
#define TX1_PIN GPIO_NUM_19
#define RX0_PIN GPIO_NUM_3
#define TX0_PIN GPIO_NUM_1
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
//#define ESP32_CAN_TX_PIN GPIO_NUM_16
//#define ESP32_CAN_RX_PIN GPIO_NUM_4
//#define ESP32_CAN_TX_PIN GPIO_NUM_5
//#define ESP32_CAN_RX_PIN GPIO_NUM_4
#define ESP32_CAN_TX_PIN GPIO_NUM_17
#define ESP32_CAN_RX_PIN GPIO_NUM_16
#define SPI_SCK GPIO_NUM_23
#define SPI_CS GPIO_NUM_27
#define SPI_MOSI GPIO_NUM_32
#define SPI_MISO GPIO_NUM_33

//#include <HardwareSerial.h>
//HardwareSerial MySerial1(1);
#define GNSS_ON_SERIAL false
#define DEFAULT_BAUD_RATE 115200
#define BAUD_RATE_SIZE 5
int baudsRates[] = {38400, 115200, 9600, 57600, 4800};



//GPS
//#include <SparkFun_Ublox_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#define NUM_SAT_MIN 5
#define GPS_HDOP_MIN 2.5
#define defaultMaxWait 10000 //Max wait for GPS Ublox ... Augmenter car bug sinon
SFE_UBLOX_GNSS myGNSS;
#define MAX_GPS_FREQUENCY 25

//NMEA 2000
#define N2K_FRAME_BUFFER_SIZE 250
#define N2K_FRAME_RECEIVE_BUFFER_SIZE 100


//#define NMEA2000_FRAME_ERROR_DEBUG
//#define NMEA2000_FRAME_IN_DEBUG
//#define NMEA2000_FRAME_OUT_DEBUG
//#define NMEA2000_MSG_DEBUG
//#define NMEA2000_BUF_DEBUG
//#define NMEA2000_DEBUG

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

// List here messages your device will transmit.
// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {129029L,129026L,129025L,0};
const unsigned long ReceiveMessages[] PROGMEM = {/*126992L,*/ // System time
      127250L, // Heading
      127258L, // Magnetic variation
      128259UL,// Boat speed
      128267UL,// Depth
      129025UL,// Position
      129026L, // COG and SOG
      129029L, // GNSS
      129025L, // LAT + LON RAPID
      130306L, // Wind
      128275UL,// Log
      127245UL,// Rudder
      129038L, //AIS Classe A
      129039L, //AIS Classe B
      12979UL, //AIS Classe A static
      129809L, //AIS Classe B static
      129810L, //AIS Classe B static
      0
    };

#define RapidDataUpdatePeriod 50 //20Hz //In millisec - Some strange periot to cause Slow and rapid to run unsync.
#define SlowDataUpdatePeriod 500 //2Hz //In millisec - Some strange periot to cause Slow and rapid to run unsync.
#define MiscDataUpdatePeriod 5000 //0.2Hz //In millisec - Some strange periot to cause Slow and rapid to run unsync.
unsigned long RapidDataUpdated=0;
unsigned long SlowDataUpdated=0;
unsigned long MiscDataUpdated=0;

//COnversion en NMEA0183
#include "N2kDataToNMEA0183.h"
#include "List.h"
#include "BoardSerialNumber.h"
#include "BoatData.h"
#include <WiFiUdp.h>
#define SEND_SEA_SMART true
#if SEND_SEA_SMART 
  #include "Seasmart.h"
#endif

using namespace std;

//Gestion de la led
/*4 mode pour la led
 * Eteinte = Module non alimenté
 * Fixe = Initialisation en cours
 * Clignotement lent = Acquisition GPS en cours
 * Clignotement rapide = Module prêt
 */
#define PIN_LED 12 //16
BlinkControl led(PIN_LED);
int blinkState = 0; //0 = ON, 1 = 1hZ, 2=3hz

//GPS 
#define NUM_SAT_MIN 5 //Nombre de satelitte mini pour considérer la position comme valide
#define GPS_HDOP_MIN 4 //HDOP mini pour considérer la position comme valide
#define HZ_TO_READ_GPS 10 //Frequence de la lecture de la position GPS
long lastTimeReadGPS; //Dernière lectuer du GPS

bool gpsInitialized = false;

long n2kMsgId = 0;

tBoatData boatData;
bool gpsPositionUpdated = false;
bool gpsCogSogUpdated = false;

bool initiatlisationCompleted = false;

//SSID et PWD
#define HOSTNAME "GPS20HZN2K"
#define WIFI_SSID "GPS20HZN2K"
#define WIFI_PWD "xtralarge"
#define WIFI_CHANNEL 6
String mySsid;
String myPwd;

//Connexion au Wifi du bateau => TODO Passer les valeur en NVS et les rendre editable via l'interface Web
const char *wifi_network_ssid = "XXXXXXXXXXXXXXXX";
const char *wifi_network_password = "XXXXXXXXXXXXXXXXXXXXXXX";

//Server Web OTA et webApp
// Set web server port number to 80
#define WEBSERVER_PORT 80
WebServer server(WEBSERVER_PORT);

//Web Server NMEA0183 TCP et UDP
tN2kDataToNMEA0183 tN2kDataToNMEA0183(&NMEA2000, 0);
#define MAX_TCP_CLIENT_NMEA0183 10
#define WIFISERVER_NMEA0183_PORT 2947
#define UDP_NMEA0183_PORT 2948
WiFiServer serverNMEA0183(WIFISERVER_NMEA0183_PORT, MAX_TCP_CLIENT_NMEA0183); //10 clients max
using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clientsNMEA0183;
WiFiUDP udpNmea0183;

#define MAX_TCP_CLIENT_NMEA2K 10
#define WIFISERVER_NMEA2K_PORT 2948
#define UDP_NMEA2K_PORT 2948
WiFiServer serverNMEA2K(WIFISERVER_NMEA0183_PORT, MAX_TCP_CLIENT_NMEA0183); //10 clients max
using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clientsNMEAN2K;
//WiFiUDP udpNmea2K;

IPAddress broadCastIPAddress;
IPAddress broadCastIPAddressStation;

#define TIME_DEBUG_ALIVE  300000 //Au bout de 5 minutes on eteind toutes les fonctionnalités de debug (Serial1 et Wifi + OTA)
String gpsCnxMode = "AUCUN GPS";

int printLogs(const char *fmt, va_list args) {
  Serial.print(fmt);
}

void setup() {
  esp_log_set_vprintf(printLogs);
  
  Serial.begin(DEFAULT_BAUD_RATE);
  //Serial1.begin(DEFAULT_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN, true, 11000UL);  // Passing 0 for baudrate to detect it, the last parameter is a timeout in ms
  Serial1.begin(DEFAULT_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN);
  Serial2.begin(DEFAULT_BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  //Serial1.begin(DEFAULT_BAUD_RATE);
  Serial.println("////////////////////////////////////////////////////////");
  Serial.print("GPS 20 Hz NMEA 2000/0183 - Version : ");
  Serial.print(VERSION);
  uint32_t id = GetBoardSerialNumber();
  Serial.print(" SN : ");
  Serial.println(id);                               
  Serial.println("////////////////////////////////////////////////////////");
  
  led.begin();
  led.on();

  //COnfiguration du reseau et serveur WIFI
  mySsid = String(WIFI_SSID); 
  char ssid_c[mySsid.length()];
  mySsid.toCharArray(ssid_c, mySsid.length());
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(ssid_c, WIFI_PWD, WIFI_CHANNEL, false, 4);
  //Connexion au routeur Wifi
  WiFi.begin(wifi_network_ssid, wifi_network_password);

  broadCastIPAddressStation = WiFi.localIP(); // find my local IP address
  broadCastIPAddress = WiFi.softAPIP();
  broadCastIPAddressStation[3] = 255;
  broadCastIPAddress[3] = 255;         // Set last octet to 255 for Class C broadcast address of this subnet

  String ipAdrrAP = String(broadCastIPAddress[0]) + "." + String(broadCastIPAddress[1]) + "." + String(broadCastIPAddress[2]) + "." + String(broadCastIPAddress[3]);
  String ipAdrrStation = String(broadCastIPAddressStation[0]) + "." + String(broadCastIPAddressStation[1]) + "." + String(broadCastIPAddressStation[2]) + "." + String(broadCastIPAddressStation[3]);
  MDNS.begin(HOSTNAME);
  MDNS.addService("_nmea","_udp",UDP_NMEA0183_PORT);
  MDNS.addServiceTxt("_nmea","_udp","Host",HOSTNAME);
  MDNS.addServiceTxt("_nmea","_udp","IP", ipAdrrAP);
  MDNS.addService("_nmea","_tcp",UDP_NMEA0183_PORT);
  MDNS.addServiceTxt("_nmea","_tcp","Host",HOSTNAME);
  MDNS.addServiceTxt("_nmea","_tcp","IP", ipAdrrAP);
   
  configureWebServer();

#if DEBUG_ON
  Serial.print("UDP Adresse : ");
  Serial.print(broadCastIPAddress[0]);
  Serial.print(".");
  Serial.print(broadCastIPAddress[1]);
  Serial.print(".");
  Serial.print(broadCastIPAddress[2]);
  Serial.print(".");
  Serial.println(broadCastIPAddress[3]);
#endif
  vTaskDelay(1000/portTICK_PERIOD_MS);
#if DEBUG_ON
    myGNSS.enableDebugging(Serial, false);
#endif

  //Trying UART
  //Connexion au GPS par le port serie
  //Serial.begin(baudsRates[0], SERIAL_8N1, -1, -1);
#if GNSS_ON_SERIAL   
  int i=0;
  for(i; i < BAUD_RATE_SIZE; i++){
#if DEBUG_ON
    Serial.print("Trying on UART port with baudrate : ");
    Serial.println(baudsRates[i]);
#endif
    Serial1.updateBaudRate(baudsRates[i]);
    vTaskDelay(100/portTICK_PERIOD_MS);
    
    gpsInitialized = myGNSS.begin(Serial1);
    if (gpsInitialized){
      break;
    }
    //A t'on trouvé 
    if (gpsInitialized){
      gpsCnxMode = "UART";
#if DEBUG_ON
      Serial.print("GPS initialisé a une vitesse de ");
      Serial.print(baudsRates[i]);
      Serial.println(" BAUD");
#endif
      delay(100);
      if(i != 0){ //On n'est pas sur la vitesse par defaut, on change ca
#if DEBUG_ON
        Serial.print("Changement du BAUD rate pour ");
        Serial.println(baudsRates[0]);
#endif
        //myGNSS.setSerial1Rate(baudsRates[0]);
        delay(100);
      }
    }else{
      Serial.println("Aucun GPS trouvé sur le port UART");
    }
  }
#endif
  if(! gpsInitialized){ //Trying I2C
    //Serial.end();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Wire.begin(I2C_SDA, I2C_SCL);
    //Wire.begin(TX0_PIN, RX0_PIN);
    //Wire.setClock(400000);
    if(myGNSS.begin()){
#if DEBUG_ON
      Serial.println(F("UBlox GPS FOUND ON DEFAUT I2C"));  
#endif
      gpsCnxMode = "I2C";
      gpsInitialized = true;
    }else{
#if DEBUG_ON
      Serial.println(F("Ublox GPS not detected at default I2C address. CHeck other addresses."));
#endif
      preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
      uint8_t gpsI2CAddress = preferences.getInt("Gps_I2C_Address", 0x42);  // I2C Address
      if (myGNSS.begin(Wire, gpsI2CAddress) == false){ //Connect to the Ublox module using Wire port
#if DEBUG_ON        
        Serial.println(F("Ublox GPS not detected at saved I2C address. CHeck other addresses."));
#endif
        uint8_t  loopCounter = 0;
        for(loopCounter=0; loopCounter<=127; loopCounter++){
#if DEBUG_ON
          Serial.print("Trying with I2C Address : ");
          Serial.println(loopCounter);
#endif
          
          if(myGNSS.begin(Wire, (uint8_t)loopCounter)){
#if DEBUG_ON
            Serial.print("GPS found on I2C Address : ");
            Serial.println(loopCounter);
#endif
            gpsCnxMode = "I2C";
            preferences.putInt("Gps_I2C_Address", loopCounter);
            gpsInitialized = true;
            break;
          }
        }
      }else{
        gpsInitialized = true;
      }
      preferences.end();
    }
  }
  
  if(gpsInitialized){
    myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    myGNSS.setUART2Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    //On réduit les infos qui tansitent sur le port série
    myGNSS.setDynamicModel(DYN_MODEL_SEA); //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
    //Passage a la fréquence maximale
    myGNSS.setNavigationFrequency(MAX_GPS_FREQUENCY);
    myGNSS.setAutoPVT(true);
    //myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
#if DEBUG_ON
    Serial.print("GPS Version ");
    Serial.print(myGNSS.getProtocolVersionHigh());
    Serial.print(".");
    Serial.println(myGNSS.getProtocolVersionLow());
#endif
  }else{
    Serial.println("GPS NON INITIALISE");
  }

//#######################################################################
// Initialisatioin NMEA2000
//#######################################################################

  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANSendFrameBufSize(N2K_FRAME_BUFFER_SIZE);
  NMEA2000.SetN2kCANReceiveFrameBufSize(N2K_FRAME_RECEIVE_BUFFER_SIZE);
  // Set Product information
  NMEA2000.SetProductInformation("1613", // Manufacturer's Model Serial1 code
                                 101, // Manufacturer's product code
                                 "N9M GNSS 25Hz",  // Manufacturer's Model ID
                                 VERSION_STR,  // Manufacturer's Software version code
                                 VERSION_STR // Manufacturer's Model version
                                 );
                                 
  // Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial1 number.
                                145, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                60, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                666 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 32);  // Read stored last NodeAddress, default 32
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.AttachMsgHandler(&tN2kDataToNMEA0183); // NMEA 2000 -> NMEA 0183 conversion
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg); // NMEA 2000 to TCP/UDP/Actisense
  
  tN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);
  
#if DEBUG_ON
  NMEA2000.SetForwardStream(&Serial);  // PC output on due programming port
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  // NMEA2000.SetForwardOwnMessages();
  //NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  //NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial1)
#else
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial1)
#endif
  NMEA2000.Open();

  initiatlisationCompleted = true;

  RapidDataUpdated=InitNextUpdate(RapidDataUpdatePeriod);
  SlowDataUpdated=InitNextUpdate(SlowDataUpdatePeriod);
  MiscDataUpdated=InitNextUpdate(MiscDataUpdatePeriod);


  //Start server NMEA0183
  serverNMEA0183.begin();
  udpNmea0183.begin(UDP_NMEA0183_PORT);
  //udpNmea2K.begin(UDP_NMEA2K_PORT);

  TaskHandle_t TaskLoopSendN2K; 
  xTaskCreatePinnedToCore(
             LoopSendN2K,  /* Task function. */
             "LoopSendN2K",    /* name of task. */
             10000,      /* Stack size of task */
             NULL,       /* parameter of the task */
             1,          /* priority of the task */ //Cette tache est la plus prioritaire!
             &TaskLoopSendN2K,     /* Task handle to keep track of created task */
             0);         /* pin task to core 0 */

  //TaskHandle_t TaskLoopManageWifiCnx; 
  //xTaskCreatePinnedToCore(
  //           LoopManageWifiCnx,  /* Task function. */
  //           "LoopManageWifiCnx",    /* name of task. */
  //           10000,      /* Stack size of task */
  //           NULL,       /* parameter of the task */
  //           1,          /* priority of the task */
  //           &TaskLoopSendN2K,     /* Task handle to keep track of created task */
  //           0);         /* pin task to core 0 */
  
}



void LoopSendN2K( void * pvParameters ){
  long latitude;
  long longitude;
  long cog;
  long sog;
  for(;;){
    yield();
    if(updateEnCours){
      vTaskDelay(10000/portTICK_PERIOD_MS);
    }
    /*On lit le GPS a la vitesse maximale, c'est l'envoie en NMEA2K qui se fera selon une certaine fréquence*/   
    if(gpsInitialized){
      //Traitement des réceptions GPS
      latitude = myGNSS.getLatitude();
      if(latitude != boatData.Latitude){
        boatData.Latitude = latitude;
        gpsPositionUpdated = true;
      }
      longitude = myGNSS.getLongitude();
      if (longitude != boatData.Longitude){
        boatData.Longitude = longitude;  
        gpsPositionUpdated = true;
      }
  
      cog = myGNSS.getHeading();
      if(cog != boatData.COG){
        boatData.COG = cog;
        gpsCogSogUpdated = true;
      }
  
      sog = myGNSS.getGroundSpeed();
      if(sog != boatData.SOG){
        boatData.SOG = sog;
        gpsCogSogUpdated = true;
      }
      //Envoie de la position NMEA2K
      tN2kMsg N2kMsg;
      bool sendOK = true;
      if ( IsTimeToUpdate(SlowDataUpdated) ) {
#if DEBUG_ON
        Serial.println("Sending low rate N2K");
#endif
        //On envoie des données plus complètes mais moin souvent
        SetNextUpdate(SlowDataUpdated,SlowDataUpdatePeriod);
  
        setTime(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear());
        time_t curDate = now();
        //*****************************************************************************
        // Date,Time & Local offset  ( see also PGN 126992 )
        // Input:
        //  - DaysSince1970         Days since 1.1.1970. You can find sample how to convert e.g. NMEA0183 date info to this on my NMEA0183 library on
        //                          NMEA0183Messages.cpp on function NMEA0183ParseRMC_nc
        //  - Time                  Seconds since midnight
        //  - Local offset          Local offset in minutes
        //inline void SetN2kLocalOffset(tN2kMsg &N2kMsg, uint16_t DaysSince1970, double SecondsSinceMidnight, int16_t LocalOffset) {
   
        /*SetN2kLocalOffset(N2kMsg,17555,62000,120);
        sendOK = NMEA2000.SendMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending Local Offset N2K");  
        }*/
    
        //*****************************************************************************
        // System date/time
        // Input:
        //  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
        //                          to indicate that they are measured at same time.
        //  - SystemDate            Days since 1970-01-01
        //  - SystemTime            seconds since midnight
        //  - TimeSource            see tN2kTimeSource
        // Output:
        //  - N2kMsg                NMEA2000 message ready to be send.
        //inline void SetN2kSystemTime(tN2kMsg &N2kMsg, unsigned char SID, uint16_t SystemDate,
        //             double SystemTime, tN2kTimeSource TimeSource=N2ktimes_GPS) {
        /*SetN2kSystemTime(N2kMsg,1,17555,62000);
        sendOK = NMEA2000.SendMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending Date Time N2K");  
        }*/
    
        //*****************************************************************************
        // GNSS Position Data
        // Input:
        //  - SID                   Sequence ID. If your device is e.g. boat speed and GPS at same time, you can set same SID for different messages
        //                          to indicate that they are measured at same time.
        //  - DaysSince1970         Days since 1.1.1970. You can find sample how to convert e.g. NMEA0183 date info to this on my NMEA0183 library on
        //                          NMEA0183Messages.cpp on function NMEA0183ParseRMC_nc
        //  - Latitude              Latitude in degrees
        //  - Longitude             Longitude in degrees
        //  - Altitude              Altitude in meters
        //  - GNSStype              GNSS type. See definition of tN2kGNSStype
        //  - GNSSmethod            GNSS method type. See definition of tN2kGNSSmethod
        //  - nSatellites           Number of satellites used for data
        //  - HDOP                  Horizontal Dilution Of Precision in meters.
        //  - PDOP                  Probable dilution of precision in meters.
        //  - GeoidalSeparation     Geoidal separation in meters
        // Output:
        //  - N2kMsg                NMEA2000 message ready to be send.
        //inline void SetN2kGNSS(tN2kMsg &N2kMsg, unsigned char SID, uint16_t DaysSince1970, double SecondsSinceMidnight,
        //                     double Latitude, double Longitude, double Altitude,
        //                     tN2kGNSStype GNSStype, tN2kGNSSmethod GNSSmethod,
        //                     unsigned char nSatellites, double HDOP, double PDOP=0, double GeoidalSeparation=0,
        //                     unsigned char nReferenceStations=0, tN2kGNSStype ReferenceStationType=N2kGNSSt_GPS, uint16_t ReferenceSationID=0,
        //                     double AgeOfCorrection=0
        //                     ) {
  
        boatData.Altitude = myGNSS.getAltitudeMSL();
        boatData.nbSat = myGNSS.getSIV();
        boatData.pDop = myGNSS.getPDOP();
        
        SetN2kGNSS(N2kMsg,
                        n2kMsgId,
                        elapsedDays(curDate),
                        (double) ((double)elapsedSecsToday(curDate) + (double) ((double)myGNSS.getMillisecond()/1000.0f)),
                        (double) ((double)boatData.Latitude/10000000.0f), 
                        (double) ((double)boatData.Longitude/10000000.0f),
                        (double) ((double)boatData.Altitude/1000.0f),
                        N2kGNSSt_GPS,
                        N2kGNSSm_GNSSfix,
                        boatData.nbSat,
                        0.8,
                        boatData.pDop,
                        0,
                        0,
                        N2kGNSSt_GPS,
                        0,
                        0);
        sendOK = NMEA2000.SendMsg(N2kMsg);
        //TODO : executer ce code sur l'autre coeur (xTaskCreatePinnedToCore)
        HandleNMEA2000Msg(N2kMsg);
        tN2kDataToNMEA0183.HandleMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending GNSS Position N2K");  
        }else{
          #if DEBUG_ON
          Serial.println("Sending GNSS Position N2K OK");  
          #endif
        }
        
        //*****************************************************************************
        // GNSS DOP data
        // Input:
        //  - SID                   Sequence ID. If your device is e.g. boat speed and GPS at same time, you can set same SID for different messages
        //                          to indicate that they are measured at same time.
        //  - DesiredMode           Desired DOP mode.
        //  - ActualMode            Actual DOP mode.
        //  - HDOP                  Horizontal Dilution Of Precision in meters.
        //  - PDOP                  Probable dilution of precision in meters.
        //  - TDOP                  Time dilution of precision
        // Output:
        //  - N2kMsg                NMEA2000 message ready to be send.
        //inline void SetN2kGNSSDOPData(tN2kMsg& N2kMsg, unsigned char SID, tN2kGNSSDOPmode DesiredMode, tN2kGNSSDOPmode ActualMode,
        //                              double HDOP, double VDOP, double TDOP)
        //TODO : pour le moment la librairie GPS ne gère pas le HDOP et le VDOP
        /*SetN2kGNSSDOPData(N2kMsg,1,N2kGNSSdm_Auto,N2kGNSSdm_Auto,1.2,-0.8,N2kDoubleNA);
        sendOK = NMEA2000.SendMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending GNSS DOP N2K");  
        }*/
  
        //*****************************************************************************
        // COG SOG rapid
        // Input:
        //  - COG                   Cource Over Ground in radians
        //  - SOG                   Speed Over Ground in m/s
        // Output:
        //  - N2kMsg                NMEA2000 message ready to be send.
        //inline void SetN2kCOGSOGRapid(tN2kMsg &N2kMsg, unsigned char SID, tN2kHeadingReference ref, double COG, double SOG) {
        SetN2kCOGSOGRapid(N2kMsg,n2kMsgId,N2khr_true,(double) (((double)boatData.COG/100000.0)* M_PI /180.0),(double) ((double)boatData.SOG/1000.0));
        sendOK = NMEA2000.SendMsg(N2kMsg);
        //TODO : executer ce code sur l'autre coeur (xTaskCreatePinnedToCore)
        HandleNMEA2000Msg(N2kMsg);
        tN2kDataToNMEA0183.HandleMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending GNSS Course/Speed Low Rate N2K");  
        } else{
          #if DEBUG_ON
          Serial.println("Sending GNSS Course/Speed Low Rate N2K OK");   
          #endif
        }
        n2kMsgId++;
      }else if ( IsTimeToUpdate(RapidDataUpdated) && (gpsCogSogUpdated || gpsPositionUpdated)) {
#if DEBUG_ON
        Serial.println("Sending high rate N2K");
#endif
        SetNextUpdate(RapidDataUpdated,RapidDataUpdatePeriod);
    
        //*****************************************************************************
        // Lat/lon rapid
        // Input:
        //  - Latitude               Latitude in degrees
        //  - Longitude              Longitude in degrees
        // Output:
        //  - N2kMsg                NMEA2000 message ready to be send.
        //inline void SetN2kLatLonRapid(tN2kMsg &N2kMsg, double Latitude, double Longitude) {
        SetN2kLatLonRapid(N2kMsg, (double) ((double)boatData.Latitude/10000000.0), (double) ((double)boatData.Longitude/10000000.0));
        sendOK = NMEA2000.SendMsg(N2kMsg);
        //TODO : executer ce code sur l'autre coeur (xTaskCreatePinnedToCore)
        HandleNMEA2000Msg(N2kMsg);
        tN2kDataToNMEA0183.HandleMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending GNSS Position High Rate N2K");  
        }else{
          #if DEBUG_ON
          Serial.println("Sending GNSS Position High Rate N2K OK");  
          #endif
        }
        
        //*****************************************************************************
        // COG SOG rapid
        // Input:
        //  - COG                   Cource Over Ground in radians
        //  - SOG                   Speed Over Ground in m/s
        // Output:
        //  - N2kMsg                NMEA2000 message ready to be send.
        //inline void SetN2kCOGSOGRapid(tN2kMsg &N2kMsg, unsigned char SID, tN2kHeadingReference ref, double COG, double SOG) {
        SetN2kCOGSOGRapid(N2kMsg,n2kMsgId,N2khr_true,(double) (((double)boatData.COG/100000.0)* M_PI /180.0),(double) ((double)boatData.SOG/1000.0));
        sendOK = NMEA2000.SendMsg(N2kMsg);
        //TODO : executer ce code sur l'autre coeur (xTaskCreatePinnedToCore)
        HandleNMEA2000Msg(N2kMsg);
        tN2kDataToNMEA0183.HandleMsg(N2kMsg);
        if(! sendOK){
          Serial.println("Error sending GNSS Course/Speed High Rate N2K");  
        }else{
          #if DEBUG_ON
          Serial.println("Sending GNSS Course/Speed High Rate N2K OK");  
          #endif
        }
        n2kMsgId++;
      }
    }else{
      vTaskDelay(10/portTICK_PERIOD_MS);    
    }

    NMEA2000.ParseMessages();
    tN2kDataToNMEA0183.Update();
    int SourceAddress = NMEA2000.GetN2kSource();
    if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
      Serial.printf("Address Change: New Address=%d\n", SourceAddress);
      preferences.begin("nvs", false);
      preferences.putInt("LastNodeAddress", SourceAddress);
      preferences.end();
    }
    
    if(IsTimeToUpdate(MiscDataUpdated)){
#if DEBUG_ON
      Serial.print("Free Memory : ");
      long freeMemory = ESP.getFreeHeap();
      Serial.print(freeMemory);
      Serial.print(" - ");
      Serial.print((int) 100*freeMemory/(520*1024));
      Serial.println("%");
      Serial.println("Sending MISC N2K");
#endif
      SetNextUpdate(MiscDataUpdated,MiscDataUpdatePeriod);
      NMEA2000.SendProductInformation();
      NMEA2000.SendIsoAddressClaim(); 
    }
  }
}

//void loop(){vTaskDelete(NULL);} //nothing to do... On gère dans les taches par coeur
void loop() {
  if(updateEnCours){
    vTaskDelay(10000/portTICK_PERIOD_MS);
  }
  //Reception des trames NMEA2000 et envoi NMEA0183
  CheckConnectionsNMEA0183();
  //Taches de gestions globales
  server.handleClient();
  led.loop();

  if (isPositionValide()) {
      //Position GPS valide 
    if(blinkState != 2){
      led.blink3();
      blinkState = 2;
    } 
  }else{
    //Attente position GPS
    if(blinkState != 1){
      led.blink1();  
      blinkState = 1;
    }  
  }
  vTaskDelay(10/portTICK_PERIOD_MS);
}



//###################################################################################
//Serveur Web
//###################################################################################
void configureWebServer(){
  server.on("/", home_page);
  server.on("/update", update_page);
  
  /*handling uploading firmware file */
  server.on("/update_go", HTTP_POST, update_go_page, update_HandleFirware);
  server.begin();
}

void home_page(){
  
  String homePage = WEB_HOME_PAGE;
  homePage.replace("[VERSION]", String(VERSION));
  homePage.replace("[ID_DISPOSITIF]", String(GetBoardSerialNumber()));
  homePage.replace("[PROTOCOLE_MODE]", gpsCnxMode);
  
  if(gpsInitialized){
    //homePage.replace("[FIX_GPS]", (_droneSignal.isPositionFixed() ? "OUI" : "En attente"));
    homePage.replace("[NB_SAT]", String(boatData.nbSat));
    //homePage.replace("[HDOP]", String(_droneSignal.getGpsHdop()));
    //String curLat = DD_DDDDDtoDDMMSS(abs(_droneSignal.getLat())/100000.0);
    String curLat = String(abs((double)boatData.Latitude/10000000.0));
    curLat.replace("°", "&deg;");
    homePage.replace("[LAT]", (boatData.Latitude>=0 ? "N" : "S") + curLat);
    //String curLng = DD_DDDDDtoDDMMSS(abs(_droneSignal.getLon())/100000.0);
    String curLng = String(abs((double)boatData.Longitude/10000000.0));
    curLng.replace("°", "&deg;");
    homePage.replace("[LON]", (boatData.Longitude>=0 ? "E" : "O") + curLng);
    homePage.replace("[Alt]", String((double) ((double)boatData.Altitude/1000.0f)));
    homePage.replace("[SPEED]", String((double) ((double)boatData.SOG/1000.0)));
    homePage.replace("[ROUTE]", String((double) (((double)boatData.COG/100000.0)* M_PI /180.0)));
  }else{
    homePage.replace("[FIX_GPS]", "GPS Non initialisé");
  }

  server.send(200, "text/html", homePage); 
}

void update_page(){
  server.send(200, "text/html", WEB_UPDATE_PAGE); 
}

void update_go_page(){
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}

void update_HandleFirware(){
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Update: %s\n", upload.filename.c_str());
    updateEnCours = true;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
      updateEnCours = false;
      Update.printError(Serial1);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    /* flashing firmware to ESP*/
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      updateEnCours = false;
      Update.printError(Serial1);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { //true to set the size to the current progress
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    } else {
      updateEnCours = false;
      Update.printError(Serial1);
    }
  }
}


//###################################################################################
//GPS
//###################################################################################

bool isPositionValide(){
  //return ( _droneSignal.isPositionFixed() && _droneSignal.getSatNumber() >= NUM_SAT_MIN && _droneSignal.getGpsHdop() <= GPS_HDOP_MIN);
  return true;
}

String DD_DDDDDtoDDMMSS( double DD_DDDDD)
{    
 int DD=(int)DD_DDDDD;
 int MM=(int)((DD_DDDDD - DD)*60);
 int SS=(int)((DD_DDDDD - DD)*60-MM)*60;
 return String(DD) + "°" + String(MM) + "'" + String(SS);
}

//###################################################################################
//NMEA2000
//###################################################################################
bool IsTimeToUpdate(unsigned long NextUpdate) {
  return (NextUpdate<millis());
}

unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset=0) {
  return millis()+Period+Offset;
}

unsigned long InitNextUpdate(unsigned long Period) {
  return InitNextUpdate(Period, 0);
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
  while ( NextUpdate<millis() ) NextUpdate+=Period;
}


//###################################################################################
//SEASMART
//###################################################################################
#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500
#define MAX_NMEA2000_MESSAGE_SIZE 82
//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
#if SEND_SEA_SMART
    if (N2kToSeasmart(N2kMsg, millis(), buf, MAX_NMEA2000_MESSAGE_SEASMART_SIZE) == 0 ){  
      SendBufToClients(buf);
    }
#endif

//TODO send UDP and TCP N2K message 
  /*char bufN2k[MAX_NMEA2000_MESSAGE_SIZE]; 
  int n2kMsgLenght = 0;
  N2kMsg.GetBuf(bufN2k, MAX_NMEA2000_MESSAGE_SIZE, n2kMsgLenght);
  Serial.println("Sending UDP NMEA2K message");
  udpNmea2K.beginPacket(broadCastIPAddress, UDP_NMEA2K_PORT);  // Send to UDP
  udpNmea2K.write((byte*)bufN2k, MAX_NMEA2000_MESSAGE_SIZE);
  udpNmea2K.endPacket();*/


}

#define MAX_NMEA0183_MESSAGE_SIZE 82
//NMEA 0183 sentences vary in length, but each sentence is limited to 79 characters or
//less. This length limit excludes the $ and the [CR][LF] characters. The data field block,
//including delimiters is limited to 74 characters or less.
//*****************************************************************************
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {
  //Sending to TCP client
  char buf[MAX_NMEA0183_MESSAGE_SIZE];
  if (NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE) ) {
    //TCP
    SendBufToClients(buf);
    //Sending to UDP
#if DEBUG_ON
    Serial.println("Sending UDP NMEA0183 message");
#endif
    udpNmea0183.beginPacket(broadCastIPAddress, UDP_NMEA0183_PORT);  // Send to UDP
    udpNmea0183.write((uint8_t*)buf, MAX_NMEA0183_MESSAGE_SIZE);
    udpNmea0183.endPacket();

    udpNmea0183.beginPacket(broadCastIPAddressStation, UDP_NMEA0183_PORT);  // Send to UDP
    udpNmea0183.write((uint8_t*)buf, MAX_NMEA0183_MESSAGE_SIZE);
    udpNmea0183.endPacket();
    

    //Serial
    Serial.println(buf);
    Serial1.println(buf);
    Serial2.println(buf);
  }
}

//***********************  WEBSERVER  *****************************************
// NMEA0183 Sätze an alle Clients senden
void SendBufToClients(const char *buf) {
  for (auto it=clientsNMEA0183.begin() ;it!=clientsNMEA0183.end(); it++) {
    if ( (*it)!=NULL && (*it)->connected() ) {
      (*it)->println(buf);
    }
  }
}

//*****************************************************************************
void AddClient(WiFiClient &client) {
#if DEBUG_ON
  Serial.println("New Client.");
#endif
  clientsNMEA0183.push_back(tWiFiClientPtr(new WiFiClient(client)));
}

//*****************************************************************************
void StopClient(LinkedList<tWiFiClientPtr>::iterator &it) {
#if DEBUG_ON
  Serial.println("Client Disconnected.");
#endif
  (*it)->stop();
  it = clientsNMEA0183.erase(it);
}

//*****************************************************************************
void CheckConnectionsNMEA0183() {
  WiFiClient client = serverNMEA0183.available();   // listen for incoming clients

  if ( client ) AddClient(client);

  for (auto it = clientsNMEA0183.begin(); it != clientsNMEA0183.end(); it++) {
    if ( (*it) != NULL ) {
      if ( !(*it)->connected() ) {
        StopClient(it);
      } else {
        if ( (*it)->available() ) {
          char c = (*it)->read();
          if ( c == 0x03 ) StopClient(it); // Close connection by ctrl-c
        }
      }
    } else {
      it = clientsNMEA0183.erase(it); // Should have been erased by StopClient
    }
  }
}
