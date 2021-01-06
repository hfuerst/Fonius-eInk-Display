/////////////////////////////////////////////////////////////////////////////////////
//                                                                                 //
//    Fronius PV and Battery Data Display  on  TTGO ESP32 with eInk-Display        //
//                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////

/*
TTGO T5 V2.3 wireless WiFi basic Wireless Module ESP-32 esp32 2.13 ePaper
( eBay 353040738445 or Search "TTGO T5" )
Features:
2.9″ ePaper module 296*128 pixels.
MCU: ESP32 can interface with other systems to provide Wi-Fi and Bluetooth functionality through its SPI / SDIO or I2C / UART interfaces.
TF CARD
3D ANTENNA
GROVE(I2C)
Lithium battery interface, 500mA Max charging current
Battery Connector: PH-1.25mm
All of the IO pins run at 3.3V.
MCU: ESP32-D0WDQ06 ESP32 can interface with other systems to provide Wi-Fi and Bluetooth functionality through its SPI / SDIO or I2C / UART interfaces.

Specifictions:
Color: White
Size: 15*13*5mm
Clock Speed(Max):240Mhz
Flash:4M bytes 

Package included:
1 x Module
*/

/////////////////////////////////////////////////////////////////////////////////////
// KnowHow

// Fronius Solar Data API Jason
// https://www.fronius.com/de/solarenergie/installateure-partner/technische-daten/alle-produkte/anlagen-monitoring/offene-schnittstellen/fronius-solar-api-json-
// https://www.onderka.com/hausautomation-technik-und-elektronik/nodemcu-json-daten-von-fronius-wechselrichter-einlesen

// TTGO eInk
// https://github.com/ZinggJM/GxEPD2/blob/master/examples/GxEPD2_Example/GxEPD2_Example.ino

// ESP32 debugging and unit testing
// https://docs.platformio.org/en/latest/tutorials/espressif32/arduino_debugging_unit_testing.html


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////


#include <Arduino.h>
#include <string.h>

#include <Wire.h>               // needed for GxEPD2
#include <GxEPD2_BW.h>          // e-Ink-Display Black and White
#include <Adafruit_I2CDevice.h> // needed for Adafruit_GFX.h
#include <Adafruit_GFX.h>       // Fonts
// all stored in Platformio-Library-Path:
// C:\Users\Name\.platformio\lib\Adafruit GFX Library\Fonts
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/Org_01.h>       // 6pt (mini)

#include <WiFi.h>                  // Wlan
#include <WiFiClient.h>            // Connect to Wifi
#include <HTTPClient.h>
#include <ArduinoJson.h>           // Arduino JSON (5.x stable, nicht 6.x)

#include <StreamString.h>          // String for eInk Text+Value-Positioning
#define PrintString StreamString

/******************************************************/
// User-Config

// Wifi-Client-Settings:
const char* ssid      = "YOUR_SSID";
const char* password  = "YOUR PW";
const char* host_name = "ESP_eInk_Fronius";

#define UseDHCP 1 // bei "1" werden die nachfolgenden IPAddressen
                  // dynamisch vom Router vergeben (empfohlen für Clients)
                  // "0" mit festen IPs wird bei Servern empfohlen
IPAddress ip     (192, 168, X, XXX);  // IP for this ESP
IPAddress gateway(192, 168, X, XXX);  // IP Router
IPAddress subnet (255, 255, 255, 0);
IPAddress dns    = gateway;
unsigned long lastWifiCheck = 0;   

// Inverter-Data
const String inverterHostname = "192.168.X.XXX";  // IP Fronius Inverter
float AkkuCapacity   = 10200; // Wh 
float PVpeak         =  9930; // W   
// helpful Markers
float AkkuMinPercent =     7; // % wich are unsused
float AkkuNeedMin    =    70; // % minimum needed for Winter Night
float AkkuNeedMax    =    75; // % usually needed for Winter Night
float PowNeedMin     =   250; // W usually need in Night / early Morning
float PowNeedMax     =   650; // W usually need in Evening

/******************************************************/
// Global Variables

// Fronius Solar API URLs "Meter Data" und "Flow Data"
String urlMeter = "http://" + inverterHostname + "/solar_api/v1/GetMeterRealtimeData.cgi?Scope=Device&DeviceId=0";
String urlFlow  = "http://" + inverterHostname + "/solar_api/v1/GetPowerFlowRealtimeData.fcgi";

// Inverter Global Variables
String dataTimestamp = "0";
String dataTime = "";
String dataDay = "";
String dataMonth = "";
String dataYear = "";
float AkkuSOC = 0;
float AkkuLast = 0;
float AkkuBefore = 0;
float PowAkku = 0;
float PowGrid = 0;
float PowPV   = 0;
float PowGridL1 = 0;
float PowGridL2 = 0;
float PowGridL3 = 0;
float PowLoad = 0;

unsigned long loopCount           = 0;
unsigned long lastLoopCountPrint  = 0;

/******************************************************/
// Display-Settings
// U8g2 constructor

// TTGO T5 V2.3 wireless WiFi basic Wireless Module ESP-32 esp32 2.13 ePaper 
// (with Black White ePaper. Hersteller: GOOD DISPLAY / Typ: GDEH0213B73)
// (SS (SSPI-Port?) is defined in pins_arduino.h)
GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B73

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

void floatTextSolar()
{
  display.setRotation(0);
  display.setTextColor(GxEPD_BLACK);
 
  display.setFullWindow();
  // here we use paged drawing, even if the processor has enough RAM for full buffer
  // so this can be used with any supported processor board.
  // the cost in code overhead and execution time penalty is marginal
  // tell the graphics class to use paged drawing mode
  display.firstPage();
  do
  {
    // this part of code is executed multiple times, as many as needed,
    // in case of full buffer it is executed once
    // IMPORTANT: each iteration needs to draw the same, to avoid strange effects
    // use a copy of values that might change, don't read e.g. from analog or pins in the loop!
    display.fillScreen(GxEPD_WHITE); // set the background to white (fill the buffer with value for white)
    
    /******************************************************/
    // Solar-PV 
    display.setFont(&FreeSans9pt7b); //9pt
    display.setCursor(30, 15);
    display.println("Solar-PV");
    //display.setFont(&FreeMonoBold18pt7b);//18pt
    display.setFont(&FreeSansBold18pt7b);
    //display.setCursor(15, 45);
      PrintString valueString;
      valueString.print(PowPV, 0); //(Value, Decimals)
      valueString.print("W");
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds(valueString, 0, 0, &tbx, &tby, &tbw, &tbh);
      display.setCursor(((display.width() - tbw) / 2)-3, 45);
    display.print(PowPV,0);
    display.println("W");

    /******************************************************/
    //Akku
    //PowAkku = 0; //debug only
    display.setFont(&FreeSans9pt7b); //9pt
    display.setCursor(30, 80);
    display.print("Akku ");
      if (PowAkku < -5000){
        display.fillTriangle(100,80,108,80,104,65,GxEPD_BLACK); 
      }
      if (PowAkku < -2000){
        display.fillTriangle(90,80,98,80,94,65,GxEPD_BLACK);
      }
      if (PowAkku < -600){
        display.fillTriangle(80,80,88,80,84,65,GxEPD_BLACK);
      }
      else if (PowAkku < -50){   // between -600 and -50
        display.drawTriangle(80,80,88,80,84,65,GxEPD_BLACK);
      }
      if (PowAkku > 600){
        display.fillTriangle(80,65,88,65,84,80,GxEPD_BLACK);
      }
      else if (PowAkku > 50){   // between 50 and 600
        display.drawTriangle(80,65,88,65,84,80,GxEPD_BLACK);
      }
      if (PowAkku > 2000){
        display.fillTriangle(90,65,98,65,94,80,GxEPD_BLACK);
      }
      if (PowAkku > 5000){
        display.fillTriangle(100,65,108,65,104,80,GxEPD_BLACK);
      }
      if (PowAkku >= -50 and PowAkku <= 50){
        //display.print(" -");
        //display.drawTriangle(80,66,80,74,95,70,GxEPD_BLACK);
        display.fillRect(78,72,15,2,GxEPD_BLACK);
       }
    //display.setFont(&FreeMonoBold18pt7b);//18pt
    display.setFont(&FreeSansBold18pt7b);
    if (AkkuSOC<100){
      display.setCursor(10, 110);
      display.print(AkkuSOC,1);
    }
    else {
      display.setCursor(22, 110);
      display.print(AkkuSOC,0);
    }
    display.println("%");
    display.setFont(&FreeSans9pt7b);//9pt
    display.setCursor(30, 128);
    if(PowAkku<0) display.print("+");
    display.print(-PowAkku,0);
    display.println("W");

    display.setFont(&Org_01);//6pt
    float PowAkkuPrecentMinute = (-1*PowAkku)/(AkkuCapacity*(1-(AkkuMinPercent/100))*60/100);
    Serial.print(PowAkkuPrecentMinute);
    Serial.println(" %/min");
    //display.setCursor(20, 137);
      PrintString speedString;
      speedString.print("Speed: ");
      speedString.print(PowAkkuPrecentMinute, 2); //(Value, Decimals)
      speedString.print("%je Min");
      display.getTextBounds(speedString, 0, 0, &tbx, &tby, &tbw, &tbh);
      display.setCursor(((display.width() - tbw) / 2)-3, 137);
    display.print(speedString);

    display.setFont(&Org_01);//6pt
    float PowAkkuPrecentHour = (-1*PowAkku)/(AkkuCapacity*(1-(AkkuMinPercent/100))/100);
    Serial.print(PowAkkuPrecentHour);
    Serial.println(" %/std");
      PrintString speedString2;
      speedString2.print("      ");
      speedString2.print(PowAkkuPrecentHour, 2); //(Value, Decimals)
      speedString2.print("%je Std");
      display.getTextBounds(speedString2, 0, 0, &tbx, &tby, &tbw, &tbh);
      display.setCursor(((display.width() - tbw) / 2)-3, 145);
    display.print(speedString2);

    /******************************************************/
    //Verbrauch
    display.setFont(&FreeSans9pt7b);//9pt
    display.setCursor(20, 165);
    display.println("Verbrauch");
    //display.setFont(&FreeSans9pt7b);//9pt
    display.setFont(&FreeSansBold9pt7b);//9pt
    display.setCursor(35, 185);
    display.print(-PowLoad,0);
    display.println("W");

    display.setFont(&Org_01);//6pt
    display.setCursor(30, 202);
    display.print("Netz: ");
    display.print(PowGrid,0);
    display.println(" W");
    display.setCursor(40, 215);
    display.print("L1.: ");
    display.print(PowGridL1,0);
    display.println(" W");
    display.setCursor(40, 225);
    display.print("L2: ");
    display.print(PowGridL2,0);
    display.println(" W");
    display.setCursor(40, 235);
    display.print("L3: ");
    display.print(PowGridL3,0);
    display.println(" W");

    /******************************************************/
    dataTime  = dataTimestamp.substring(11,19);
    dataYear  = dataTimestamp.substring(2,4);
    dataMonth = dataTimestamp.substring(5,7);
    dataDay   = dataTimestamp.substring(8,10);
    Serial.println(dataTimestamp);
    display.setCursor(11,249);
    display.print("UTC ");
    display.print(dataTime);
    display.print(" ");
    display.print(dataDay);
    display.print(".");
    display.print(dataMonth);
    display.print(".");
    display.print(dataYear);

    /******************************************************/
    // BAR Solar-PV on Left 
    int linePos, linePos2, lineColor, barHeight;
    float faktor;
    faktor    = sqrt(PowPV/PVpeak); // = aktuelle Leistung / 9,93kW Peak
    barHeight = display.height() * (1-faktor);
    display.drawRect(-1,0,6,display.height(),GxEPD_BLACK);
    display.fillRect(0,barHeight,4,display.height(),GxEPD_BLACK);
    // Power usually needed
    linePos  = display.height() * (1-sqrt(PowNeedMin/PVpeak));
    linePos2 = display.height() * (1-sqrt(PowNeedMax/PVpeak));
    display.drawLine(6,linePos,6,linePos2,GxEPD_BLACK);
    Serial.print("PowNeedMin-Line: ");
    Serial.println(linePos);
    Serial.print("PowNeedMax-Line: ");
    Serial.println(linePos2);
    // Power actually needed
    linePos  = display.height() * (1-sqrt(-PowLoad/PVpeak));
    linePos2 = display.height() * (1-sqrt(PowPV+PowLoad/PVpeak));
    display.drawRect(4,linePos,5,display.height(),GxEPD_BLACK);
    // scale left: percent Lines 
    float i;
    for (i = 0.1; i < 1; i=i+0.1){
      linePos = display.height() * (1-sqrt(i));
      lineColor = GxEPD_BLACK; 
      if (barHeight<=linePos) lineColor = GxEPD_WHITE;
      display.drawLine(0,linePos,2,linePos,lineColor);
    }
    for (i = 0.01; i < 0.09; i=i+0.01){
      linePos = display.height() * (1-sqrt(i));
      lineColor = GxEPD_BLACK; 
      if (barHeight<=linePos) lineColor = GxEPD_WHITE;
      display.drawLine(3,linePos,4,linePos,lineColor);
    }
    
    /******************************************************/
    // BAR Akku-Percent on Right
    barHeight = display.height() * (1-(AkkuSOC/100));
    display.drawRect(display.width()-11,0,display.width(),display.height(),GxEPD_BLACK);
    display.fillRect(display.width()-10,barHeight,4,display.height(),GxEPD_BLACK);
    // scale right: percent Lines 
    for (i = 0.1; i < 1; i=i+0.1)
    {
      linePos = display.height() * (1-i);
      lineColor = GxEPD_BLACK; 
      if (barHeight<=linePos) lineColor = GxEPD_WHITE;
      display.drawLine(display.width()-9,linePos,display.width(),linePos,lineColor);
    }
    // Line usually needed for Winter-Night
    linePos  = display.height() * (1 - (AkkuNeedMin/100));
    linePos2 = display.height() * (1 - (AkkuNeedMax/100));
    display.drawLine(display.width()-13,linePos,display.width()-13,linePos2,GxEPD_BLACK);
    // Line usually not used
    linePos  = display.height() * (1 - 0);
    linePos2 = display.height() * (1 - (AkkuMinPercent/100));
    display.drawLine(display.width()-13,linePos,display.width()-13,linePos2,GxEPD_BLACK);
    
  }
  while (display.nextPage());
}

/////////////////////////////////////////////////////////////////////////////////////

void connectWiFi(){
  
  Serial.print("Connecting to ssid: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("--------------------------------------");
  Serial.print("WiFi connected to: ");
  Serial.println(WiFi.SSID());
  Serial.print("Kanal............: ");
  Serial.println(WiFi.channel());
  Serial.print("Signal (RX)......: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.print("IP-Adresse.......: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC-Adresse......: ");
  Serial.println(WiFi.macAddress());
  Serial.println("--------------------------------------\n");
  // displayText("WiFi", 8,32,1);
}

/////////////////////////////////////////////////////////////////////////////////////

void getDataSolar()
{
  HTTPClient http;

  // Send request Meter-Data
  http.useHTTP10(true);
  http.begin(urlMeter);
  http.GET();
  // Parse response
  DynamicJsonDocument docMeter(4096);
  deserializeJson(docMeter, http.getStream());
  // Read values
  dataTimestamp = docMeter["Head"]["Timestamp"].as<String>();
  PowGridL1 = docMeter["Body"]["Data"]["SMARTMETER_POWERACTIVE_MEAN_01_F64"].as<float>();
  PowGridL2 = docMeter["Body"]["Data"]["SMARTMETER_POWERACTIVE_MEAN_02_F64"].as<float>();
  PowGridL3 = docMeter["Body"]["Data"]["SMARTMETER_POWERACTIVE_MEAN_03_F64"].as<float>();
  // Disconnect
  http.end();
  
  // Send request Flow-Data
  http.useHTTP10(true);
  http.begin(urlFlow);
  http.GET();
  // Parse response
  DynamicJsonDocument docFlow(4096);
  deserializeJson(docFlow, http.getStream());
  // Read values
  dataTimestamp = docFlow["Head"]["Timestamp"].as<String>();
  AkkuSOC = docFlow["Body"]["Data"]["Inverters"]["1"]["SOC"].as<float>();
  PowGrid = docFlow["Body"]["Data"]["Site"]["P_Grid"].as<float>();
  PowLoad = docFlow["Body"]["Data"]["Site"]["P_Load"].as<float>();
  PowAkku = docFlow["Body"]["Data"]["Site"]["P_Akku"].as<float>();
  if (AkkuLast != PowAkku) {
    AkkuBefore = AkkuLast;
    AkkuLast = PowAkku;
  }
  PowPV = docFlow["Body"]["Data"]["Site"]["P_PV"].as<float>();
  // Disconnect
  http.end();

  Serial.print("TimeStamp: ");
  Serial.print(dataTimestamp);
  Serial.println("");
  Serial.print("Battery.......: ");
  Serial.print(AkkuSOC);
  Serial.println(" %");
  Serial.print("Power Netz....: ");
  Serial.print(PowGrid);
  Serial.println(" W");
  Serial.print("      Netz L1.: ");
  Serial.print(PowGridL1);
  Serial.println(" W");
  Serial.print("      Netz L2.: ");
  Serial.print(PowGridL2);
  Serial.println(" W");
  Serial.print("      Netz L3.: ");
  Serial.print(PowGridL3);
  Serial.println(" W");
  Serial.print("Power Battery.: ");
  Serial.print(PowAkku);
  Serial.println(" W");
  Serial.print("Power Solar...: ");
  Serial.print(PowPV);
  Serial.println(" W");
  Serial.print("Power Load....: ");
  Serial.print(PowLoad);
  Serial.println(" W");
  Serial.println();

}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(39, INPUT_PULLUP); //Button

  Serial.begin(115200); 
  //Serial.begin(9600); 
  while (!Serial);      // wait for serial monitor
  delay(1000);

  Serial.println();
  Serial.println("************************************");
  Serial.println("*  Fronius PV and Battery Display  *");
  Serial.println("************************************");
  Serial.println();
  Serial.println("---SETUP---");
  Serial.println();

  /******************************************************/
  // WIFI

  // WiFi aus
  Serial.println("Wifi Off");
  WiFi.mode(WIFI_MODE_NULL);
  delay(1000);
  // WiFi Client-Mode, AP-Mode off
  Serial.println("Wifi Mode: Client");
  WiFi.mode(WIFI_MODE_STA);
 
  connectWiFi(); 
 
  /******************************************************/
  // e-Ink Display for Lilygo TTGO 20190107 T5_V2.3_2.13
  
  Serial.println("display.init");
  display.init(115200);
  // first update should be full refresh
  delay(1000);

  Serial.println();
  Serial.println("---LOOP---");
  Serial.println();
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

void loop() {

  /******************************************************/
  // Check WiFi-Connection and re-connect
  if(millis() > lastWifiCheck+2000) {  // check every 2 Seconds
    if(WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    lastWifiCheck = millis();
  } 

  /******************************************************/
  // Wait for redo

  loopCount++; 
  //if (millis() > lastLoopCountPrint+600000 or lastLoopCountPrint == 0) // 10 Minutes
  if (millis() > lastLoopCountPrint+60000 or lastLoopCountPrint == 0) // 1 Minute
  //if (millis() > lastLoopCountPrint+10000 or lastLoopCountPrint == 0) // 10 sek
  {
    Serial.print("===> Loops: ");
    Serial.println(loopCount);
    getDataSolar();
    floatTextSolar();
    lastLoopCountPrint = millis();
  } 
  

  /******************************************************/
  // Check Button
  if (digitalRead(39)==0) {
    Serial.println("----- Button Pressed -----");
    getDataSolar();
    floatTextSolar();
    lastLoopCountPrint = millis();
  }
  
}
