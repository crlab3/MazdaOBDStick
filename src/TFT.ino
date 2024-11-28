#include "Arduino.h"
#include "girl.h"
#include "logo.h"
#include "logo2.h"
#include "dpfregen.h"
#include "pin_config.h"
#include <WiFi.h>
#include "ELMduino.h"
#include "esp_mac.h"
#include "obdData.h"

/* external library */
/* To use Arduino, you need to place lv_conf.h in the \Arduino\libraries directory */
#include "TFT_eSPI.h" // https://github.com/Bodmer/TFT_eSPI

#define DEBUGMODE 1
#define OBD_ERROR_COUNT_MAX 100000000

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite coolantTempSprite = TFT_eSprite(&tft);
IPAddress server(192, 168, 0, 10);
WiFiClient client;
ELM327 myELM327;

enum ObdConnectionState
{
    ELM_NOT_CONNECTED,
    ELM_WIFI_CONNECTED,
    ELM_CLIENT_CONNECTED,
    ELM_DEVICE_CONNECTED
};

void IRAM_ATTR timerAISR();
int8_t obdGetData(uint8_t pidIdx);

const char *SSID = "WiFi_OBDII";    // WiFi ELM327 SSID

// Main 200 ms timer for multiple operations
hw_timer_t * timerA = NULL;
uint8_t timerAID = 0;
uint16_t timerAPrescaler = 80;
int timerAThreshold = 100000;
uint8_t timerAFlag = 0;

hw_timer_t * timerTFT = NULL;
uint8_t timerTFTID = 1;
uint16_t timerTFTPrescaler = 80;
int timerTFTThreshold = 50000;
uint8_t timerTFTFlag = 0;

uint8_t initCycle = 1;

uint8_t pidIndex = 0;
uint8_t numPids = 3;
uint16_t obdErrorCount = 0;

uint8_t k = 0;

obdData obdArray[3] = {obdEngineCoolantTempData,obdEngineOilTempData,obdRegenerationStatusData};

ObdConnectionState connectionState=ELM_NOT_CONNECTED;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Hello T-Dongle-S3");

  pinMode(TFT_LEDA_PIN, OUTPUT);
  // Initialise TFT
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_DARKGREY);

  digitalWrite(TFT_LEDA_PIN, 0);
  tft.setTextFont(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  coolantTempSprite.createSprite(tft.width(),tft.height());
  
  timerTFT = timerBegin(timerTFTID, timerTFTPrescaler, true);
  timerAttachInterrupt(timerTFT, &timerTFTISR, true);
  timerAlarmWrite(timerTFT, timerTFTThreshold, true);
  timerAlarmEnable(timerTFT);
  timerStart(timerTFT);


  timerA = timerBegin(timerAID, timerAPrescaler, true);
  timerAttachInterrupt(timerA, &timerAISR, true);
  timerAlarmWrite(timerA, timerAThreshold, true);
  timerAlarmEnable(timerA);
  timerStart(timerA);


  // Initialize WiFi Connection
  WiFi.mode(WIFI_AP);
  WiFi.begin(SSID);
}

void loop() 
{ // Put your main code here, to run repeatedly:

if(timerAFlag && initCycle)
    {
        timerAFlag = 0;
        if(WiFi.status() == WL_CONNECTED && connectionState == ELM_NOT_CONNECTED)
        {
            #if DEBUGMODE
                Serial.println("Connection state = ELM_WIFI_CONNECTED");
            #endif
            connectionState = ELM_WIFI_CONNECTED;
        }
        if(connectionState == ELM_WIFI_CONNECTED)
        {
            uint32_t connResp = client.connect(server, 35000);
            if(connResp)
            {    
                #if DEBUGMODE
                    Serial.println("Connection state = ELM_CLIENT_CONNECTED");
                #endif
                connectionState = ELM_CLIENT_CONNECTED;
            }
        }
        if(connectionState == ELM_CLIENT_CONNECTED)
        {
            bool elmStatus = myELM327.begin(client, false, 2000);
            if(elmStatus)
            {
                #if DEBUGMODE
                    Serial.println("Connection state = ELM_DEVICE_CONNECTED");
                #endif
                connectionState = ELM_DEVICE_CONNECTED;
                initCycle = 0;
            }
        }
    } 

if(connectionState == ELM_DEVICE_CONNECTED)
{
    int8_t ELMRxState = obdGetData(pidIndex);
    if(ELMRxState == ELM_SUCCESS)
    {
      // Go to the next pidIndex
      pidIndex++;
      // Always modulo the pidIndex
      pidIndex %= numPids;
    }
    if(ELMRxState == OBD_ERROR_COUNT_MAX)
    {
      // Display some kind of error on the TFT display
      // Reset the esp32
      ESP.restart();
    }
    #if DEBUGMODE
        Serial.print("ELMRxState: ");
        Serial.println(ELMRxState);
        Serial.print("Number of OBD errors: ");
        Serial.println(obdErrorCount);
        Serial.print("Current PID Index: ");
        Serial.println(pidIndex);
    #endif
}

  /*if(timerAFlag)
  {
  timerAFlag = ~timerAFlag;
  switch (i++) 
    {
    case 0:
      tft.pushImage(0, 0, 160, 80, (uint16_t *)gImage_dpfregen);
      break;
    case 1:
      tft.pushImage(0, 0, 160, 80, (uint16_t *)gImage_logo2);
      break;
    case 2:
      tft.pushImage(0, 0, 160, 80, (uint16_t *)gImage_girl);
      i = 0;
      break;
    default:
      break;
    }
  }
  */
if(timerTFTFlag)
{
  coolantTempSprite.fillScreen(TFT_BLACK);
  coolantTempSprite.setTextDatum(MC_DATUM);
  coolantTempSprite.setTextColor(TFT_BLUE);
  coolantTempSprite.setTextSize(2);
  coolantTempSprite.drawNumber(obdArray[0].pidData,tft.width()/2,tft.height()/2,4);
  coolantTempSprite.setTextColor(TFT_ORANGE);
  coolantTempSprite.drawNumber(obdArray[1].pidData,tft.width()/2,tft.height()/2+50,4);
  coolantTempSprite.setTextColor(TFT_YELLOW);
  coolantTempSprite.drawNumber(obdArray[2].pidData,tft.width()/2,tft.height()/2-50,4);
  coolantTempSprite.pushSprite(0,0);
  k++;
}
}

void IRAM_ATTR timerAISR()
{
  timerAFlag = ~timerAFlag;
}

void IRAM_ATTR timerTFTISR()
{
  timerTFTFlag = ~timerTFTFlag;
}

int8_t obdGetData(uint8_t pidIdx)
{
  float respValue = myELM327.processPID(obdArray[pidIdx].serviceID, obdArray[pidIdx].pidID, obdArray[pidIdx].numExpResp, obdArray[pidIdx].numExpResp, obdArray[pidIdx].scaleFactor, obdArray[pidIdx].bias);
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
          obdErrorCount = 0;
          // Fresh new data is received, health is at 100
          obdArray[pidIdx].pidData = respValue;
          obdArray[pidIdx].dataHealthScore = 100;     
          #if DEBUGMODE
              Serial.println("PID Query Successful from ELM327!");
              Serial.print("PID Response Value:" );
              Serial.print(obdArray[pidIdx].pidData);
              Serial.print(", Health: ");
              Serial.println(obdArray[pidIdx].dataHealthScore);
          #endif
      }
      else if (myELM327.nb_rx_state == ELM_GETTING_MSG)
      {
          // While getting the new value, previous PID value quality decreases
          if(obdArray[pidIdx].dataHealthScore > 0)
              obdArray[pidIdx].dataHealthScore -= 1;
          else
              obdArray[pidIdx].dataHealthScore = 0;
          // Keep trying to get PID value
          #if DEBUGMODE
              Serial.print("PID value:");
              Serial.print(obdArray[pidIdx].pidData);
              Serial.print(", Health: ");
              Serial.println(obdArray[pidIdx].dataHealthScore);
          #endif
      }
      else
      {
          // Error while waiting for response: PID value quality is set to zero
          obdArray[pidIdx].dataHealthScore = 0;
          obdErrorCount++;
          // After having more errors than expected
          if(obdErrorCount>OBD_ERROR_COUNT_MAX)
          {
              #if DEBUGMODE
                  Serial.println("OBD Error Count Maximum reached with ELM327. Restarting ESP32...");
              #endif
              ESP.restart();
          }
      }
        
      #if DEBUGMODE
          Serial.println("--------------obdGetData END-------------");
      #endif

      return myELM327.nb_rx_state;
}