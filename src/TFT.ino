#include "Arduino.h"
#include "dpfregen.h"
#include "pin_config.h"
#include <WiFi.h>
#include "ELMduino.h"
#include "esp_mac.h"
#include "obdData.h"
#include <FastLED.h>
#include "OneButton.h" // https://github.com/mathertel/OneButton
#include "TFT_eSPI.h" // https://github.com/Bodmer/TFT_eSPI
#include "digital7font.h"
#include "mzdlogo.h"

#define DEBUGMODE 1
#define OBD_ERROR_COUNT_MAX 100
// Pin definitions for RGB LED CI,DI
#define DATA_PIN 3
#define CLOCK_PIN 13

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite mainSprite = TFT_eSprite(&tft);
IPAddress server(192, 168, 0, 10);
WiFiClient client;
ELM327 myELM327;
OneButton button(BTN_PIN, true);
CRGB onBoardRGBLED;

enum ObdConnectionState
{
    ELM_NOT_CONNECTED,
    ELM_WIFI_CONNECTED,
    ELM_CLIENT_CONNECTED,
    ELM_DEVICE_CONNECTED
};

// Function Prototypes
void IRAM_ATTR timerAISR();
void IRAM_ATTR timerTFTISR();
int8_t obdGetData(uint8_t);
void singleClick();

// GLOBAL VARIABLES
const char *SSID = "WiFi_OBDII";    // WiFi ELM327 SSID

// OBD update timer (200 ms)
hw_timer_t * timerA = NULL;
uint8_t timerAID = 0;
uint16_t timerAPrescaler = 80;
int timerAThreshold = 100000;
uint8_t timerAFlag = 0;

// TFT update timer (1000 ms)
hw_timer_t * timerTFT = NULL;
uint8_t timerTFTID = 1;
uint16_t timerTFTPrescaler = 80;
int timerTFTThreshold = 1000000;
uint8_t timerTFTFlag = 0;

uint8_t initCycle = 1;

uint8_t pidIndex = 0;
uint8_t numPids = 3;
uint16_t obdErrorCount = 0;

uint8_t buttonPressed = 0;
uint8_t brightnessLevelDiv = 1;

obdData obdArray[3] = {obdEngineCoolantTempData,obdEngineOilTempData,obdRegenerationStatusData};
ObdConnectionState connectionState=ELM_NOT_CONNECTED;

void setup() 
{
    // Setup LED Configuration of a single RGB LED on Board
    FastLED.addLeds<APA102, LED_DI_PIN, LED_CI_PIN, BGR>(&onBoardRGBLED, 1);
    onBoardRGBLED = CRGB(255,255,255);
    FastLED.show();
    
    // Start serial debugging
    #if DEBUGMODE
    Serial.begin(115200);
    Serial.println("Hello T-Dongle-S3");
    #endif
    
    // Initialise TFT
    tft.init();
    tft.setRotation(2);
    tft.fillScreen(TFT_BLACK);
    
    // Set TFT background LED to on
    pinMode(TFT_LEDA_PIN, OUTPUT);
    analogWrite(TFT_LEDA_PIN, 0);

    // Sprite creation
    mainSprite.createSprite(tft.width(),tft.height());

    // Create TFT timer
    timerTFT = timerBegin(timerTFTID, timerTFTPrescaler, true);
    timerAttachInterrupt(timerTFT, &timerTFTISR, true);
    timerAlarmWrite(timerTFT, timerTFTThreshold, true);
    timerAlarmEnable(timerTFT);
    timerStart(timerTFT);

    // Create timer for OBD updates
    timerA = timerBegin(timerAID, timerAPrescaler, true);
    timerAttachInterrupt(timerA, &timerAISR, true);
    timerAlarmWrite(timerA, timerAThreshold, true);
    timerAlarmEnable(timerA);
    timerStart(timerA);

    button.attachClick(singleClick);

    // Initialize WiFi Connection
    WiFi.mode(WIFI_AP);
    WiFi.begin(SSID);
    tft.pushImage(-40, 40, 160, 80, (uint16_t *)gImage_mzdlogo);
}

void loop() 
{ 
    // Call button callback function to€ trigger interrupt
    button.tick();

    // Build connection to WiFi OBD module
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

    // WiFi Connection is stabilized, start OBD command
    if(timerAFlag && connectionState == ELM_DEVICE_CONNECTED)
    {
        timerAFlag = 0;
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

    // Update the TFT Screen with a main sprite
    if(timerTFTFlag && connectionState == ELM_DEVICE_CONNECTED)
    {
        mainSprite.loadFont(digital7font);
        mainSprite.fillScreen(TFT_BLACK);
        mainSprite.setTextDatum(MC_DATUM);
        mainSprite.setTextSize(2);
        mainSprite.setTextColor(TFT_BLUE);
        mainSprite.drawNumber(obdArray[0].pidData,tft.width()/2,tft.height()/2,4);
        mainSprite.setTextColor(TFT_ORANGE);
        mainSprite.drawNumber(obdArray[1].pidData,tft.width()/2,tft.height()/2+50,4);
        mainSprite.setTextColor(TFT_YELLOW,false);
        if(obdArray[2].pidData>0)
        {
            mainSprite.drawString("DPF",tft.width()/2,tft.height()/2-50,4);    
        }
        else
        {
            mainSprite.drawString("---",tft.width()/2,tft.height()/2-50,4); 
        }
        mainSprite.pushSprite(0,0);
        
        // Turn the RGB LED to yellow for showing DPF regeneration
        if(obdArray[2].pidData>0)
        {
            // Show that regeneration is in progress!
            onBoardRGBLED = CRGB(180/brightnessLevelDiv,255/brightnessLevelDiv,0);
        }
        // If no regeneration is in progress, change to show temperature value
        else
        {
            if(obdArray[0].pidData<80 && obdArray[1].pidData<80)
            {
                onBoardRGBLED = CRGB(0,0,255/brightnessLevelDiv);
            } 
            if(obdArray[0].pidData>=80 && obdArray[1].pidData>=80 && obdArray[0].pidData<110 && obdArray[1].pidData<110)
            {
                onBoardRGBLED = CRGB(0,255/brightnessLevelDiv,0);
            }
            if(obdArray[0].pidData>=110)
            {
                // Alert only when coolant has higher temperature than 110C
                onBoardRGBLED = CRGB(255/brightnessLevelDiv,0,0);
            }
        }
        FastLED.show();
    }
    if(buttonPressed)
    {
        buttonPressed = 0;
        if(brightnessLevelDiv == 1)
        {
            brightnessLevelDiv = 5;
            analogWrite(TFT_LEDA_PIN, 250);
        }

        else
        {
            analogWrite(TFT_LEDA_PIN, 0);
            brightnessLevelDiv = 1;
        }
            
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

void singleClick()
{
    buttonPressed = 1;
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