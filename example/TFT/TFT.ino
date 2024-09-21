#include "Arduino.h"
#include "girl.h"
#include "logo.h"
#include "logo2.h"
#include "pin_config.h"
#include <WiFi.h>
#include "ELMduino.h"
#include "esp_mac.h"

/* external library */
/* To use Arduino, you need to place lv_conf.h in the \Arduino\libraries directory */
#include "TFT_eSPI.h" // https://github.com/Bodmer/TFT_eSPI

TFT_eSPI tft = TFT_eSPI();
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

const char *SSID = "WiFi_OBDII";    // WiFi ELM327 SSID
hw_timer_t * timerA = NULL;
uint8_t timerAID = 0;
uint16_t timerAPrescaler = 80;
int timerAThreshold = 1000000;
uint8_t timerAFlag = 0;


void setup() 
{
  Serial.begin(115200);
  Serial.println("Hello T-Dongle-S3");

  pinMode(TFT_LEDA_PIN, OUTPUT);
  // Initialise TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_DARKGREY);

  digitalWrite(TFT_LEDA_PIN, 0);
  tft.setTextFont(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  timerA = timerBegin(timerAID, timerAPrescaler, true);
  timerAttachInterrupt(timerA, &timerAISR, true);
  timerAlarmWrite(timerA, timerAThreshold, true);
  timerAlarmEnable(timerA);

  // Initialize WiFi Connection
  WiFi.mode(WIFI_AP);
  WiFi.begin(SSID);

}

void loop() 
{ // Put your main code here, to run repeatedly:
  static uint8_t i;
  if(timerAFlag)
  {
  switch (i++) 
    {
    case 0:
      tft.pushImage(0, 0, 160, 80, (uint16_t *)gImage_logo);
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
}

void IRAM_ATTR timerAISR()
{
  timerAFlag = ~timerAFlag;
}
