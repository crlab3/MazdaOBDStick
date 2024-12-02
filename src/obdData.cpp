#include "obdData.h"
#include "ELMduino.h"
#define SID_GETCURRENT 1
#define SID_GETBYID 0x22

obdData obdEngineRpmData = {SID_GETCURRENT,ENGINE_RPM,0,0,1,2,0.25,0}; 
obdData obdEngineCoolantTempData = {SID_GETCURRENT,ENGINE_COOLANT_TEMP,0,0,1,1,1,-40.0}; 
obdData obdEngineOilTempData = {SID_GETCURRENT,ENGINE_OIL_TEMP,0,0,1,1,1,-40.0}; 
obdData obdEngineLoadData = {SID_GETCURRENT,ENGINE_LOAD,0,0,1,1,100.0/255.0,0};
obdData obdThrottlePositionData = {SID_GETCURRENT,THROTTLE_POSITION,0,0,1,1,100.0/255.0,0};
obdData obdRegenerationStatusData = {SID_GETBYID,0x0380,0,0,1,1,1,0};