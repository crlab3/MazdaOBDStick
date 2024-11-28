#include <stdint.h>

#ifndef OBDDATA_H
#define OBDDATA_H
//TYPEDEFs
typedef struct{
    uint8_t serviceID;
    uint16_t pidID;
    float pidData;
    uint8_t dataHealthScore;
    uint8_t numExpResp;
    uint8_t numExpBytes;
    double scaleFactor;
    float bias;
} obdData;

extern obdData obdEngineRpmData; 
extern obdData obdEngineCoolantTempData; 
extern obdData obdEngineOilTempData; 
extern obdData obdEngineLoadData; 
extern obdData obdThrottlePositionData; 
extern obdData obdRegenerationStatusData; 
#endif