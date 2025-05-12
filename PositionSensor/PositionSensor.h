#ifndef __POSITIONSENSOR_H
#define __POSITIONSENSOR_H

#include "mbed.h"

void PositionSensor_Init(int CPR, float offset, int ppairs);
void PositionSensor_Sample(float dt);
float PositionSensor_GetMechPosition();
float PositionSensor_GetMechPositionFixed();
float PositionSensor_GetElecPosition();
float PositionSensor_GetMechVelocity();
float PositionSensor_GetElecVelocity();
int PositionSensor_GetRawPosition();
void PositionSensor_ZeroPosition();
void PositionSensor_SetElecOffset(float offset);
void PositionSensor_SetMechOffset(float offset);
int PositionSensor_GetCPR(void);
void PositionSensor_WriteLUT(int new_lut[128]);


#endif
