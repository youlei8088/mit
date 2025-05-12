#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "foc.h"
#include "mbed.h"
#include "PositionSensor.h"
#include "PreferenceWriter.h"
#include "user_config.h"

#define V_CAL 0.125f;

void order_phases(GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);
void calibrate(GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);

#endif
