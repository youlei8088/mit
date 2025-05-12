#ifndef __MATH_OPS_H
#define __MATH_OPS_H

#ifdef __cplusplus
 extern "C" {
#endif
     
#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#include "math.h"

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
float roundf(float x);
void limit_norm(float *x, float *y, float limit);
void limit(float *x, float min, float max);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
     
#ifdef __cplusplus
}
#endif

#endif
     
     
