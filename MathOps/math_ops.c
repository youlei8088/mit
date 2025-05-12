#include "math_ops.h"

/* Returns maximum of x, y */
float fmaxf(float x, float y)
{
    return (((x)>(y))?(x):(y));
}

/* Returns minimum of x, y */
float fminf(float x, float y)
{
    return (((x)<(y))?(x):(y));
}

/* Returns maximum of x, y, z */
float fmaxf3(float x, float y, float z)
{
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

/* Returns minimum of x, y, z */
float fminf3(float x, float y, float z)
{
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

/* Returns nearest integer */
float roundf(float x)
{
    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
}

/* Scales the lenght of vector (x, y) to be <= limit */
void limit_norm(float *x, float *y, float limit)
{
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit)
    {
        *x = *x * limit/norm;
        *y = *y * limit/norm;
    }
}


void limit(float *x, float min, float max)
{
    *x = fmaxf(fminf(*x, max), min);
}

/* Converts a float to an unsigned int, given range and number of bits */
int float_to_uint(float x, float x_min, float x_max, int bits)
{ 
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
/* converts unsigned int to float, given range and number of bits */  
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

