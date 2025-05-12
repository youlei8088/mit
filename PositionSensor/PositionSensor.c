#include "PositionSensor.h"
#include "math_ops.h"

float position, ElecPosition, ElecOffset, MechPosition, MechOffset, modPosition, oldModPosition, oldVel, velVec[40], MechVelocity, ElecVelocity, ElecVelocityFilt;
int raw, _CPR, rotations, old_counts, _ppairs;
SPI *spi;
DigitalOut *cs;
int offset_lut[128];

void PositionSensor_Init(int CPR, float offset, int ppairs)
{
    _CPR = CPR;
    _ppairs = ppairs;
    ElecOffset = offset;
    rotations = 0;
    spi = new SPI(PC_12, PC_11, PC_10);
    spi->format(16, 1);
    spi->frequency(25000000);

    cs = new DigitalOut(PA_15);
    cs->write(1);
    MechOffset = offset;
    modPosition = 0;
    oldModPosition = 0;
    oldVel = 0;
    raw = 0;
}
    
void PositionSensor_Sample(float dt)
{
    GPIOA->ODR &= ~(1 << 15);
    raw = spi->write(0);
    raw = raw>>2;                                                           //Extract last 14 bits
    GPIOA->ODR |= (1 << 15);
    
    int off_1 = offset_lut[raw>>7];
    int off_2 = offset_lut[((raw>>7)+1)%128];
    int off_interp = off_1 + ((off_2 - off_1)*(raw - ((raw>>7)<<7))>>7);    // Interpolate between lookup table entries
    int angle = raw + off_interp;                                           // Correct for nonlinearity with lookup table from calibration
    if(angle - old_counts > _CPR/2)
    {
        rotations -= 1;
    }
    else if (angle - old_counts < -_CPR/2)
    {
        rotations += 1;
    }
    
    old_counts = angle;
    oldModPosition = modPosition;
    modPosition = ((2.0f*PI * ((float) angle))/ (float)_CPR);
    position = (2.0f*PI * ((float) angle+(_CPR*rotations)))/ (float)_CPR;
    MechPosition = position - MechOffset;
    float elec = ((2.0f*PI/(float)_CPR) * (float) ((_ppairs*angle)%_CPR)) + ElecOffset;
    if(elec < 0)
    {
        elec += 2.0f*PI;
    }
    else if(elec > 2.0f*PI)
    {
        elec -= 2.0f*PI;
    }
    ElecPosition = elec;
    
    float vel;
    if((modPosition-oldModPosition) < -3.0f)
    {
        vel = (modPosition - oldModPosition + 2.0f*PI)/dt;
    }
    else if((modPosition - oldModPosition) > 3.0f)
    {
        vel = (modPosition - oldModPosition - 2.0f*PI)/dt;
    }
    else
    {
        vel = (modPosition-oldModPosition)/dt;
    }    
    
    int n = 40;
    float sum = vel;
    for (int i = 1; i < (n); i++)
    {
        velVec[n - i] = velVec[n-i-1];
        sum += velVec[n-i];
    }
    velVec[0] = vel;
    MechVelocity =  sum/((float)n);
    ElecVelocity = MechVelocity*_ppairs;
    ElecVelocityFilt = 0.99f*ElecVelocityFilt + 0.01f*ElecVelocity;
}

int PositionSensor_GetRawPosition()
{
    return raw;
}

float PositionSensor_GetMechPositionFixed()
{
    return MechPosition+MechOffset;
}
    
float PositionSensor_GetMechPosition()
{
    return MechPosition;
}

float PositionSensor_GetElecPosition()
{
    return ElecPosition;
}

float PositionSensor_GetElecVelocity()
{
    return ElecVelocity;
}

float PositionSensor_GetMechVelocity()
{
    return MechVelocity;
}

void PositionSensor_ZeroPosition()
{
    rotations = 0;
    MechOffset = 0;
    PositionSensor_Sample(.00025f);
    MechOffset = PositionSensor_GetMechPosition();
}
    
void PositionSensor_SetElecOffset(float offset)
{
    ElecOffset = offset;
}

void PositionSensor_SetMechOffset(float offset)
{
    MechOffset = offset;
}

int PositionSensor_GetCPR()
{
    return _CPR;
}

void PositionSensor_WriteLUT(int new_lut[128])
{
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
}

