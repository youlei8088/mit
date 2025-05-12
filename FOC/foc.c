#include "foc.h"

// 两相旋转坐标直接转换为三相静止坐标，得到三相静止坐标的电压值
void abc( float theta, float d, float q, float *a, float *b, float *c)
{
    float cf = FastMath_FastCos(theta);
    float sf = FastMath_FastSin(theta);

    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
}
    
// 三相静止坐标转换为两相旋转坐标d-q  
void dq0(float theta, float a, float b, float c, float *d, float *q)
{
    float cf = FastMath_FastCos(theta);
    float sf = FastMath_FastSin(theta);

    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);    
}

// 三相静止坐标下的电压值转换为PWM占空比
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
    
    *dtc_u = fminf(fmaxf(((u -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX); 
}
    
// Measure zero-offset of the current sensors
void zero_current(int *offset_1, int *offset_2)
{                                
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i<n; i++)
    {                                           // Average n samples of the ADC
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f);       // Write duty cycles
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f);
        ADC1->CR2 |= 0x40000000;                // Begin sample and conversion
        wait(.001);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC1->DR;
    }
    *offset_1 = adc1_offset/n;
    *offset_2 = adc2_offset/n;
}

void init_controller_params(ControllerStruct *controller)
{
    controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->k_d = K_SCALE*I_BW;
    controller->k_q = K_SCALE*I_BW;
}

void reset_foc(ControllerStruct *controller)
{
    TIM1->CCR3 = (PWM_ARR>>1)*(0.5f);
    TIM1->CCR1 = (PWM_ARR>>1)*(0.5f);
    TIM1->CCR2 = (PWM_ARR>>1)*(0.5f);
    controller->i_d_ref = 0;
    controller->i_q_ref = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
}

void commutate(ControllerStruct *controller, float theta)
{
    /// Commutation Loop ///
    controller->loop_count ++;
    
    // Calculate phase currents from ADC readings
    {
        if(PHASE_ORDER)
        { 
           controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);
           controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
        }
        else
        {
            controller->i_b = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);    
            controller->i_c = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);
        }
        controller->i_a = -controller->i_b - controller->i_c;
    }
    
    // dq0 transform on currents
    {
        dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);
    }
    
    // DQ First order low pass filter
    {
        controller->i_q_filt = (0.95f * controller->i_q_filt) + (0.05f * controller->i_q);
        controller->i_d_filt = (0.95f * controller->i_d_filt) + (0.05f * controller->i_d);
    }

    limit_norm(&controller->i_d_ref, &controller->i_q_ref, I_MAX);

    /// PI Controller ///
    float i_d_error = controller->i_d_ref - controller->i_d;
    float i_q_error = controller->i_q_ref - controller->i_q;//  + cogging_current;

    // Integrate Error //
    controller->d_int += controller->k_d*controller->ki_d*i_d_error;   
    controller->q_int += controller->k_q*controller->ki_q*i_q_error;

    controller->d_int = fmaxf(fminf(controller->d_int, OVERMODULATION*controller->v_bus), - OVERMODULATION*controller->v_bus);
    controller->q_int = fmaxf(fminf(controller->q_int, OVERMODULATION*controller->v_bus), - OVERMODULATION*controller->v_bus); 
   
    controller->v_d = controller->k_d*i_d_error + controller->d_int ;
    controller->v_q = controller->k_q*i_q_error + controller->q_int ;

    controller->v_ref = sqrt(controller->v_d*controller->v_d + controller->v_q*controller->v_q);

    limit_norm(&controller->v_d, &controller->v_q, OVERMODULATION*controller->v_bus);       // Normalize voltage vector to lie within curcle of radius v_bus

    abc(controller->theta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
    svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
    
    // Check which phase order to use
    if(PHASE_ORDER)
    {
        TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);    // Write duty cycles
        TIM1->CCR2 = (PWM_ARR)*(1.0f-controller->dtc_v);
        TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_w);
    }
    else
    {
        TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);
        TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_v);
        TIM1->CCR2 = (PWM_ARR)*(1.0f-controller->dtc_w);
    }

    controller->theta_elec = theta;                                            
}
    
    
void torque_control(ControllerStruct *controller)
{
    float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + 
                        controller->t_ff + 
                        controller->kd*(controller->v_des - controller->dtheta_mech);
    
    controller->i_q_ref = torque_ref/KT_OUT;    
    controller->i_d_ref = 0.0f;
}
