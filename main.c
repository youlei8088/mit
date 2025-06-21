#include "mbed.h"
#include "PositionSensor.h"
#include "structs.h"
#include "foc.h"
#include "calibration.h"
#include "hw_setup.h"
#include "math_ops.h" 
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include "user_config.h"
#include "PreferenceWriter.h"
#include "CAN_com.h"
#include "DRV8323.h"

#define REST_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5

#define VERSION_NUM "ZR-V1001"

float   __float_reg[64];        // Floats stored in flash
int     __int_reg[256];         // Ints stored in flash.  Includes position sensor calibration lookup table

PreferenceWriter    prefs(6);

GPIOStruct          gpio;
ControllerStruct    controller;

Serial              pc(PA_2, PA_3);

CAN                 can(PB_8, PB_9, 1000000);      // CAN Rx pin name, CAN Tx pin name
CANMessage          rxMsg;
CANMessage          txMsg;

#if defined(DRV8323RS)
SPI                 drv_spi(PA_7, PA_6, PA_5);
DigitalOut          drv_cs(PA_4);
#elif defined(DRV8323RH)
DigitalOut          drv8323rh_cal(PA_12);
DigitalIn           drv8323rh_fault(PD_2);
#endif

void DRV8323_Enable(void)
{
#if defined(DRV8323RS)
    DRV832x_enable_gd();
#elif defined(DRV8323RH)
    if(!gpio.drv_enable->read())
    {
        gpio.drv_enable->write(1);
        wait_us(100);
    }
#endif
}

void DRV8323_Disable(void)
{
#if defined(DRV8323RS)
    DRV832x_disable_gd();
#elif defined(DRV8323RH)
    gpio.drv_enable->write(0);
#endif
}

volatile int state = REST_MODE;
volatile int state_change;

void onMsgReceived()
{
    can.read(rxMsg);  
    if(((int)rxMsg.id == CAN_ID))
    {
        controller.timeout = 0;
        if(((rxMsg.data[0]==0xFF) && (rxMsg.data[1]==0xFF) && (rxMsg.data[2]==0xFF) && (rxMsg.data[3]==0xFF) && 
            (rxMsg.data[4]==0xFF) && (rxMsg.data[5]==0xFF) && (rxMsg.data[6]==0xFF) && (rxMsg.data[7]==0xFC)))
        {
            state = MOTOR_MODE;
            state_change = 1;
        }
        else if(((rxMsg.data[0]==0xFF) && (rxMsg.data[1]==0xFF) && (rxMsg.data[2]==0xFF) && (rxMsg.data[3]==0xFF) && 
            (rxMsg.data[4]==0xFF) && (rxMsg.data[5]==0xFF) && (rxMsg.data[6]==0xFF) && (rxMsg.data[7]==0xFD)))
        {
            state = REST_MODE;
            state_change = 1;
            gpio.led->write(0);; 
        }
        else if(((rxMsg.data[0]==0xFF) && (rxMsg.data[1]==0xFF) && (rxMsg.data[2]==0xFF) && (rxMsg.data[3]==0xFF) && 
            (rxMsg.data[4]==0xFF) && (rxMsg.data[5]==0xFF) && (rxMsg.data[6]==0xFF) && (rxMsg.data[7]==0xFE)))
        {
            PositionSensor_ZeroPosition();
        }
        else if(state == MOTOR_MODE)
        {
            unpack_cmd(rxMsg, &controller);
        }
        pack_reply(&txMsg, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);
        can.write(txMsg);
    } 
}

void enter_menu_state(void)
{
    DRV8323_Disable();
    printf("\n\r\n\r\n\r");
    printf(" Commands:\n\r");
    wait_us(10);
    printf(" m - Motor Mode\n\r");
    wait_us(10);
    printf(" c - Calibrate Encoder\n\r");
    wait_us(10);
    printf(" s - Setup\n\r");
    wait_us(10);
    printf(" e - Display Encoder\n\r");
    wait_us(10);
    printf(" z - Set Zero Position\n\r");
    wait_us(10);
    printf(" esc - Exit to Menu\n\r");
    wait_us(10);
    state_change = 0;
    gpio.led->write(0);
}

void enter_setup_state(void)
{
    printf("\n\r\n\r Configuration Options \n\r\n\n");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    wait_us(10);
    printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
    wait_us(10);
    state_change = 0;
}
    
void enter_torque_mode(void)
{
    DRV8323_Enable();
    controller.ovp_flag = 0;
    reset_foc(&controller);                     // Tesets integrators, and other control loop parameters
    wait_ms(1);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0;                     // Current Setpoints
    gpio.led->write(1);                         // Turn on status LED
    state_change = 0;
    printf("\n\r Entering Motor Mode \n\r");
}
    
void calibrate(void)
{
    DRV8323_Enable();
    gpio.led->write(1);                         // Turn on status LED
    order_phases(&gpio, &controller, &prefs);   // Check phase ordering
    calibrate(&gpio, &controller, &prefs);      // Perform calibration procedure
    gpio.led->write(0);                         // Turn off status LED
    wait_ms(200);
    printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
    DRV8323_Disable();
    state_change = 0;
}
    
void print_encoder(void)
{
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", PositionSensor_GetMechPosition(), PositionSensor_GetElecPosition(), PositionSensor_GetRawPosition());
    wait_ms(1);
}

/// Current Sampling Interrupt ///
/// This runs at 40 kHz, regardless of of the mode the controller is in ///
extern "C" void TIM1_UP_TIM10_IRQHandler(void) 
{
    if (TIM1->SR & TIM_SR_UIF )
    {
        ADC1->CR2  |= 0x40000000;               // Begin sample and conversion

        PositionSensor_Sample(DT);              // sample position sensor
        controller.adc2_raw = ADC2->DR;         // Read ADC Data Registers
        controller.adc1_raw = ADC1->DR;
        controller.adc3_raw = ADC3->DR;
        controller.theta_elec = PositionSensor_GetElecPosition();
        controller.theta_mech = (1.0f/GR)*PositionSensor_GetMechPosition();
        controller.dtheta_mech = (1.0f/GR)*PositionSensor_GetMechVelocity();  
        controller.dtheta_elec = PositionSensor_GetElecVelocity();
        controller.v_bus = 0.95f*controller.v_bus + 0.05f*((float)controller.adc3_raw)*V_SCALE; //filter the dc link voltage measurement
        
        /// Check state machine state, and run the appropriate function ///
        switch(state)
        {
            case REST_MODE:                     // Do nothing
                if(state_change)
                {
                    enter_menu_state();
                }
                break;
            
            case CALIBRATION_MODE:              // Run encoder calibration procedure
                if(state_change)
                {
                    calibrate();
                }
                break;
             
            case MOTOR_MODE:                    // Run torque control
                if(state_change)
                {
                    enter_torque_mode();
                }
                else
                {
                    if((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0))
                    {
                        controller.i_d_ref = 0;
                        controller.i_q_ref = 0;
                        controller.kp = 0;
                        controller.kd = 0;
                        controller.t_ff = 0;
                    }
                    torque_control(&controller);
                    commutate(&controller, controller.theta_elec);      // Run current loop

                    controller.timeout++;
                }     
                break;
                
            case SETUP_MODE:
                if(state_change)
                {
                    enter_setup_state();
                }
                break;
                
            case ENCODER_MODE:
                print_encoder();
                break;
            
            default:
                break;
        }                 
    }
    TIM1->SR = 0x0; // reset the status register
}


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void)
{
    while(pc.readable())
    {
        char c = pc.getc();
        if(c == 27)
        {
            state = REST_MODE;
            state_change = 1;
            char_count = 0;
            cmd_id = 0;
            gpio.led->write(0);
            for(int i = 0; i<8; i++)
            {
                cmd_val[i] = 0;
            }
        }
        if(state == REST_MODE)
        {
            switch (c)
            {
            case 'c':
                state = CALIBRATION_MODE;
                state_change = 1;
                break;

            case 'm':
                state = MOTOR_MODE;
                state_change = 1;
                break;

            case 'e':
                state = ENCODER_MODE;
                state_change = 1;
                break;

            case 's':
                state = SETUP_MODE;
                state_change = 1;
                break;

            case 'z':
                PositionSensor_SetMechOffset(0);
                PositionSensor_Sample(DT);
                wait_us(20);
                M_OFFSET = PositionSensor_GetMechPosition();
                if (!prefs.ready())
                    prefs.open();
                prefs.flush();      // Write new prefs to flash
                prefs.close();    
                prefs.load(); 
                PositionSensor_SetMechOffset(M_OFFSET);
                printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_OFFSET);
                break;
            }    
        }
        else if(state == SETUP_MODE)
        {
            if(c == 13)
            {
                switch (cmd_id)
                {
                case 'b':
                    I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                    break;
                
                case 'i':
                    CAN_ID = atoi(cmd_val);
                    break;
                
                case 'm':
                    CAN_MASTER = atoi(cmd_val);
                    break;
                
                case 'l':
                    I_MAX = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                    break;
                
                case 'f':
                    I_FW_MAX = fmaxf(fminf(atof(cmd_val), 33.0f), 0.0f);
                    break;
                
                case 't':
                    CAN_TIMEOUT = atoi(cmd_val);
                    break;
                
                default:
                    printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
                    break;
                }
                    
                if (!prefs.ready())
                {
                    prefs.open();
                }
                prefs.flush();      // Write new prefs to flash
                prefs.close();    
                prefs.load();                                              
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++)
                {
                    cmd_val[i] = 0;
                }
            }
            else
            {
                if(char_count == 0)
                {
                    cmd_id = c;
                }
                else
                {
                    cmd_val[char_count-1] = c;   
                }
                pc.putc(c);
                char_count++;
            }
        }
        else if (state == ENCODER_MODE)
        {
            switch (c)
            {
            case 27:
                state = REST_MODE;
                state_change = 1;
                break;
            
            default:
                break;
            }
        }
        else if (state == MOTOR_MODE)
        {
            switch (c)
            {
            case 'd':
                controller.i_q_ref = 0;
                controller.i_d_ref = 0;
                break;
            
            default:
                break;
            }
        }     
    }
}
       
int main()
{
    pc.baud(921600);                        // set serial baud rate
    wait_ms(10);
    PositionSensor_Init(16384, 0.0, NPP); 
    Init_All_HW(&gpio);                     // Setup PWM, ADC, GPIO
    wait_ms(100);
    
    gpio.drv_enable->write(1);
    wait_ms(1);

#if defined(DRV8323RS)
    // DRV8323RS Config
    {
        DRV832x_Init(&drv_spi, &drv_cs);
        wait_ms(100);
        DRV832x_calibrate();                                                        // Current offset calibration
        wait_us(100);
        DRV832x_write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);     // 3x PWM mode, Clear latched fault bits.
        wait_us(100);
        DRV832x_write_HSR(LOCK_OFF, IDRIVEP_HS_120MA, IDRIVEN_HS_240MA);            // IDRIVEP_HS_120MA, IDRIVEN_HS_240MA
        wait_us(100);
        DRV832x_write_LSR(1, TDRIVE_500NS, IDRIVEP_LS_120MA, IDRIVEN_LS_240MA);     // TDRIVE_500NS, IDRIVEP_LS_120MA, IDRIVEN_LS_240MA
        wait_us(100);
        DRV832x_write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0); // CSA_GAIN_40
        wait_us(100);
        DRV832x_write_OCPCR(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);  //DEADTIME_200NS, Overcurrent deglitch time of 8 ¦Ìs, VDS_LVL_1_88
        wait_us(100);
        DRV832x_read_register(FSR1);
        wait_us(100);
    }
#elif defined(DRV8323RH)
    // DRV8323RH Config
    {
        drv8323rh_cal.write(0);
        gpio.led->write(1);
    }
#endif
    
    zero_current(&controller.adc1_offset, &controller.adc2_offset);     // Measure current sensor zero-offset
    DRV8323_Disable();
    wait_ms(100);
    
    // If preferences haven't been user configured yet, set defaults 
    prefs.load();                                                       // Read flash
    if(isnan(E_OFFSET))                         {E_OFFSET = 0.0f;}
    if(isnan(M_OFFSET))                         {M_OFFSET = 0.0f;}
    if(isnan(I_BW) || I_BW==-1)                 {I_BW = 500;} // Lowered default I_BW for more conservative start
    if(isnan(I_MAX) || I_MAX ==-1)              {I_MAX = 40;}
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1)        {I_FW_MAX = 0;}
    // Set fixed -60 to +60 degree position limits (120 degree total range).
    // These values define the permissible range starting from M_OFFSET.
    // THETA_MIN is __float_reg[4] (user_config.h)
    // THETA_MAX is __float_reg[5] (user_config.h)
    const float pi_f = 3.141592653589793f; 
    const float limit_degrees = 60.0f; // +/- 60 degrees
    const float limit_radians = limit_degrees * pi_f / 180.0f; // Ensure pi_f is used here

    __float_reg[4] = -limit_radians;               // THETA_MIN set to -60 degrees in radians (relative to M_OFFSET)
    __float_reg[5] = limit_radians;                // THETA_MAX set to +60 degrees in radians (relative to M_OFFSET)
    if(isnan(CAN_ID) || CAN_ID==-1)             {CAN_ID = 1;}
    if(isnan(CAN_MASTER) || CAN_MASTER==-1)     {CAN_MASTER = 0;}
    if(isnan(CAN_TIMEOUT) || CAN_TIMEOUT==-1)   {CAN_TIMEOUT = 0;}
    PositionSensor_SetElecOffset(E_OFFSET);                             // Set position sensor offset
    PositionSensor_SetMechOffset(M_OFFSET);
    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));
    PositionSensor_WriteLUT(lut);                                       // Set potision sensor nonlinearity lookup table
    
    can.filter(CAN_ID , 0xFFF, CANStandard, 0);                                                           
    txMsg.id = CAN_MASTER;
    txMsg.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);  

#if defined(DRV8323RS)
    pc.printf("\n\r\n\r MiniCheetah-DRV8323RS\n\r\n\r");
#elif defined(DRV8323RH)
    pc.printf("\n\r\n\r MiniCheetah-DRV8323RH\n\r\n\r");
    gpio.led->write(0);
#endif
    wait_ms(10);
    printf("\n\r Debug Info:\n\r");
    printf(" Firmware Version: %s\n\r", VERSION_NUM);
    printf(" ADC1 Offset: %d    ADC2 Offset: %d\n\r", controller.adc1_offset, controller.adc2_offset);
    printf(" Position Sensor Electrical Offset:   %.4f\n\r", E_OFFSET);
    printf(" Output Zero Position:  %.4f\n\r", M_OFFSET);
    printf(" CAN ID:  %d\n\r", CAN_ID);
    pc.attach(&serial_interrupt);                                       // attach serial interrupt
    
    reset_foc(&controller);                                             // Reset current controller
    wait_ms(100);

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);                            // commutation > communication
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    
    init_controller_params(&controller);
    controller.v_bus = V_BUS;
    controller.mode = 0;
    
    state_change = 1;
    TIM1->CR1 ^= TIM_CR1_UDIS;

    while(1)
    {
#if defined(DRV8323RS)
        DRV832x_print_faults();
#elif defined(DRV8323RH)
        if(!drv8323rh_fault.read())
        {
            printf("Motor Drive Fault!\n\r");
        }
#endif
        wait_ms(100);
    }
}

