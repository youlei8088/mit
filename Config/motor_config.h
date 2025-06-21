#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#if defined(MD101)

#define NPP 21                  //Number of pole pairs
#define KT .08f                 //N-m per peak phase amp, = WB*NPP*3/2
#define GR 6.0f                 //Gear rtio
#define KT_OUT 0.45f            //KT*GR

#elif defined(MD300)

#define NPP 14                  //Number of pole pairs
#define KT 0.1133f              //N-m per peak phase amp, = WB*NPP*3/2. For GIM6010-6: 0.68 Nm/A (output) / 6.0 GR = 0.1133 Nm/A (motor shaft)
#define GR 6.0f                 //Gear rtio
#define KT_OUT 0.6798f          //KT*GR. For GIM6010-6: 0.1133f * 6.0f = 0.6798f

#endif

#endif
