#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#if defined(MD101)

#define NPP 21                  //Number of pole pairs
#define KT .08f                 //N-m per peak phase amp, = WB*NPP*3/2
#define GR 6.0f                 //Gear rtio
#define KT_OUT 0.45f            //KT*GR

#elif defined(MD300)

#define NPP 14                  //Number of pole pairs
#define KT .08f                 //N-m per peak phase amp, = WB*NPP*3/2
#define GR 6.0f                 //Gear rtio
#define KT_OUT 1.0f            //KT*GR

#endif

#endif
