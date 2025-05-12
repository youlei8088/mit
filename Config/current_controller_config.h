#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

// Current controller///
#define K_SCALE         0.0001f     // K_loop/Loop BW (Hz) 0.0042
#define KI_D            0.0255f     // PI zero, in radians per sample
#define KI_Q            0.0255f     // PI zero, in radians per sample
#define V_BUS           24.0f       // Volts
#define OVERMODULATION  1.15f       // 1.0 = no overmodulation

//Observer//
#define DT              0.000025f

#endif
