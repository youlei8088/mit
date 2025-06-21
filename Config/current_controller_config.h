#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

// Current controller///
// For GIM6010-6 with R=0.55, L=0.00045. Target Kp = wc*L, Target Ki_factor = Ts*R/L
// wc = 2*PI*I_BW_Hz. controller->k_d = K_SCALE*I_BW_Hz. So K_SCALE should be 2*PI*L.
// 2*PI*0.00045 = 0.002827
#define K_SCALE         0.002827f     // Target: 2*PI*L. For Kp = wc*L (approx)
// Ts*R/L = (1/40000)*0.55/0.00045 = 0.03055
#define KI_D            0.03055f     // PI zero, in radians per sample. Approx Ts*R/L
#define KI_Q            0.03055f     // PI zero, in radians per sample. Approx Ts*R/L
#define V_BUS           24.0f       // Volts
#define OVERMODULATION  1.15f       // 1.0 = no overmodulation

//Observer//
#define DT              0.000025f

#endif
