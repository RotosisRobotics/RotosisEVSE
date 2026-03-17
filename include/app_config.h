#pragma once
#include <Arduino.h>

// Kalibrasyon / esikler (webden degisebilir)
extern float CP_DIVIDER_RATIO;

extern float TH_A_MIN;
extern float TH_B_MIN;
extern float TH_C_MIN;
extern float TH_D_MIN;
extern float TH_E_MIN;

extern float marginUp;
extern float marginDown;

extern int   stableCount;
extern int   loopIntervalMs;
