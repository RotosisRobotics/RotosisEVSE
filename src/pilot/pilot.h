#pragma once
#include <Arduino.h>

// Pilot modulu CP hattini olcer ve A/B/C/D/E state bilgisini uretir.
// State davranisini degistirmek istiyorsan asil mudahale noktasi pilot.cpp'dir.

struct PilotMeasurements {
  float adcHigh;
  float adcLow;
  float cpHigh;
  float cpLow;
  String stateRaw;
  String stateStable;
};

void pilot_init();
void pilot_apply_pwm();
void pilot_update();
// true iken pilot_apply_pwm CP'yi sabit LOW yazar (IEC 61851 / ISO 15118-3 state E/F uyandirma darbesi).
// Yalnizca main.cpp'deki uyandirma dongusu kullanir; bu surede pilot_update cagrilmamalidir.
void pilot_force_cp_low(bool on);
PilotMeasurements pilot_get();

// PWM kontrolü (web dokunmayacak ama main kullanacak)
extern bool pwmEnabled;
extern int  pwmDutyPercent;
