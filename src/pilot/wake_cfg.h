#pragma once
// Arac uyandirma ayari: NVS ("evsewake") + JSON. Durum makinesi main.cpp'de (wake_core).
#include <stddef.h>
#include <stdint.h>

#include "wake_core.h"

void wake_cfg_init();
evse_wake::Config wake_cfg_get();
uint32_t wake_cfg_gen();  // ayar her kaydedildiginde artar (main.cpp makineye uygular)
bool wake_cfg_set(const evse_wake::Config& c, const char** err);
// {"on":..,"seq":"B1,B1,F","max":3,"waitMs":..,"b1Ms":..,"fMs":..,"pwm5Ms":..,"efMs":..,"phase":"..","attempt":n}
void wake_status_json(char* out, size_t outLen);

// main.cpp saglar (calisan makinenin durumu).
const char* wake_runtime_phase();
uint8_t wake_runtime_attempts();
