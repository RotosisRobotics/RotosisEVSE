#pragma once
// Dolu arac korumasi ayari: NVS ("evsefull") + JSON. Sayac/kilit mantigi main.cpp'de.
#include <stddef.h>
#include <stdint.h>

struct FullCfg {
  bool enabled;
  uint16_t maxSec;     // bu sureden kisa C/D donemi "kisa deneme" sayilir
  uint16_t maxWh;      // bu enerjiden az verilen donem "kisa deneme" sayilir
  uint8_t  maxCycles;  // ust uste kac kisa denemede kilitlenir
};

void full_cfg_init();
FullCfg full_cfg_get();
bool full_cfg_set(const FullCfg& c, const char** err);
// {"on":..,"sec":..,"wh":..,"n":..,"locked":..,"cycles":..}
void full_status_json(char* out, size_t outLen);

// main.cpp saglar.
bool full_runtime_locked();
uint8_t full_runtime_cycles();
void full_runtime_clear();
