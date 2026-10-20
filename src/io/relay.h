#pragma once
#include <Arduino.h>

// Role modulu hem klasik role cikisini hem de latch set/reset cikislarini yonetir.
// Donanim degisikliginde app_pins.h ve relay.cpp birlikte kontrol edilmelidir.
// Kapatma (SET) gecikmeli; acma (RESET) relay_core::off_decision kapisindan gecer:
// A/E/F'de hemen, aksi halde akim ~0 olunca (kisa zaman asimiyla) -- yuk altinda acma yapilmaz.

void relay_init();
void relay_set(bool on);
bool relay_get();
// Kosulsuz acma (fis cikti / yedek): role + latch RESET.
void relay_force_off_now();
// Kontaktor kesin birakik mi? (role cikisi kapali + latch son pulse RESET + bekleyen SET yok)
bool relay_contacts_open();

// Röle AUTO yönetimi (yavas dongu). Kontaktor yalniz durum C/D + PWM acik + sarj duty'sinde kapanir.
// Manuel modda da (relay_set) bu kosul kalkarsa kontaktor guvenli acma yoluyla acilir.
void relay_update_auto(const String& stableState, bool pwmEnabled, int pwmDutyPercent);

// Hizli CP yolu (~25 ms): durum C/D degilse kontaktoru gecikmesiz ac (A/E/F hemen, B akim ~0 olunca).
void relay_fast_guard(char stableState);

// AUTO kontrol
void relay_set_auto_enabled(bool en);
bool relay_is_auto_enabled();

// Röle geçişlerini yavaşlat (ms)
void relay_set_min_switch_ms(uint32_t ms);

// SET latch pulse'u stable state'e gore (C/D + PWM). RESET guvenli acma yolundadir.
void relay_handle_state_pulse(const String& stableState, bool pwmEnabled, int pwmDutyPercent);

// Servis paneli: elle latch SET/RESET (modul durumu da guncellenir). Kosul kontrolu cagirandadir.
void relay_latch_manual_set();
void relay_latch_manual_reset();
