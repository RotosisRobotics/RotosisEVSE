#pragma once

// Web, Wi-Fi, OTA ve JSON endpoint modulunun giris noktalari.
// Yeni endpoint eklenince implementasyon web_ui.cpp icinde, cagri ise genelde main.cpp'de degil burada kalir.

void web_init();
void web_loop();
bool web_ready_for_ota_validation();
// Kurtarma/kapali-istemci AP'si acikken true (Wi-Fi LED'i yavas yanip soner).
bool web_rescue_ap_active();
