#ifndef OLED_UI_H
#define OLED_UI_H

#include <Arduino.h>

// Ekranı ve I2C'yi başlatır
void oled_init();

// Ekranı günceller
// stateStable: A, B, C durumu
// amps: Ölçülen akım (current_sensor'den gelen)
// relayOn: Röle açık mı kapalı mı
// staConnected: Wi-Fi bağlı mı
// cableConnected: Kablo takılı mı
void oled_draw(const String& stateStable,
               float ia,
               float ib,
               float ic,
               float powerW,
               float energyKwh,
               uint32_t chargeSeconds,
               bool relayOn,
               bool staConnected,
               bool cableConnected);

#endif