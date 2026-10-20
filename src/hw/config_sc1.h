#pragma once
#include <Arduino.h>

// SC_1 - ilk sarj karti (atolyede calisan cihaz, 192.168.1.200).
// Bu dosyadaki degerler eski include/app_pins.h'den BIREBIR tasindi.
// SC_1 davranisi bozulmasin diye degerleri degistirme.

#define HARDWARE_ID "SC_1"

// OLED (I2C: Wire.begin(SDA, SCL))
#define OLED_SDA 14
#define OLED_SCL 13

// CP / role / guc elektroniigi hatlari.
#define CP_PWM_PIN       2
#define CP_ADC_PIN       1
#define RELAY_PIN        3

// 3 faz akim sensor girisleri.
// Not: SC_1'de akim olcum kodu bugune kadar hep derlenip calisiyordu
// (role guvenli acma karari current_sensor_quick_max_a() kullanir).
// Davranis birebir kalsin diye 1 birakildi.
#define CURRENT_SENSORS_ENABLED 1
#define CURRENT_SENSOR_PIN_A 4
#define CURRENT_SENSOR_PIN_B 5
#define CURRENT_SENSOR_PIN_C 6

// Ayri RGB LED (simdilik uygulama mantiginda kullanilmiyor)
#define RGB_LED_PIN      16

#define STATE_LED_PIN    8
#define WIFI_LED_PIN     17
#define ERROR_LED_PIN    18

// Geri uyumluluk (eski isim)
#define BLUE_LED_PIN     STATE_LED_PIN

#define MOSFET_RESET_PIN 7
#define MOSFET_SET_PIN   15

// Role cikisi aktif-high
#define RELAY_ACTIVE_LOW 0

// Farkli role donanimlari icin cikis secimi.
// 1 = klasik role cikisi kullan
// 1 = SET/RESET latch cikislarini kullan
// 1 = latch pulse'lari dogrudan stable state gecisinden tetiklenir
// 0 = latch pulse'lari role cikisi degisiminden tetiklenir
#define RELAY_USE_OUTPUT_PIN 1
#define RELAY_USE_LATCH_PINS 1
#define RELAY_LATCH_FOLLOW_STATE 1
#define RELAY_LATCH_PULSE_MS 100

// CP PWM surucu ayarlari.
#define PWM_CHANNEL      0
#define PWM_FREQ         1000
#define PWM_RESOLUTION   10

// 32A limiti icin IEC 61851'e gore yaklasik duty.
#define PWM_DUTY_32A     53

// OTA: SC_1 eski manifest yolunu kullanmaya devam eder (geriye donuk uyumluluk).
#define HW_OTA_MANIFEST_URL \
  "https://raw.githubusercontent.com/RotosisRobotics/RotosisEVSE/main/version.json"
// 1 olursa manifestteki "hw" alani HARDWARE_ID ile karsilastirilir.
// SC_1'de 0: davranis eskisiyle ayni kalir.
#define HW_OTA_CHECK_HW 0
