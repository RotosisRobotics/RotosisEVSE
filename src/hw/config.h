#pragma once

// Donanim secici. platformio.ini'deki build flag'e gore dogru kart dosyasini alir.
// Yeni donanim eklerken: config_scN.h olustur, asagiya bir #elif satiri ekle,
// platformio.ini'de yeni env'e -D HW_SCN koy.

#if defined(HW_SC1) && defined(HW_SC2)
#error "HW_SC1 ve HW_SC2 ayni anda tanimli olamaz"
#endif

#if defined(HW_SC2)
#include "config_sc2.h"
#else
// HW_SC1 veya hic flag yoksa varsayilan SC_1.
#include "config_sc1.h"
#endif

#ifndef HARDWARE_ID
#error "HARDWARE_ID tanimli degil"
#endif
