#pragma once
// Seri log "tee": LogOut'a yazilan her sey hem Serial'e gider hem de RAM'deki
// halka tampona (log_ring) zaman damgali satir olarak duser. Wi-Fi'den GET /api/log ile okunur.
// Kullanim: Serial.printf(...) yerine LogOut.printf(...) (Print arayuzu: print/println/printf/write).
#include <Arduino.h>

class LogTee : public Print {
 public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t size) override;
  using Print::write;
};

extern LogTee LogOut;

// Halka tampon boyutu (statik, .bss). Bir satir en cok kDevLogMaxLine karakter tutulur.
static constexpr size_t kDevLogBufBytes = 10 * 1024;
static constexpr size_t kDevLogMaxLine = 240;

// seq > since olan satirlari "<seq> <metin>\n" olarak out'a yazar (kilitli kopya).
// firstSeq/nextSeq: tamponun durumu (since+1 < firstSeq ise arada satir kaybolmus demektir).
size_t devlog_read(uint32_t since, char* out, size_t outCap, uint32_t* lastSeq,
                   uint32_t* firstSeq, uint32_t* nextSeq);
