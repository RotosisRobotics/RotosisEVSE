#include "dev_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <time.h>

#include "log_ring.h"
#include "../net/schedule_core.h"

LogTee LogOut;

namespace {

char s_storage[kDevLogBufBytes];
evse_log::LogRing s_ring(s_storage, sizeof(s_storage), kDevLogMaxLine);

// Satir birlestirme tamponu (printf parcali yazabilir; satir '\n' gelince tampona gider).
char s_line[kDevLogMaxLine + 1];
size_t s_lineLen = 0;

StaticSemaphore_t s_mutexBuf;
SemaphoreHandle_t s_mutex = nullptr;

bool lock() {
  // ISR'den ya da zamanlayici baslamadan once kilit alinamaz: o durumda tampona yazma atlanir.
  if (xPortInIsrContext()) return false;
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return false;
  if (s_mutex == nullptr) {
    // Ilk cagri setup() icinde (tek gorev) gelir; sonrasinda yaris yok.
    s_mutex = xSemaphoreCreateMutexStatic(&s_mutexBuf);
  }
  return xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE;
}

void unlock() { xSemaphoreGive(s_mutex); }

// Kilit altinda cagrilir: birikmis satiri damgalayip halka tampona yazar.
void flushLine() {
  char out[kDevLogMaxLine + 32];
  uint32_t ms = millis();
  int n;
  time_t now = time(nullptr);
  if ((int64_t)now >= evse_sched::kMinValidEpoch) {
    // NTP UTC tutuyor; Turkiye UTC+3 (yaz saati yok).
    time_t loc = now + 3 * 3600;
    struct tm t;
    gmtime_r(&loc, &t);
    n = snprintf(out, sizeof(out), "[%lu.%03lu %02d:%02d:%02d] ", (unsigned long)(ms / 1000),
                 (unsigned long)(ms % 1000), t.tm_hour, t.tm_min, t.tm_sec);
  } else {
    n = snprintf(out, sizeof(out), "[%lu.%03lu] ", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
  }
  if (n < 0) n = 0;
  size_t room = sizeof(out) - (size_t)n;
  size_t len = s_lineLen < room ? s_lineLen : room;
  memcpy(out + n, s_line, len);
  s_ring.append(out, (size_t)n + len);
  s_lineLen = 0;
}

void feed(const uint8_t* buf, size_t size) {
  for (size_t i = 0; i < size; i++) {
    char c = (char)buf[i];
    if (c == '\n') {
      flushLine();
    } else if (c != '\r') {
      if (s_lineLen < kDevLogMaxLine) s_line[s_lineLen++] = c;
    }
  }
}

}  // namespace

size_t LogTee::write(uint8_t c) { return write(&c, 1); }

size_t LogTee::write(const uint8_t* buf, size_t size) {
  size_t w = Serial.write(buf, size);
  if (lock()) {
    feed(buf, size);
    unlock();
  }
  return w;
}

size_t devlog_read(uint32_t since, char* out, size_t outCap, uint32_t* lastSeq,
                   uint32_t* firstSeq, uint32_t* nextSeq) {
  if (!lock()) {
    if (out && outCap) out[0] = '\0';
    if (lastSeq) *lastSeq = since;
    if (firstSeq) *firstSeq = 0;
    if (nextSeq) *nextSeq = 0;
    return 0;
  }
  size_t n = s_ring.read(since, out, outCap, lastSeq);
  if (firstSeq) *firstSeq = s_ring.firstSeq();
  if (nextSeq) *nextSeq = s_ring.nextSeq();
  unlock();
  return n;
}
