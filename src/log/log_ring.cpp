#include "log_ring.h"

#include <stdio.h>
#include <string.h>

namespace evse_log {

LogRing::LogRing(char* storage, size_t capacity, size_t maxLineLen)
    : buf_(storage), cap_(capacity), max_line_(maxLineLen) {
  // Bir satir + '\n' her zaman sigmali.
  if (cap_ < 2) cap_ = 0;
  if (cap_ > 0 && max_line_ > cap_ - 1) max_line_ = cap_ - 1;
}

void LogRing::clear() {
  head_ = tail_ = used_ = 0;
  first_seq_ = next_seq_;
}

void LogRing::evictOldest() {
  // En eski satiri '\n' dahil atar.
  size_t n = 0;
  while (n < used_) {
    char c = at(n);
    n++;
    if (c == '\n') break;
  }
  tail_ = (tail_ + n) % cap_;
  used_ -= n;
  first_seq_++;
}

void LogRing::append(const char* text, size_t len) {
  if (cap_ == 0 || text == nullptr) return;
  // '\r' sayilmaz; uzunluk kirpmasi temizlenmis metne gore yapilir.
  size_t clean = 0;
  for (size_t i = 0; i < len; i++) {
    if (text[i] != '\r' && text[i] != '\0') clean++;
  }
  if (clean > max_line_) clean = max_line_;
  size_t need = clean + 1;
  while (cap_ - used_ < need && used_ > 0) evictOldest();

  size_t written = 0;
  for (size_t i = 0; i < len && written < clean; i++) {
    char c = text[i];
    if (c == '\r' || c == '\0') continue;
    if (c == '\n') c = ' ';
    buf_[head_] = c;
    head_ = (head_ + 1) % cap_;
    written++;
  }
  buf_[head_] = '\n';
  head_ = (head_ + 1) % cap_;
  used_ += need;
  next_seq_++;
}

size_t LogRing::read(uint32_t since, char* out, size_t outCap, uint32_t* lastSeq) const {
  uint32_t last = (since >= first_seq_) ? since : first_seq_ - 1;
  if (last >= next_seq_) last = next_seq_ - 1;
  size_t pos = 0;
  if (out == nullptr || outCap == 0) {
    if (lastSeq) *lastSeq = last;
    return 0;
  }
  out[0] = '\0';

  uint32_t seq = first_seq_;
  size_t off = 0;
  while (off < used_) {
    // Satir sonunu bul.
    size_t end = off;
    while (end < used_ && at(end) != '\n') end++;
    size_t lineLen = end - off;
    if (seq > since) {
      char num[12];
      int nlen = snprintf(num, sizeof(num), "%lu ", (unsigned long)seq);
      if (nlen < 0) nlen = 0;
      size_t need = (size_t)nlen + lineLen + 1;
      if (pos + need + 1 > outCap) break;  // +1: sondaki '\0'
      memcpy(out + pos, num, (size_t)nlen);
      pos += (size_t)nlen;
      for (size_t i = 0; i < lineLen; i++) out[pos++] = at(off + i);
      out[pos++] = '\n';
      last = seq;
    }
    off = end + 1;
    seq++;
  }
  out[pos] = '\0';
  if (lastSeq) *lastSeq = last;
  return pos;
}

}  // namespace evse_log
