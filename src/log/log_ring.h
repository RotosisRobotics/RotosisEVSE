#pragma once
// Satir tabanli sabit boyutlu halka tampon (donanimdan bagimsiz, native testte de derlenir).
// - Depolama disaridan verilir (statik dizi): heap kullanmaz.
// - Her satira 1'den baslayan artan sira numarasi (seq) verilir; seq tamponda saklanmaz,
//   en eski satirin numarasindan (firstSeq) sayilarak bulunur.
// - Yer kalmazsa en eski satirlar atilir. Satir ici '\n' bosluga cevrilir, '\r' atilir.
// - Kilit YOK: cok gorevli kullanimda cagiran taraf kilitlemelidir (bkz. dev_log.cpp).
#include <stddef.h>
#include <stdint.h>

namespace evse_log {

class LogRing {
 public:
  LogRing(char* storage, size_t capacity, size_t maxLineLen = 256);

  // Bir satir ekler (sonundaki '\n' gerekmez). Cok uzunsa maxLineLen'e kirpilir.
  void append(const char* text, size_t len);
  void clear();

  // En eski tamponda duran satirin numarasi (bos ise nextSeq()).
  uint32_t firstSeq() const { return first_seq_; }
  // Bir sonraki eklenecek satirin numarasi (son satir = nextSeq()-1).
  uint32_t nextSeq() const { return next_seq_; }
  size_t lineCount() const { return next_seq_ - first_seq_; }
  size_t usedBytes() const { return used_; }
  size_t capacity() const { return cap_; }

  // seq > since olan satirlari "<seq> <metin>\n" bicimiyle out'a yazar (sonuna '\0' koyar).
  // Yer biterse yarim satir yazmaz, durur. *lastSeq: yazilan son satirin numarasi
  // (hic satir yazilmadiysa since ya da firstSeq-1'den buyuk olani). Donus: yazilan bayt (\0 haric).
  size_t read(uint32_t since, char* out, size_t outCap, uint32_t* lastSeq) const;

 private:
  void evictOldest();
  char at(size_t logicalOffsetFromTail) const { return buf_[(tail_ + logicalOffsetFromTail) % cap_]; }

  char* buf_;
  size_t cap_;
  size_t max_line_;
  size_t head_ = 0;  // yazma konumu
  size_t tail_ = 0;  // en eski satirin basi
  size_t used_ = 0;
  uint32_t first_seq_ = 1;
  uint32_t next_seq_ = 1;
};

}  // namespace evse_log
