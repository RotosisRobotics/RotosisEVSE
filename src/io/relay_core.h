#pragma once
// Kontaktor karar mantigi (donanimdan bagimsiz; native testli: test/test_relay).
// relay.cpp bu fonksiyonlarla "simdi kapat / ac" kararini verir.
#include <stdint.h>

namespace evse_relay {

// Akim bu degerin altindaysa "sifir" sayilir (kontaktor yuk altinda acilmaz).
constexpr float kZeroCurrentA = 0.5f;
// Arac C->B yaptiginda (S2 acti) akim hala gorunuyorsa en cok bu kadar beklenir, sonra acilir.
// Eski sabit gecikme (relayOffDelayMs=500) ile ayni: en kotu durum eskisinden yavas degil.
constexpr uint32_t kOffLoadTimeoutBMs = 500;
// PWM kesildi (STOP / izin kalkti) ama arac hala C/D'de: yumusak durdurma ust siniri.
// evse_cloud::kSoftStopMaxWaitMs ile ayni olmali (cloud_client.cpp static_assert).
constexpr uint32_t kOffSoftStopTimeoutMs = 3000;
// Kontaktor kapatmak icin gereken en dusuk duty (%5 = dijital iletisim, akim izni degil).
constexpr int kMinChargeDuty = 8;
constexpr int kMaxChargeDuty = 97;

enum class OffReason : uint8_t {
  None = 0,     // henuz acma
  Emergency,    // A / E / F / bilinmeyen durum: hemen ac
  ZeroCurrent,  // akim ~0 (ve onay suresi doldu)
  Timeout,      // akim sifirlanmadi ama bekleme suresi doldu
};

const char* off_reason_name(OffReason r);

// Kontaktor kapali olmali mi? (durum C/D + PWM acik + sarj duty'si)
bool should_close(char st, bool pwmOn, int dutyPercent);

// Panel /relay?on=1 ve /pulse_set: kontaktor yalniz should_close iken elle kapatilabilir.
inline bool manual_close_allowed(char st, bool pwmOn, int dutyPercent) {
  return should_close(st, pwmOn, dutyPercent);
}

// Kontaktor acilmak isteniyor; simdi acilabilir mi?
// st: kararli CP durumu, currentA: anlik (filtresiz) en buyuk faz akimi; < 0 = bilinmiyor,
// elapsedMs: acma isteginden beri gecen sure, confirmMs: akim sifirken ek onay (relayOffDelayMs).
OffReason off_decision(char st, float currentA, uint32_t elapsedMs, uint32_t confirmMs);

}  // namespace evse_relay
