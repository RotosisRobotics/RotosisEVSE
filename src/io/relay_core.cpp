#include "relay_core.h"

namespace evse_relay {

const char* off_reason_name(OffReason r) {
  switch (r) {
    case OffReason::Emergency:   return "acil (A/E/F)";
    case OffReason::ZeroCurrent: return "akim sifir";
    case OffReason::Timeout:     return "zaman asimi";
    default:                     return "-";
  }
}

bool should_close(char st, bool pwmOn, int dutyPercent) {
  return (st == 'C' || st == 'D') && pwmOn && dutyPercent >= kMinChargeDuty && dutyPercent <= kMaxChargeDuty;
}

OffReason off_decision(char st, float currentA, uint32_t elapsedMs, uint32_t confirmMs) {
  // Arac yok / CP hatasi: beklemeden ac (IEC 61851-1: A, E, F'de kontaktor acik olmali).
  if (st != 'B' && st != 'C' && st != 'D') return OffReason::Emergency;
  bool zero = (currentA >= 0.0f && currentA < kZeroCurrentA);
  if (zero && elapsedMs >= confirmMs) return OffReason::ZeroCurrent;
  // B: arac S2'yi acti, akimi kesmis olmali -> kisa bekleme. C/D: PWM kesildi, arac henuz birakmadi.
  uint32_t timeout = (st == 'B') ? kOffLoadTimeoutBMs : kOffSoftStopTimeoutMs;
  if (elapsedMs >= timeout) return OffReason::Timeout;
  return OffReason::None;
}

}  // namespace evse_relay
