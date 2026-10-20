#include "relay.h"
#include "relay_core.h"
#include "current_sensor.h"
#include "../log/dev_log.h"
#include "../pilot/pilot.h"
#include "app_pins.h"
#include <Arduino.h>

// Role kontrol modulu.
// Web panelinden ayarlanan on/off gecikmeleri burada uygulanir.

// Web arayuzunden gelen gecikme degerleri.
uint32_t relayOnDelayMs  = 1000; // Varsayılan 1 saniye (B -> C)
// Acma (C -> B): akim ~0 olcuyse ek onay suresi. Akim varken relay_core zaman asimlari gecerli
// (B: 500 ms, PWM kesik C/D: 3 sn). Eski sabit 500 ms gecikme yerine (IEC 61851-1: C->B < 100 ms).
uint32_t relayOffDelayMs = 0;

static bool relayOn = false;
static bool relayAutoEnabled = true;

static uint32_t minSwitchMs = 0;
static uint32_t lastSwitchMs = 0;

// Zamanlayıcı için yardımcı değişkenler (yalniz kapatma/AÇMA yonu)
static uint32_t stateChangeStartTime = 0;
static bool waitingForChange = false;
static bool pendingTargetState = false;

// Guvenli acma istegi: 0 = beklemiyor.
static uint32_t s_offSinceMs = 0;
static float s_quickI = -1.0f;
static uint32_t s_quickAtMs = 0;
static constexpr uint32_t kQuickMeasureMinGapMs = 40;

#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
static bool lastLatchSetState = false;
static bool latchStateReady = false;
static bool latchWaitingForPulse = false;
static bool pendingLatchSetState = false;
static uint32_t latchStateChangeStartMs = 0;
#endif

static char stateChar(const String& s) { return s.length() > 0 ? s[0] : 'F'; }

static void writeRelayPin()
{
#if RELAY_USE_OUTPUT_PIN
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, relayOn ? LOW : HIGH);
#else
  digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);
#endif
#endif
}

static void pulseLatchPin(uint8_t pin)
{
#if RELAY_USE_LATCH_PINS
  digitalWrite(pin, HIGH);
  delay(RELAY_LATCH_PULSE_MS);
  digitalWrite(pin, LOW);
#else
  (void)pin;
#endif
}

static void applyRelayOutputs(bool pulseLatch)
{
  writeRelayPin();

#if RELAY_USE_LATCH_PINS
  if (pulseLatch && !RELAY_LATCH_FOLLOW_STATE) {
    pulseLatchPin(relayOn ? MOSFET_SET_PIN : MOSFET_RESET_PIN);
  }
#else
  (void)pulseLatch;
#endif
}

void relay_init()
{
#if RELAY_USE_OUTPUT_PIN
  pinMode(RELAY_PIN, OUTPUT);
#endif
#if RELAY_USE_LATCH_PINS
  pinMode(MOSFET_RESET_PIN, OUTPUT);
  pinMode(MOSFET_SET_PIN, OUTPUT);
  digitalWrite(MOSFET_RESET_PIN, LOW);
  digitalWrite(MOSFET_SET_PIN, LOW);
#endif
  relayOn = false;
  applyRelayOutputs(false);
}
// Role cikislarini cok sik ac/kapa yapmamak icin alt koruma (yalniz kapatma yonunde).
void relay_set_min_switch_ms(uint32_t ms)
{
  minSwitchMs = ms;
}
void relay_set_auto_enabled(bool en) { relayAutoEnabled = en; }
bool relay_is_auto_enabled() { return relayAutoEnabled; }
bool relay_get() { return relayOn; }

bool relay_contacts_open()
{
  // Role cikisi birakik, bekleyen acma gecisi yok ve (latch donanimda) son pulse RESET ise true.
  if (relayOn) return false;
  if (waitingForChange && pendingTargetState) return false;
#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  if (lastLatchSetState) return false;
  if (latchWaitingForPulse && pendingLatchSetState) return false;
#endif
  return true;
}

// Bekleyen kapatma (SET) gecislerini iptal et.
static void cancelPendingClose()
{
  if (waitingForChange && pendingTargetState) {
    waitingForChange = false;
    LogOut.println("[ROLE] Bekleyen kapatma iptal (kosul kalkti)");
  }
#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  if (latchWaitingForPulse && pendingLatchSetState) latchWaitingForPulse = false;
#endif
}

static bool contactorClosedAny()
{
  if (relayOn) return true;
#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  if (lastLatchSetState) return true;
#endif
  return false;
}

void relay_force_off_now()
{
  relayOn = false;
  waitingForChange = false;
  pendingTargetState = false;
  writeRelayPin();

#if RELAY_USE_LATCH_PINS
  pulseLatchPin(MOSFET_RESET_PIN);
#endif

#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  latchWaitingForPulse = false;
  pendingLatchSetState = false;
  lastLatchSetState = false;
  latchStateReady = true;
  latchStateChangeStartMs = millis();
#endif

  lastSwitchMs = millis();
  s_offSinceMs = 0;
}

// Kontaktoru ac: role cikisi + (kapaliysa) latch RESET.
static void openContactor(evse_relay::OffReason why, char st, float iA, uint32_t waitedMs, const char* src)
{
  relayOn = false;
  waitingForChange = false;
  pendingTargetState = false;
  writeRelayPin();
#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  if (lastLatchSetState || !latchStateReady) pulseLatchPin(MOSFET_RESET_PIN);
  lastLatchSetState = false;
  latchWaitingForPulse = false;
  pendingLatchSetState = false;
  latchStateReady = true;
  latchStateChangeStartMs = millis();
#elif RELAY_USE_LATCH_PINS
  pulseLatchPin(MOSFET_RESET_PIN);
#endif
  lastSwitchMs = millis();
  s_offSinceMs = 0;
  LogOut.printf(">>> RÖLE GÜNCELLENDİ: KAPALI [%s] neden=%s st=%c I=%.2f A bekleme=%lu ms\n",
                src, evse_relay::off_reason_name(why), st, iA, (unsigned long)waitedMs);
}

// Guvenli acma: true donerse kontaktor acik.
static bool tryOpenGated(char st, uint32_t now, const char* src)
{
  if (!contactorClosedAny()) {
    s_offSinceMs = 0;
    return true;
  }
  if (s_offSinceMs == 0) {
    s_offSinceMs = (now != 0) ? now : 1;
    s_quickAtMs = 0;
    LogOut.printf("Röle geçişi bekleniyor: KAPATMA [%s] st=%c PWM=%s (akim sifirlaninca)\n",
                  src, st, pwmEnabled ? "ON" : "OFF");
  }
  uint32_t el = now - s_offSinceMs;
  float iA = -1.0f;
  bool emergency = (st != 'B' && st != 'C' && st != 'D');
  if (!emergency) {
    if (s_quickAtMs == 0 || (now - s_quickAtMs) >= kQuickMeasureMinGapMs) {
      s_quickI = current_sensor_quick_max_a();
      s_quickAtMs = (now != 0) ? now : 1;
    }
    iA = s_quickI;
  }
  evse_relay::OffReason r = evse_relay::off_decision(st, iA, el, relayOffDelayMs);
  if (r == evse_relay::OffReason::None) return false;
  // Asiri sik anahtarlama korumasi yalniz acil olmayan durumda.
  if (r != evse_relay::OffReason::Emergency && minSwitchMs > 0 && (now - lastSwitchMs) < minSwitchMs) return false;
  openContactor(r, st, iA, el, src);
  return true;
}

void relay_set(bool on)
{
  relayAutoEnabled = false; // Manuel müdahalede otomatiği kapat
  if (relayOn == on) return;
  relayOn = on;
  applyRelayOutputs(true);
  lastSwitchMs = millis();
  if (on) s_offSinceMs = 0;
}

void relay_update_auto(const String& stableState, bool pwmEnabled, int pwmDutyPercent)
{
  char st = stateChar(stableState);
  bool shouldBeOn = evse_relay::should_close(st, pwmEnabled, pwmDutyPercent);
  uint32_t now = millis();

  if (!relayAutoEnabled) {
    // Manuel mod: elle kapatilan kontaktor de sarj kosulu kalkinca guvenli yoldan acilir.
    if (!shouldBeOn && contactorClosedAny()) tryOpenGated(st, now, "manuel");
    return;
  }

  if (!shouldBeOn) {
    cancelPendingClose();
    if (contactorClosedAny()) tryOpenGated(st, now, "auto");
    else s_offSinceMs = 0;
    return;
  }

  s_offSinceMs = 0;
  if (relayOn) {
    waitingForChange = false;
    return;
  }

  // Durum değişikliği ilk kez algılandığında zamanlayıcıyı başlat
  if (!waitingForChange || !pendingTargetState) {
    stateChangeStartTime = now;
    pendingTargetState = true;
    waitingForChange = true;
    LogOut.println("Röle geçişi bekleniyor: AÇMA (Gecikme uygulanıyor...)");
  }

  if ((now - stateChangeStartTime) >= relayOnDelayMs) {
    // Role donanimi icin minimum anahtarlama araligi korumasi.
    if (minSwitchMs > 0 && (now - lastSwitchMs) < minSwitchMs) return;
    relayOn = true;
    applyRelayOutputs(true);
    lastSwitchMs = millis();
    waitingForChange = false;
    LogOut.printf(">>> RÖLE GÜNCELLENDİ: AÇIK\n");
  }
}

void relay_fast_guard(char st)
{
  if (st == 'C' || st == 'D') return;  // sarj durumunda karar yavas dongude (PWM/izin)
  cancelPendingClose();
  if (!contactorClosedAny()) {
    s_offSinceMs = 0;
    return;
  }
  tryOpenGated(st, millis(), "hizli");
}

void relay_handle_state_pulse(const String& stableState, bool pwmEnabled, int pwmDutyPercent)
{
#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  // Latch SET karari: durum C/D + PWM acik + sarj duty'si, relayOnDelayMs sonra.
  // RESET (acma) relay_update_auto / relay_fast_guard guvenli acma yolundadir.
  bool want = evse_relay::should_close(stateChar(stableState), pwmEnabled, pwmDutyPercent);

  if (!latchStateReady) {
    lastLatchSetState = false;
    latchStateReady = true;
  }
  if (!want) {
    if (latchWaitingForPulse && pendingLatchSetState) latchWaitingForPulse = false;
    return;
  }
  if (lastLatchSetState) {
    latchWaitingForPulse = false;
    return;
  }
  if (!latchWaitingForPulse || !pendingLatchSetState) {
    pendingLatchSetState = true;
    latchStateChangeStartMs = millis();
    latchWaitingForPulse = true;
  }
  if ((millis() - latchStateChangeStartMs) < relayOnDelayMs) return;

  pulseLatchPin(MOSFET_SET_PIN);
  lastLatchSetState = true;
  latchWaitingForPulse = false;
  s_offSinceMs = 0;
#else
  (void)stableState;
  (void)pwmEnabled;
  (void)pwmDutyPercent;
#endif
}

void relay_latch_manual_set()
{
#if RELAY_USE_LATCH_PINS
  pulseLatchPin(MOSFET_SET_PIN);
#if RELAY_LATCH_FOLLOW_STATE
  lastLatchSetState = true;
  latchWaitingForPulse = false;
  latchStateReady = true;
#endif
  lastSwitchMs = millis();
  s_offSinceMs = 0;
#endif
}

void relay_latch_manual_reset()
{
#if RELAY_USE_LATCH_PINS
  pulseLatchPin(MOSFET_RESET_PIN);
#if RELAY_LATCH_FOLLOW_STATE
  lastLatchSetState = false;
  latchWaitingForPulse = false;
  latchStateReady = true;
#endif
  lastSwitchMs = millis();
#endif
}
