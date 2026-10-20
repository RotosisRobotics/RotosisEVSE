#include "wake_cfg.h"
#include "../log/dev_log.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>

using namespace evse_wake;

namespace {
constexpr const char* kPrefsNs = "evsewake";
Config s_cfg = default_config();
uint32_t s_gen = 1;
}  // namespace

void wake_cfg_init() {
  Config loaded = default_config();
  Preferences p;
  if (p.begin(kPrefsNs, true)) {
    uint8_t blob[kBlobLen];
    if (p.getBytesLength("cfg") == kBlobLen && p.getBytes("cfg", blob, sizeof(blob)) == kBlobLen) {
      Config tmp;
      if (deserialize(blob, sizeof(blob), &tmp)) loaded = tmp;
    }
    p.end();
  }
  s_cfg = loaded;
  s_gen++;
  char seq[40];
  format_seq(loaded, seq, sizeof(seq));
  LogOut.printf("[WAKE] Ayar: %s, yontem=%s, en cok %u deneme, bekleme %lu ms, B1 %lu ms, F %lu ms, PWM5 %lu+%lu ms\n",
                loaded.enabled ? "ACIK" : "kapali", seq, (unsigned)loaded.maxAttempts, (unsigned long)loaded.waitMs,
                (unsigned long)loaded.b1Ms, (unsigned long)loaded.fMs, (unsigned long)loaded.pwm5Ms,
                (unsigned long)loaded.efMs);
}

Config wake_cfg_get() { return s_cfg; }
uint32_t wake_cfg_gen() { return s_gen; }

bool wake_cfg_set(const Config& c, const char** err) {
  if (!validate(c, err)) return false;
  uint8_t blob[kBlobLen];
  serialize(c, blob);
  Preferences p;
  if (!p.begin(kPrefsNs, false)) {
    if (err) *err = "NVS acilamadi";
    return false;
  }
  size_t w = p.putBytes("cfg", blob, sizeof(blob));
  p.end();
  if (w != sizeof(blob)) {
    if (err) *err = "NVS yazilamadi";
    return false;
  }
  s_cfg = c;
  s_gen++;
  char seq[40];
  format_seq(c, seq, sizeof(seq));
  LogOut.printf("[WAKE] Ayar kaydedildi: %s, yontem=%s, en cok %u, bekleme %lu ms, B1 %lu ms, F %lu ms, PWM5 %lu+%lu ms\n",
                c.enabled ? "ACIK" : "kapali", seq, (unsigned)c.maxAttempts, (unsigned long)c.waitMs,
                (unsigned long)c.b1Ms, (unsigned long)c.fMs, (unsigned long)c.pwm5Ms, (unsigned long)c.efMs);
  return true;
}

void wake_status_json(char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) return;
  Config c = s_cfg;
  char seq[40];
  format_seq(c, seq, sizeof(seq));
  int n = snprintf(out, outLen,
                   "{\"on\":%s,\"seq\":\"%s\",\"max\":%u,\"waitMs\":%lu,\"b1Ms\":%lu,\"fMs\":%lu,\"pwm5Ms\":%lu,"
                   "\"efMs\":%lu,\"phase\":\"%s\",\"attempt\":%u}",
                   c.enabled ? "true" : "false", seq, (unsigned)c.maxAttempts, (unsigned long)c.waitMs,
                   (unsigned long)c.b1Ms, (unsigned long)c.fMs, (unsigned long)c.pwm5Ms, (unsigned long)c.efMs,
                   wake_runtime_phase(), (unsigned)wake_runtime_attempts());
  if (n < 0 || (size_t)n >= outLen) out[0] = '\0';
}
