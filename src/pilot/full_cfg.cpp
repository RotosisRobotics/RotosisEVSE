#include "full_cfg.h"
#include "../log/dev_log.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>

namespace {
constexpr const char* kPrefsNs = "evsefull";
FullCfg s_cfg = {true, 600, 500, 2};
}  // namespace

static bool valid(const FullCfg& c, const char** err) {
  if (c.maxSec < 30 || c.maxSec > 7200) { if (err) *err = "sure 30-7200 sn olmali"; return false; }
  if (c.maxWh < 10 || c.maxWh > 20000) { if (err) *err = "enerji 10-20000 Wh olmali"; return false; }
  if (c.maxCycles < 1 || c.maxCycles > 10) { if (err) *err = "deneme 1-10 olmali"; return false; }
  return true;
}

void full_cfg_init() {
  Preferences p;
  if (p.begin(kPrefsNs, true)) {
    FullCfg c;
    c.enabled = p.getBool("on", true);
    c.maxSec = p.getUShort("sec", 600);
    c.maxWh = p.getUShort("wh", 500);
    c.maxCycles = p.getUChar("n", 2);
    p.end();
    if (valid(c, nullptr)) s_cfg = c;
  }
  LogOut.printf("[DOLU] Ayar: %s, kisa=%u sn / %u Wh, %u deneme\n", s_cfg.enabled ? "ACIK" : "kapali",
                (unsigned)s_cfg.maxSec, (unsigned)s_cfg.maxWh, (unsigned)s_cfg.maxCycles);
}

FullCfg full_cfg_get() { return s_cfg; }

bool full_cfg_set(const FullCfg& c, const char** err) {
  if (!valid(c, err)) return false;
  Preferences p;
  if (!p.begin(kPrefsNs, false)) {
    if (err) *err = "NVS acilamadi";
    return false;
  }
  p.putBool("on", c.enabled);
  p.putUShort("sec", c.maxSec);
  p.putUShort("wh", c.maxWh);
  p.putUChar("n", c.maxCycles);
  p.end();
  s_cfg = c;
  LogOut.printf("[DOLU] Ayar kaydedildi: %s, kisa=%u sn / %u Wh, %u deneme\n", c.enabled ? "ACIK" : "kapali",
                (unsigned)c.maxSec, (unsigned)c.maxWh, (unsigned)c.maxCycles);
  return true;
}

void full_status_json(char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) return;
  FullCfg c = s_cfg;
  int n = snprintf(out, outLen, "{\"on\":%s,\"sec\":%u,\"wh\":%u,\"n\":%u,\"locked\":%s,\"cycles\":%u}",
                   c.enabled ? "true" : "false", (unsigned)c.maxSec, (unsigned)c.maxWh, (unsigned)c.maxCycles,
                   full_runtime_locked() ? "true" : "false", (unsigned)full_runtime_cycles());
  if (n < 0 || (size_t)n >= outLen) out[0] = '\0';
}
