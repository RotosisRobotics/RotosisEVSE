#include "wake_core.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace evse_wake {

Config default_config() { return Config(); }

bool validate(const Config& c, const char** err) {
  const char* dummy;
  if (err == nullptr) err = &dummy;
  *err = "";
  if (c.seqLen < 1 || c.seqLen > kMaxSeq) { *err = "Yontem listesi 1-6 eleman olmali"; return false; }
  for (uint8_t i = 0; i < c.seqLen; ++i) {
    if ((uint8_t)c.seq[i] > (uint8_t)Method::Pwm5) { *err = "Gecersiz yontem"; return false; }
  }
  if (c.maxAttempts < 1 || c.maxAttempts > 10) { *err = "Deneme sayisi 1-10 olmali"; return false; }
  if (c.waitMs < 5000 || c.waitMs > 600000) { *err = "Bekleme 5-600 sn olmali"; return false; }
  if (c.b1Ms < 500 || c.b1Ms > 10000) { *err = "B1 darbe suresi 0,5-10 sn olmali"; return false; }
  if (c.fMs < 500 || c.fMs > 10000) { *err = "F darbe suresi 0,5-10 sn olmali"; return false; }
  if (c.pwm5Ms < 500 || c.pwm5Ms > 10000) { *err = "%5 PWM suresi 0,5-10 sn olmali"; return false; }
  if (c.efMs < 500 || c.efMs > 5000) { *err = "E/F suresi 0,5-5 sn olmali"; return false; }
  return true;
}

const char* method_name(Method m) {
  switch (m) {
    case Method::B1Pulse:   return "B1";
    case Method::FPulse:    return "F";
    case Method::Pwm5: return "PWM5";
  }
  return "?";
}

bool parse_seq(const char* s, Config* c) {
  if (s == nullptr || c == nullptr) return false;
  Method tmp[kMaxSeq];
  uint8_t n = 0;
  const char* p = s;
  for (;;) {
    while (*p == ' ') ++p;
    char tok[8];
    size_t k = 0;
    while (*p && *p != ',' && *p != ' ') {
      if (k >= sizeof(tok) - 1) return false;
      tok[k++] = (char)toupper((unsigned char)*p++);
    }
    tok[k] = '\0';
    while (*p == ' ') ++p;
    if (k == 0) return false;
    if (n >= kMaxSeq) return false;
    if (strcmp(tok, "B1") == 0) tmp[n++] = Method::B1Pulse;
    else if (strcmp(tok, "F") == 0) tmp[n++] = Method::FPulse;
    else if (strcmp(tok, "PWM5") == 0) tmp[n++] = Method::Pwm5;
    else return false;
    if (*p == '\0') break;
    if (*p != ',') return false;
    ++p;
  }
  c->seqLen = n;
  for (uint8_t i = 0; i < kMaxSeq; ++i) c->seq[i] = (i < n) ? tmp[i] : tmp[n - 1];
  return true;
}

void format_seq(const Config& c, char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) return;
  out[0] = '\0';
  size_t n = 0;
  for (uint8_t i = 0; i < c.seqLen && i < kMaxSeq; ++i) {
    int k = snprintf(out + n, outLen - n, "%s%s", i ? "," : "", method_name(c.seq[i]));
    if (k < 0 || (size_t)k >= outLen - n) { out[n] = '\0'; return; }
    n += (size_t)k;
  }
}

Method method_for_attempt(const Config& c, uint8_t attempt1) {
  uint8_t len = (c.seqLen >= 1 && c.seqLen <= kMaxSeq) ? c.seqLen : 1;
  uint8_t idx = (attempt1 == 0) ? 0 : (uint8_t)(attempt1 - 1);
  if (idx >= len) idx = (uint8_t)(len - 1);
  return c.seq[idx];
}

static void put32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static constexpr uint8_t kBlobVer = 1;

void serialize(const Config& c, uint8_t out[kBlobLen]) {
  memset(out, 0, kBlobLen);
  out[0] = kBlobVer;
  out[1] = c.enabled ? 1 : 0;
  out[2] = c.maxAttempts;
  out[3] = c.seqLen;
  for (uint8_t i = 0; i < kMaxSeq; ++i) out[4 + i] = (uint8_t)c.seq[i];
  put32(out + 10, c.waitMs);
  put32(out + 14, c.b1Ms);
  put32(out + 18, c.fMs);
  put32(out + 22, c.pwm5Ms);
  put32(out + 26, c.efMs);
}

bool deserialize(const uint8_t* in, size_t len, Config* out) {
  if (in == nullptr || out == nullptr || len != kBlobLen || in[0] != kBlobVer || in[1] > 1) return false;
  Config c;
  c.enabled = in[1] != 0;
  c.maxAttempts = in[2];
  c.seqLen = in[3];
  for (uint8_t i = 0; i < kMaxSeq; ++i) {
    if (in[4 + i] > (uint8_t)Method::Pwm5) return false;
    c.seq[i] = (Method)in[4 + i];
  }
  c.waitMs = get32(in + 10);
  c.b1Ms = get32(in + 14);
  c.fMs = get32(in + 18);
  c.pwm5Ms = get32(in + 22);
  c.efMs = get32(in + 26);
  if (!validate(c, nullptr)) return false;
  *out = c;
  return true;
}

const char* phase_name(Phase p) {
  switch (p) {
    case Phase::Idle:    return "Idle";
    case Phase::Waiting: return "Waiting";
    case Phase::PulseB1: return "PulseB1";
    case Phase::PulseF:  return "PulseF";
    case Phase::Pwm5:    return "Pwm5";
    case Phase::StepEF:  return "StepEF";
    case Phase::Settle:  return "Settle";
  }
  return "?";
}

uint32_t Machine::reset_() {
  uint32_t n = attempts_;
  phase_ = Phase::Idle;
  attempts_ = 0;
  unsafeLogged_ = false;
  return n;
}

Event Machine::set_config(const Config& c) {
  cfg_ = c;
  if (phase_ == Phase::Idle) return Event::None;
  reset_();
  return Event::ResetConfig;
}

void Machine::on_unplug() {
  reset_();
  bNoPwmSince_ = 0;
}

Output Machine::step(const Input& in) {
  Output o;
  const uint32_t now = in.now;

  if (!cfg_.enabled) {
    prevPwmWant_ = in.pwmWant;
    bNoPwmSince_ = 0;
    if (phase_ != Phase::Idle) { o.info = reset_(); o.ev = Event::ResetConfig; }
    return o;
  }

  bool edge = in.pwmWant && !prevPwmWant_;
  prevPwmWant_ = in.pwmWant;

  if (in.st != 'B') {
    bNoPwmSince_ = 0;
    if (phase_ != Phase::Idle) {
      o.info = reset_();
      o.ev = (in.st == 'C' || in.st == 'D') ? Event::ResetCharging : Event::ResetLeftB;
    }
    return o;
  }
  if (!in.pwmWant) {
    if (bNoPwmSince_ == 0) bNoPwmSince_ = (now != 0) ? now : 1;
    if (phase_ != Phase::Idle) { o.info = reset_(); o.ev = Event::ResetNoPermit; }
    return o;
  }
  if (edge) {
    uint32_t idle = (bNoPwmSince_ != 0) ? (now - bNoPwmSince_) : 0;
    bNoPwmSince_ = 0;
    if (idle >= kArmMinIdleBMs) {
      phase_ = Phase::Waiting;
      attempts_ = 0;
      unsafeLogged_ = false;
      t0_ = now;
      o.ev = Event::Armed;
      o.info = idle;
    } else if (idle > 0) {
      o.ev = Event::NotArmed;
      o.info = idle;
    }
    return o;
  }

  const uint32_t el = now - t0_;
  switch (phase_) {
    case Phase::Idle:
      return o;

    case Phase::Waiting:
      if (el < cfg_.waitMs) return o;
      if (attempts_ >= cfg_.maxAttempts) {
        o.info = reset_();
        o.ev = Event::GaveUp;
        return o;
      }
      // Guvenlik: kontaktor kesin acik ve akim yok olmadan CP'ye dokunma.
      if (!in.contactsOpen || in.currentA > kMaxIdleCurrentA) {
        if (!unsafeLogged_) { o.ev = Event::Deferred; unsafeLogged_ = true; }
        t0_ = now;
        return o;
      }
      attempts_++;
      method_ = method_for_attempt(cfg_, attempts_);
      t0_ = now;
      o.ev = Event::PulseStart;
      if (method_ == Method::B1Pulse) {
        phase_ = Phase::PulseB1;
        o.pwmOff = true;
      } else if (method_ == Method::FPulse) {
        phase_ = Phase::PulseF;
        o.pwmOff = true;
        o.forceCpLow = true;
      } else {
        phase_ = Phase::Pwm5;
        o.dutyOverride = kPwm5Duty;
      }
      return o;

    case Phase::PulseB1:
      if (el < cfg_.b1Ms) { o.pwmOff = true; return o; }
      o.ev = Event::PulseEnd;
      o.info = el;
      phase_ = Phase::Waiting;
      t0_ = now;
      return o;

    case Phase::PulseF:
      if (el < cfg_.fMs) { o.pwmOff = true; o.forceCpLow = true; return o; }
      o.ev = Event::PulseEnd;
      o.info = el;
      phase_ = Phase::Settle;
      t0_ = now;
      return o;

    case Phase::Pwm5:
      if (el < cfg_.pwm5Ms) { o.dutyOverride = kPwm5Duty; return o; }
      o.ev = Event::StepEF;
      o.info = el;
      phase_ = Phase::StepEF;
      t0_ = now;
      o.pwmOff = true;
      o.forceCpLow = true;
      return o;

    case Phase::StepEF:
      if (el < cfg_.efMs) { o.pwmOff = true; o.forceCpLow = true; return o; }
      o.ev = Event::PulseEnd;
      o.info = el;
      phase_ = Phase::Settle;
      t0_ = now;
      return o;

    case Phase::Settle:
      if (el >= kSettleMs) {
        phase_ = Phase::Waiting;
        t0_ = now;
        o.ev = Event::SettleEnd;
      }
      return o;
  }
  return o;
}

}  // namespace evse_wake
