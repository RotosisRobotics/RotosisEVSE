// Arac uyandirma durum makinesi (src/pilot/wake_core) birim testleri: pio test -e native
#include <string.h>
#include <unity.h>

#include "pilot/wake_core.h"

using namespace evse_wake;

#define EQ_E(a, b) TEST_ASSERT_EQUAL_INT((int)(a), (int)(b))

void setUp() {}
void tearDown() {}

// Yardimci: B'de PWM'siz bekleme (t0..t0+idle) ve PWM acilisi; donus = acilis zamani.
static uint32_t armAfterIdle(Machine& w, uint32_t t0, uint32_t idleMs) {
  t0 += 1000;  // millis()=0 "zaman yok" sayilir (makine 1'e yuvarlar); gercek cihazda boot sonrasi > 0
  w.step({'B', false, true, 0.0f, t0});
  w.step({'B', false, true, 0.0f, t0 + idleMs});
  Output o = w.step({'B', true, true, 0.0f, t0 + idleMs});
  EQ_E(Event::Armed, o.ev);
  return t0 + idleMs;
}

void test_defaults_match_previous_behavior() {
  Config c = default_config();
  TEST_ASSERT_TRUE(c.enabled);
  TEST_ASSERT_EQUAL_UINT8(1, c.seqLen);
  EQ_E(Method::B1Pulse, c.seq[0]);
  TEST_ASSERT_EQUAL_UINT8(3, c.maxAttempts);
  TEST_ASSERT_EQUAL_UINT32(25000, c.waitMs);
  TEST_ASSERT_EQUAL_UINT32(4000, c.b1Ms);
  TEST_ASSERT_TRUE(validate(c, nullptr));
}

void test_parse_format_seq() {
  Config c;
  TEST_ASSERT_TRUE(parse_seq("b1, B1 ,f", &c));
  TEST_ASSERT_EQUAL_UINT8(3, c.seqLen);
  char buf[40];
  format_seq(c, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("B1,B1,F", buf);
  EQ_E(Method::B1Pulse, method_for_attempt(c, 1));
  EQ_E(Method::B1Pulse, method_for_attempt(c, 2));
  EQ_E(Method::FPulse, method_for_attempt(c, 3));
  EQ_E(Method::FPulse, method_for_attempt(c, 7));  // liste bitince son yontem
  TEST_ASSERT_TRUE(parse_seq("PWM5", &c));
  EQ_E(Method::Pwm5, method_for_attempt(c, 2));
  Config before = c;
  TEST_ASSERT_FALSE(parse_seq("", &c));
  TEST_ASSERT_FALSE(parse_seq("B1,,F", &c));
  TEST_ASSERT_FALSE(parse_seq("B2", &c));
  TEST_ASSERT_FALSE(parse_seq("B1,F,B1,F,B1,F,B1", &c));  // > 6
  TEST_ASSERT_FALSE(parse_seq("B1,", &c));
  TEST_ASSERT_EQUAL_UINT8(before.seqLen, c.seqLen);
}

void test_validate_ranges() {
  Config c;
  const char* err = nullptr;
  c.waitMs = 1000;
  TEST_ASSERT_FALSE(validate(c, &err));
  c = Config();
  c.maxAttempts = 0;
  TEST_ASSERT_FALSE(validate(c, &err));
  c = Config();
  c.efMs = 6000;
  TEST_ASSERT_FALSE(validate(c, &err));
  c = Config();
  c.fMs = 3000;
  c.efMs = 1000;
  TEST_ASSERT_TRUE(validate(c, &err));
}

void test_blob_roundtrip() {
  Config c;
  parse_seq("B1,B1,F", &c);
  c.enabled = false;
  c.maxAttempts = 5;
  c.waitMs = 30000;
  c.fMs = 2500;
  c.efMs = 1500;
  uint8_t b[kBlobLen];
  serialize(c, b);
  Config d;
  TEST_ASSERT_TRUE(deserialize(b, sizeof(b), &d));
  TEST_ASSERT_FALSE(d.enabled);
  TEST_ASSERT_EQUAL_UINT8(3, d.seqLen);
  EQ_E(Method::FPulse, d.seq[2]);
  TEST_ASSERT_EQUAL_UINT8(5, d.maxAttempts);
  TEST_ASSERT_EQUAL_UINT32(30000, d.waitMs);
  TEST_ASSERT_EQUAL_UINT32(2500, d.fMs);
  TEST_ASSERT_EQUAL_UINT32(1500, d.efMs);
  b[0] = 99;  // bilinmeyen surum
  TEST_ASSERT_FALSE(deserialize(b, sizeof(b), &d));
  serialize(c, b);
  b[10] = 0; b[11] = 0; b[12] = 0; b[13] = 0;  // waitMs = 0 (gecersiz)
  TEST_ASSERT_FALSE(deserialize(b, sizeof(b), &d));
  TEST_ASSERT_FALSE(deserialize(b, kBlobLen - 1, &d));
}

void test_not_armed_short_idle() {
  Machine w;
  w.step({'B', false, true, 0.0f, 1000});
  Output o = w.step({'B', true, true, 0.0f, 5000});
  EQ_E(Event::NotArmed, o.ev);
  TEST_ASSERT_FALSE(w.active());
}

void test_b1_default_sequence() {
  Machine w;
  uint32_t t = armAfterIdle(w, 1000, kArmMinIdleBMs);
  Output o = w.step({'B', true, true, 0.0f, t + 24999});
  TEST_ASSERT_FALSE(o.pwmOff);
  o = w.step({'B', true, true, 0.0f, t + 25000});
  EQ_E(Event::PulseStart, o.ev);
  TEST_ASSERT_TRUE(o.pwmOff);
  TEST_ASSERT_FALSE(o.forceCpLow);
  EQ_E(Method::B1Pulse, w.method());
  TEST_ASSERT_FALSE(w.pilot_frozen());  // B1'de CP olculmeye devam eder
  t += 25000;
  o = w.step({'B', true, true, 0.0f, t + 3999});
  TEST_ASSERT_TRUE(o.pwmOff);
  o = w.step({'B', true, true, 0.0f, t + 4000});
  EQ_E(Event::PulseEnd, o.ev);
  TEST_ASSERT_FALSE(o.pwmOff);
  EQ_E(Phase::Waiting, w.phase());
  // 2. ve 3. darbe, sonra vazgec.
  t += 4000;
  for (int k = 2; k <= 3; ++k) {
    o = w.step({'B', true, true, 0.0f, t + 25000});
    EQ_E(Event::PulseStart, o.ev);
    TEST_ASSERT_EQUAL_UINT8(k, w.attempts());
    t += 25000;
    o = w.step({'B', true, true, 0.0f, t + 4000});
    EQ_E(Event::PulseEnd, o.ev);
    t += 4000;
  }
  o = w.step({'B', true, true, 0.0f, t + 25000});
  EQ_E(Event::GaveUp, o.ev);
  TEST_ASSERT_EQUAL_UINT32(3, o.info);
  TEST_ASSERT_FALSE(w.active());
}

void test_b1_b1_f_strategy() {
  Config c;
  parse_seq("B1,B1,F", &c);
  Machine w(c);
  uint32_t t = armAfterIdle(w, 0, kArmMinIdleBMs + 5);
  Method seen[3];
  for (int k = 0; k < 3; ++k) {
    Output o = w.step({'B', true, true, 0.0f, t + c.waitMs});
    EQ_E(Event::PulseStart, o.ev);
    seen[k] = w.method();
    t += c.waitMs;
    uint32_t dur = (seen[k] == Method::FPulse) ? c.fMs : c.b1Ms;
    if (seen[k] == Method::FPulse) {
      TEST_ASSERT_TRUE(o.forceCpLow);
      TEST_ASSERT_TRUE(w.pilot_frozen());
    }
    o = w.step({'B', true, true, 0.0f, t + dur});
    EQ_E(Event::PulseEnd, o.ev);
    TEST_ASSERT_FALSE(o.forceCpLow);
    t += dur;
    if (seen[k] == Method::FPulse) {
      EQ_E(Phase::Settle, w.phase());
      TEST_ASSERT_TRUE(w.pilot_frozen());
      o = w.step({'B', true, true, 0.0f, t + kSettleMs});
      EQ_E(Event::SettleEnd, o.ev);
      TEST_ASSERT_FALSE(w.pilot_frozen());
      t += kSettleMs;
    }
  }
  EQ_E(Method::B1Pulse, seen[0]);
  EQ_E(Method::B1Pulse, seen[1]);
  EQ_E(Method::FPulse, seen[2]);
}

void test_f_pulse_3s() {
  Config c;
  parse_seq("F", &c);
  c.maxAttempts = 1;
  Machine w(c);
  uint32_t t = armAfterIdle(w, 0, kArmMinIdleBMs);
  Output o = w.step({'B', true, true, 0.0f, t + c.waitMs});
  TEST_ASSERT_TRUE(o.pwmOff);
  TEST_ASSERT_TRUE(o.forceCpLow);
  t += c.waitMs;
  o = w.step({'B', true, true, 0.0f, t + 2999});
  TEST_ASSERT_TRUE(o.forceCpLow);
  o = w.step({'B', true, true, 0.0f, t + 3000});
  EQ_E(Event::PulseEnd, o.ev);
  TEST_ASSERT_EQUAL_UINT32(3000, o.info);
  TEST_ASSERT_FALSE(o.forceCpLow);
  TEST_ASSERT_FALSE(o.pwmOff);  // nominal PWM'e donus (Phoenix: tek sefer 3 sn F)
}

void test_pwm5_sequence() {
  Config c;
  parse_seq("PWM5", &c);
  Machine w(c);
  uint32_t t = armAfterIdle(w, 0, kArmMinIdleBMs);
  Output o = w.step({'B', true, true, 0.0f, t + c.waitMs});
  EQ_E(Event::PulseStart, o.ev);
  TEST_ASSERT_FALSE(o.pwmOff);
  TEST_ASSERT_EQUAL_INT(kPwm5Duty, o.dutyOverride);
  TEST_ASSERT_FALSE(w.pilot_frozen());
  t += c.waitMs;
  o = w.step({'B', true, true, 0.0f, t + c.pwm5Ms - 1});
  TEST_ASSERT_EQUAL_INT(5, o.dutyOverride);
  o = w.step({'B', true, true, 0.0f, t + c.pwm5Ms});
  EQ_E(Event::StepEF, o.ev);
  TEST_ASSERT_TRUE(o.pwmOff);
  TEST_ASSERT_TRUE(o.forceCpLow);
  TEST_ASSERT_TRUE(w.pilot_frozen());
  t += c.pwm5Ms;
  o = w.step({'B', true, true, 0.0f, t + c.efMs - 1});
  TEST_ASSERT_TRUE(o.forceCpLow);
  o = w.step({'B', true, true, 0.0f, t + c.efMs});
  EQ_E(Event::PulseEnd, o.ev);
  TEST_ASSERT_FALSE(o.forceCpLow);
  TEST_ASSERT_FALSE(o.pwmOff);
  TEST_ASSERT_EQUAL_INT(-1, o.dutyOverride);  // nominal
  EQ_E(Phase::Settle, w.phase());
}

void test_pwm5_car_goes_c_resets() {
  Config c;
  parse_seq("PWM5", &c);
  Machine w(c);
  uint32_t t = armAfterIdle(w, 0, kArmMinIdleBMs);
  w.step({'B', true, true, 0.0f, t + c.waitMs});
  Output o = w.step({'C', true, true, 0.0f, t + c.waitMs + 500});
  EQ_E(Event::ResetCharging, o.ev);
  TEST_ASSERT_EQUAL_INT(-1, o.dutyOverride);
  TEST_ASSERT_FALSE(w.active());
}

void test_safety_deferral() {
  Machine w;
  uint32_t t = armAfterIdle(w, 0, kArmMinIdleBMs);
  // Kontaktor kapali: darbe yok, bir kez loglanir.
  Output o = w.step({'B', true, false, 0.0f, t + 25000});
  EQ_E(Event::Deferred, o.ev);
  TEST_ASSERT_FALSE(o.pwmOff);
  o = w.step({'B', true, false, 0.0f, t + 25200});
  EQ_E(Event::None, o.ev);
  // Akim var: darbe yok.
  o = w.step({'B', true, true, 1.0f, t + 60000});
  TEST_ASSERT_FALSE(o.pwmOff);
  TEST_ASSERT_EQUAL_UINT8(0, w.attempts());
  // Kosul duzeldi: bekleme yeniden sayilir (t0 erteleme aninda guncellendi).
  o = w.step({'B', true, true, 0.0f, t + 60000 + 24999});
  TEST_ASSERT_FALSE(o.pwmOff);
  o = w.step({'B', true, true, 0.0f, t + 60000 + 25000});
  EQ_E(Event::PulseStart, o.ev);
}

void test_resets() {
  Config c;
  parse_seq("F", &c);
  Machine w(c);
  uint32_t t = armAfterIdle(w, 0, kArmMinIdleBMs);
  w.step({'B', true, true, 0.0f, t + c.waitMs});
  TEST_ASSERT_TRUE(w.pilot_frozen());
  Output o = w.step({'B', false, true, 0.0f, t + c.waitMs + 100});  // izin kalkti
  EQ_E(Event::ResetNoPermit, o.ev);
  TEST_ASSERT_EQUAL_UINT32(1, o.info);
  TEST_ASSERT_FALSE(o.forceCpLow);
  TEST_ASSERT_FALSE(w.pilot_frozen());

  Machine w2;
  t = armAfterIdle(w2, 0, kArmMinIdleBMs);
  o = w2.step({'A', true, true, 0.0f, t + 100});
  EQ_E(Event::ResetLeftB, o.ev);

  Machine w3;
  t = armAfterIdle(w3, 0, kArmMinIdleBMs);
  Config off;
  off.enabled = false;
  EQ_E(Event::ResetConfig, w3.set_config(off));
  TEST_ASSERT_FALSE(w3.active());
  o = w3.step({'B', true, true, 0.0f, t + 30000});
  TEST_ASSERT_FALSE(o.pwmOff);
}

void test_disabled_never_pulses() {
  Config c;
  c.enabled = false;
  Machine w(c);
  w.step({'B', false, true, 0.0f, 0});
  Output o = w.step({'B', true, true, 0.0f, 100000});
  EQ_E(Event::None, o.ev);
  o = w.step({'B', true, true, 0.0f, 200000});
  TEST_ASSERT_FALSE(o.pwmOff);
  TEST_ASSERT_FALSE(w.active());
}

void test_unplug_clears_idle_timer() {
  Machine w;
  w.step({'B', false, true, 0.0f, 0});
  w.on_unplug();
  // Fis cikip takildi: yeni bekleme kisa, kurulmamali.
  w.step({'B', false, true, 0.0f, 40000});
  Output o = w.step({'B', true, true, 0.0f, 41000});
  EQ_E(Event::NotArmed, o.ev);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_match_previous_behavior);
  RUN_TEST(test_parse_format_seq);
  RUN_TEST(test_validate_ranges);
  RUN_TEST(test_blob_roundtrip);
  RUN_TEST(test_not_armed_short_idle);
  RUN_TEST(test_b1_default_sequence);
  RUN_TEST(test_b1_b1_f_strategy);
  RUN_TEST(test_f_pulse_3s);
  RUN_TEST(test_pwm5_sequence);
  RUN_TEST(test_pwm5_car_goes_c_resets);
  RUN_TEST(test_safety_deferral);
  RUN_TEST(test_resets);
  RUN_TEST(test_disabled_never_pulses);
  RUN_TEST(test_unplug_clears_idle_timer);
  return UNITY_END();
}
