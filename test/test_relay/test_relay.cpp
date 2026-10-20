// Kontaktor karar mantigi (src/io/relay_core) birim testleri: pio test -e native
#include <unity.h>

#include "io/relay_core.h"

using namespace evse_relay;

#define EQ_R(a, b) TEST_ASSERT_EQUAL_INT((int)(a), (int)(b))

void setUp() {}
void tearDown() {}

void test_should_close() {
  TEST_ASSERT_TRUE(should_close('C', true, 53));
  TEST_ASSERT_TRUE(should_close('D', true, 10));
  TEST_ASSERT_FALSE(should_close('B', true, 53));
  TEST_ASSERT_FALSE(should_close('A', true, 53));
  TEST_ASSERT_FALSE(should_close('E', true, 53));
  TEST_ASSERT_FALSE(should_close('F', true, 53));
  TEST_ASSERT_FALSE(should_close('C', false, 53));  // PWM kesik (STOP / B1 / izin yok)
  TEST_ASSERT_FALSE(should_close('C', true, 0));
  TEST_ASSERT_FALSE(should_close('C', true, 5));    // %5 = dijital iletisim, akim izni degil (PWM5 uyandirma)
  TEST_ASSERT_FALSE(should_close('C', true, 100));
}

void test_manual_close_allowed() {
  TEST_ASSERT_TRUE(manual_close_allowed('C', true, 53));
  TEST_ASSERT_FALSE(manual_close_allowed('B', true, 53));   // /relay?on=1 B'de reddedilir
  TEST_ASSERT_FALSE(manual_close_allowed('A', false, 0));
  TEST_ASSERT_FALSE(manual_close_allowed('C', false, 0));   // STOP sonrasi
}

void test_emergency_immediate() {
  EQ_R(OffReason::Emergency, off_decision('A', 16.0f, 0, 0));
  EQ_R(OffReason::Emergency, off_decision('E', -1.0f, 0, 0));
  EQ_R(OffReason::Emergency, off_decision('F', 0.0f, 0, 50));
  EQ_R(OffReason::Emergency, off_decision('?', 0.0f, 0, 50));
}

void test_c_to_b_zero_current_immediate() {
  // C->B, akim yok: ilk kontrolde (debounce sonrasi) ac -> hedef < 100 ms.
  EQ_R(OffReason::ZeroCurrent, off_decision('B', 0.1f, 0, 0));
  // Onay suresi verildiyse beklenir.
  EQ_R(OffReason::None, off_decision('B', 0.1f, 10, 50));
  EQ_R(OffReason::ZeroCurrent, off_decision('B', 0.1f, 50, 50));
}

void test_c_to_b_under_load_waits_short() {
  // Akim varken C->B: yuk altinda acma yok, akim sifirlaninca ya da kisa zaman asiminda.
  EQ_R(OffReason::None, off_decision('B', 10.0f, 0, 0));
  EQ_R(OffReason::None, off_decision('B', 10.0f, kOffLoadTimeoutBMs - 1, 0));
  EQ_R(OffReason::ZeroCurrent, off_decision('B', 0.2f, 120, 0));
  EQ_R(OffReason::Timeout, off_decision('B', 10.0f, kOffLoadTimeoutBMs, 0));
  // Akim bilinmiyor (olcum yok): zaman asimi.
  EQ_R(OffReason::None, off_decision('B', -1.0f, 100, 0));
  EQ_R(OffReason::Timeout, off_decision('B', -1.0f, kOffLoadTimeoutBMs, 0));
}

void test_soft_stop_pwm_off_in_c() {
  // PWM kesildi (STOP / izin kalkti), arac hala C ve akim cekiyor: 3 sn'ye kadar bekle.
  EQ_R(OffReason::None, off_decision('C', 16.0f, 0, 0));
  EQ_R(OffReason::None, off_decision('C', 16.0f, 2999, 0));
  EQ_R(OffReason::Timeout, off_decision('C', 16.0f, kOffSoftStopTimeoutMs, 0));
  EQ_R(OffReason::None, off_decision('D', 16.0f, 1000, 0));
  // Arac akimi kesti (C'de kalsa da): ac.
  EQ_R(OffReason::ZeroCurrent, off_decision('C', 0.3f, 800, 0));
  TEST_ASSERT_EQUAL(3000u, kOffSoftStopTimeoutMs);
}

void test_threshold() {
  EQ_R(OffReason::None, off_decision('B', kZeroCurrentA, 0, 0));
  EQ_R(OffReason::ZeroCurrent, off_decision('B', kZeroCurrentA - 0.01f, 0, 0));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_should_close);
  RUN_TEST(test_manual_close_allowed);
  RUN_TEST(test_emergency_immediate);
  RUN_TEST(test_c_to_b_zero_current_immediate);
  RUN_TEST(test_c_to_b_under_load_waits_short);
  RUN_TEST(test_soft_stop_pwm_off_in_c);
  RUN_TEST(test_threshold);
  return UNITY_END();
}
