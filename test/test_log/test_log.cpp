// Wi-Fi log halka tamponu (src/log/log_ring) birim testleri: pio test -e native
#include <stdio.h>
#include <string.h>
#include <string>
#include <unity.h>

#include "log/log_ring.h"

using evse_log::LogRing;

void setUp() {}
void tearDown() {}

static std::string readAll(const LogRing& r, uint32_t since, uint32_t* last = nullptr, size_t cap = 4096) {
  std::string out(cap, '\0');
  uint32_t l = 0;
  size_t n = r.read(since, &out[0], cap, &l);
  if (last) *last = l;
  out.resize(n);
  return out;
}

void test_empty() {
  char st[64];
  LogRing r(st, sizeof(st));
  uint32_t last = 99;
  TEST_ASSERT_EQUAL_STRING("", readAll(r, 0, &last).c_str());
  TEST_ASSERT_EQUAL_UINT32(0, last);
  TEST_ASSERT_EQUAL_UINT32(1, r.firstSeq());
  TEST_ASSERT_EQUAL_UINT32(1, r.nextSeq());
}

void test_append_and_seq() {
  char st[128];
  LogRing r(st, sizeof(st));
  r.append("alfa", 4);
  r.append("beta\r\n", 6);  // \r atilir, \n bosluk olur
  uint32_t last = 0;
  TEST_ASSERT_EQUAL_STRING("1 alfa\n2 beta \n", readAll(r, 0, &last).c_str());
  TEST_ASSERT_EQUAL_UINT32(2, last);
  TEST_ASSERT_EQUAL_UINT32(2, r.lineCount());
}

void test_since_incremental() {
  char st[256];
  LogRing r(st, sizeof(st));
  r.append("a", 1);
  r.append("b", 1);
  r.append("c", 1);
  uint32_t last = 0;
  TEST_ASSERT_EQUAL_STRING("3 c\n", readAll(r, 2, &last).c_str());
  TEST_ASSERT_EQUAL_UINT32(3, last);
  TEST_ASSERT_EQUAL_STRING("", readAll(r, 3, &last).c_str());
  TEST_ASSERT_EQUAL_UINT32(3, last);
  // Gelecekteki since: bos, last = son satir
  TEST_ASSERT_EQUAL_STRING("", readAll(r, 100, &last).c_str());
  TEST_ASSERT_EQUAL_UINT32(3, last);
  r.append("d", 1);
  TEST_ASSERT_EQUAL_STRING("4 d\n", readAll(r, 3, &last).c_str());
}

void test_eviction_and_wrap() {
  char st[16];  // "satirN\n" = 7 bayt -> en cok 2 satir
  LogRing r(st, sizeof(st));
  for (int i = 1; i <= 10; i++) {
    char line[8];
    snprintf(line, sizeof(line), "satir%d", i % 10);
    r.append(line, strlen(line));
    TEST_ASSERT_TRUE(r.usedBytes() <= r.capacity());
  }
  TEST_ASSERT_EQUAL_UINT32(9, r.firstSeq());
  TEST_ASSERT_EQUAL_UINT32(11, r.nextSeq());
  TEST_ASSERT_EQUAL_STRING("9 satir9\n10 satir0\n", readAll(r, 0).c_str());
  // Kaybolan satirlari isteyen since: eldeki en eskiden baslar
  TEST_ASSERT_EQUAL_STRING("9 satir9\n10 satir0\n", readAll(r, 3).c_str());
}

void test_long_line_truncated() {
  char st[64];
  LogRing r(st, sizeof(st), 10);
  r.append("0123456789ABCDEF", 16);
  TEST_ASSERT_EQUAL_STRING("1 0123456789\n", readAll(r, 0).c_str());
  // Kapasiteden uzun satir da tampona sigar (kirpilir), tampon bozulmaz
  char st2[8];
  LogRing r2(st2, sizeof(st2), 100);
  r2.append("abcdefghijk", 11);
  r2.append("xy", 2);
  TEST_ASSERT_EQUAL_STRING("2 xy\n", readAll(r2, 1).c_str());
}

void test_read_out_too_small_no_partial_line() {
  char st[128];
  LogRing r(st, sizeof(st));
  r.append("birinci", 7);
  r.append("ikinci", 6);
  uint32_t last = 0;
  // "1 birinci\n" = 10 bayt + '\0' = 11; ikinci satir sigmaz
  std::string s = readAll(r, 0, &last, 14);
  TEST_ASSERT_EQUAL_STRING("1 birinci\n", s.c_str());
  TEST_ASSERT_EQUAL_UINT32(1, last);
  s = readAll(r, last, &last, 14);
  TEST_ASSERT_EQUAL_STRING("2 ikinci\n", s.c_str());
  TEST_ASSERT_EQUAL_UINT32(2, last);
}

void test_clear_keeps_seq_monotonic() {
  char st[64];
  LogRing r(st, sizeof(st));
  r.append("x", 1);
  r.append("y", 1);
  r.clear();
  TEST_ASSERT_EQUAL_UINT32(0, r.lineCount());
  r.append("z", 1);
  TEST_ASSERT_EQUAL_STRING("3 z\n", readAll(r, 0).c_str());
}

void test_many_wraps_consistent() {
  char st[100];
  LogRing r(st, sizeof(st), 30);
  uint32_t last = 0;
  std::string got;
  for (int i = 0; i < 500; i++) {
    char line[40];
    int n = snprintf(line, sizeof(line), "L%d-%.*s", i, i % 20, "xxxxxxxxxxxxxxxxxxxx");
    r.append(line, (size_t)n);
    // Her ekleme sonrasi artimli oku: hic satir atlanmamali (tampon yeterince sik okunuyor)
    std::string s = readAll(r, last, &last);
    char want[64];
    snprintf(want, sizeof(want), "%d %s\n", i + 1, line);
    TEST_ASSERT_EQUAL_STRING(want, s.c_str());
  }
  TEST_ASSERT_EQUAL_UINT32(500, last);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_empty);
  RUN_TEST(test_append_and_seq);
  RUN_TEST(test_since_incremental);
  RUN_TEST(test_eviction_and_wrap);
  RUN_TEST(test_long_line_truncated);
  RUN_TEST(test_read_out_too_small_no_partial_line);
  RUN_TEST(test_clear_keeps_seq_monotonic);
  RUN_TEST(test_many_wraps_consistent);
  return UNITY_END();
}
