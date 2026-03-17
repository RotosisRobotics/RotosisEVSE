#include "oled_ui.h"
#include "app_pins.h"
#include <U8g2lib.h>
#include <WiFi.h>
#include <Wire.h>

// 1.3" 128x64 SH1106 I2C
// Reset pini yoksa U8X8_PIN_NONE kullanilir.
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE
);

static bool oledAvailable = false;
static uint8_t oledAddress = 0;

static uint8_t activeScreen = 0;
static uint32_t lastScreenSwitchMs = 0;
static String lastUiState = "";

static const uint32_t kScreenRotateMs = 4000;
static const float kPhaseOnThresholdA = 0.90f;

static bool probe_oled_addr(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static bool is_error_state(const String& stateStable)
{
  return (stateStable == "E" || stateStable == "F");
}

static const char* ui_state_label(const String& stateStable)
{
  if (stateStable == "A") return "ARAC YOK";
  if (stateStable == "B") return "HAZIR";
  if (stateStable == "C") return "SARJ";
  if (stateStable == "D") return "HAVALANDIR";
  if (stateStable == "E" || stateStable == "F") return "HATA";
  return "DURUM";
}

static const char* phase_label(float ia, float ib, float ic)
{
  if (ib > kPhaseOnThresholdA || ic > kPhaseOnThresholdA) return "3F";
  if (ia > kPhaseOnThresholdA) return "1F";
  return "--";
}

static void format_duration(uint32_t chargeSeconds, char* out, size_t outSize)
{
  uint32_t hh = chargeSeconds / 3600;
  uint32_t mm = (chargeSeconds % 3600) / 60;
  uint32_t ss = chargeSeconds % 60;
  snprintf(out, outSize, "%02lu:%02lu:%02lu",
           (unsigned long)hh,
           (unsigned long)mm,
           (unsigned long)ss);
}

static void draw_centered_text(int y, const char* text)
{
  int x = (128 - u8g2.getStrWidth(text)) / 2;
  if (x < 0) x = 0;
  u8g2.drawStr(x, y, text);
}

static void draw_header(const char* title, const String& stateStable, bool alert)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  if (alert) {
    u8g2.drawBox(0, 0, 128, 12);
    u8g2.setDrawColor(0);
  } else {
    u8g2.drawRFrame(0, 0, 128, 12, 2);
  }

  draw_centered_text(9, title);

  int codeX = 124 - u8g2.getStrWidth(stateStable.c_str());
  if (codeX < 2) codeX = 2;
  u8g2.drawStr(codeX, 9, stateStable.c_str());

  if (alert) u8g2.setDrawColor(1);
}

static void draw_badge(int x, int y, int w, const char* text, bool active)
{
  const int h = 10;
  u8g2.setFont(u8g2_font_5x7_tf);

  if (active) {
    u8g2.drawRBox(x, y, w, h, 2);
    u8g2.setDrawColor(0);
  } else {
    u8g2.drawRFrame(x, y, w, h, 2);
  }

  int textX = x + (w - u8g2.getStrWidth(text)) / 2;
  if (textX < x + 1) textX = x + 1;
  u8g2.drawStr(textX, y + 8, text);
  u8g2.setDrawColor(1);
}

static void draw_charging_indicator(bool charging)
{
  if (!charging) {
    u8g2.drawCircle(118, 26, 4, U8G2_DRAW_ALL);
    return;
  }

  if (((millis() / 350) % 2U) == 0U) {
    u8g2.drawDisc(118, 26, 4, U8G2_DRAW_ALL);
  } else {
    u8g2.drawCircle(118, 26, 4, U8G2_DRAW_ALL);
  }
}

static void draw_overview_screen(const String& stateStable,
                                 float powerW,
                                 float energyKwh,
                                 uint32_t chargeSeconds,
                                 bool relayOn,
                                 bool staConnected,
                                 bool cableConnected)
{
  char powerText[10];
  char statsLine[28];
  char timeText[12];
  bool charging = (stateStable == "C" || stateStable == "D");

  draw_header(ui_state_label(stateStable), stateStable, false);

  u8g2.drawRFrame(0, 15, 128, 27, 3);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(5, 22, "ANLIK GUC");
  u8g2.drawStr(107, 22, "kW");

  snprintf(powerText, sizeof(powerText), "%.1f", powerW / 1000.0f);
  u8g2.setFont(u8g2_font_logisoso18_tn);
  draw_centered_text(38, powerText);
  draw_charging_indicator(charging);

  format_duration(chargeSeconds, timeText, sizeof(timeText));
  u8g2.setFont(u8g2_font_5x7_tf);
  snprintf(statsLine, sizeof(statsLine), "E:%.2fkWh  T:%s", energyKwh, timeText);
  draw_centered_text(49, statsLine);

  draw_badge(0, 54, 40, "WiFi", staConnected);
  draw_badge(44, 54, 40, "Kablo", cableConnected);
  draw_badge(88, 54, 40, "Role", relayOn);
}

static void draw_technical_screen(const String& stateStable,
                                  float ia,
                                  float ib,
                                  float ic,
                                  float powerW,
                                  float energyKwh,
                                  uint32_t chargeSeconds,
                                  bool relayOn,
                                  bool staConnected)
{
  char line[32];
  char timeText[12];

  draw_header("DETAYLAR", stateStable, false);

  u8g2.setFont(u8g2_font_6x10_tf);

  snprintf(line, sizeof(line), "I1:%4.1fA I2:%4.1fA", ia, ib);
  u8g2.drawStr(2, 22, line);

  snprintf(line, sizeof(line), "I3:%4.1fA Faz:%s", ic, phase_label(ia, ib, ic));
  u8g2.drawStr(2, 34, line);

  snprintf(line, sizeof(line), "P:%4.1fkW E:%5.2f", powerW / 1000.0f, energyKwh);
  u8g2.drawStr(2, 46, line);

  format_duration(chargeSeconds, timeText, sizeof(timeText));
  snprintf(line, sizeof(line), "T:%s R:%s", timeText, relayOn ? "ON" : "OFF");
  u8g2.drawStr(2, 58, line);

  u8g2.setFont(u8g2_font_5x7_tf);
  if (staConnected && WiFi.localIP()[0] != 0) {
    String ipLine = "IP:" + WiFi.localIP().toString();
    u8g2.drawStr(2, 64, ipLine.c_str());
  } else {
    u8g2.drawStr(2, 64, "WiFi: baglaniyor");
  }
}

static void draw_error_screen(const String& stateStable,
                              bool staConnected,
                              bool cableConnected)
{
  draw_header("HATA", stateStable, true);

  u8g2.drawTriangle(10, 18, 26, 18, 18, 34);
  u8g2.drawLine(18, 21, 18, 28);
  u8g2.drawDisc(18, 31, 1, U8G2_DRAW_ALL);

  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(34, 28, "Sarj durdu");

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(34, 42, "Pilot/CP kontrol edin");
  u8g2.drawStr(34, 50, "Guvenlikte bekliyor");

  draw_badge(0, 54, 40, "WiFi", staConnected);
  draw_badge(44, 54, 40, "Kablo", cableConnected);
  draw_badge(88, 54, 40, "Hata", true);
}

void oled_init()
{
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  if (probe_oled_addr(0x3C)) {
    oledAddress = 0x3C;
    oledAvailable = true;
  } else if (probe_oled_addr(0x3D)) {
    oledAddress = 0x3D;
    oledAvailable = true;
  } else {
    oledAvailable = false;
    Serial.println("OLED bulunamadi: 0x3C / 0x3D cevap vermedi.");
    return;
  }

  u8g2.setI2CAddress(oledAddress << 1);
  u8g2.begin();
  Serial.printf("OLED hazir: SH1106 @ 0x%02X\n", oledAddress);

  activeScreen = 0;
  lastScreenSwitchMs = millis();
  lastUiState = "";

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 30, "Rotosis EVSE");
  u8g2.drawStr(10, 45, "Baslatiliyor...");
  u8g2.sendBuffer();
}

void oled_draw(const String& stateStable,
               float ia,
               float ib,
               float ic,
               float powerW,
               float energyKwh,
               uint32_t chargeSeconds,
               bool relayOn,
               bool staConnected,
               bool cableConnected)
{
  if (!oledAvailable) return;

  uint32_t nowMs = millis();
  bool errorState = is_error_state(stateStable);

  if (stateStable != lastUiState) {
    activeScreen = 0;
    lastUiState = stateStable;
    lastScreenSwitchMs = nowMs;
  }

  if (!errorState && (nowMs - lastScreenSwitchMs >= kScreenRotateMs)) {
    activeScreen = (activeScreen + 1U) % 2U;
    lastScreenSwitchMs = nowMs;
  }

  u8g2.clearBuffer();

  if (errorState) {
    draw_error_screen(stateStable, staConnected, cableConnected);
  } else if (activeScreen == 0) {
    draw_overview_screen(stateStable,
                         powerW,
                         energyKwh,
                         chargeSeconds,
                         relayOn,
                         staConnected,
                         cableConnected);
  } else {
    draw_technical_screen(stateStable,
                          ia,
                          ib,
                          ic,
                          powerW,
                          energyKwh,
                          chargeSeconds,
                          relayOn,
                          staConnected);
  }

  u8g2.sendBuffer();
}
