#include <Arduino.h>
#include "log/dev_log.h"
#include <WiFi.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "app_config.h"
#include "app_pins.h"
#include "pilot/pilot.h"
#include "pilot/wake_core.h"
#include "pilot/wake_cfg.h"
#include "io/relay.h"
#include "net/web_ui.h"
#include "net/cloud_client.h"
#include "net/schedule.h"
#include "ui/oled_ui.h"
#include "io/current_sensor.h"
#include "OTA_Manager.h"

// Manifest yolu donanima gore src/hw/config_scN.h icinde (SC_1: main/version.json).
static constexpr char kOtaManifestUrl[] = HW_OTA_MANIFEST_URL;
static constexpr char kGitHubFingerprint[] = "";
static constexpr uint32_t kOtaAutoCheckIntervalMs = 60UL * 60UL * 1000UL;
static constexpr uint8_t kMaxConsecutiveWdtResets = 3;
static constexpr uint8_t kQuickResetFactoryThreshold = 5;
static constexpr uint32_t kQuickResetClearAfterMs = 30000;
static constexpr int kForceFactoryPin = 0;             // BOOT butonu (GPIO0)
static constexpr uint32_t kForceFactoryHoldMs = 10000; // 10 saniye
static bool sFactoryRestartPending = false;
static bool sFactoryButtonPressActive = false;
static bool sFactoryButtonHoldHandled = false;
static uint32_t sFactoryButtonPressStartMs = 0;
static uint8_t sQuickResetBootStreak = 0;
static bool sQuickResetClearArmed = false;
static uint32_t sQuickResetClearStartMs = 0;
static esp_reset_reason_t sLastResetReason = ESP_RST_UNKNOWN;

static const char* resetReasonName(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static bool isWdtResetReason(esp_reset_reason_t reason)
{
  return reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT;
}

static bool isQuickResetReason(esp_reset_reason_t reason)
{
  return reason == ESP_RST_POWERON || reason == ESP_RST_EXT || reason == ESP_RST_BROWNOUT;
}

static bool switchBootToFactory()
{
  const esp_partition_t* factory = esp_partition_find_first(
    ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr
  );
  if (factory == nullptr) {
    LogOut.println("[BOOTCTL] Factory partition bulunamadi");
    return false;
  }
  esp_err_t err = esp_ota_set_boot_partition(factory);
  if (err != ESP_OK) {
    LogOut.printf("[BOOTCTL] Factory secilemedi: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
}

static void handleForceFactoryByButton()
{
  pinMode(kForceFactoryPin, INPUT_PULLUP);
  uint32_t startMs = millis();
  while (millis() - startMs < kForceFactoryHoldMs) {
    if (digitalRead(kForceFactoryPin) != LOW) {
      return;
    }
    delay(10);
  }

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
    LogOut.println("[BOOTCTL] Zaten factory partitionda");
    return;
  }

  if (switchBootToFactory()) {
    LogOut.println("[BOOTCTL] GPIO0 ile factory secildi; buton birakilinca yeniden baslatilacak");
    while (digitalRead(kForceFactoryPin) == LOW) {
      delay(10);
    }
    delay(100);
    esp_restart();
  }
}

static void pollForceFactoryByButtonRuntime()
{
  bool pressed = (digitalRead(kForceFactoryPin) == LOW);
  if (!pressed) {
    sFactoryButtonPressActive = false;
    sFactoryButtonHoldHandled = false;
    sFactoryButtonPressStartMs = 0;
    return;
  }

  if (!sFactoryButtonPressActive) {
    sFactoryButtonPressActive = true;
    sFactoryButtonHoldHandled = false;
    sFactoryButtonPressStartMs = millis();
    return;
  }

  if (sFactoryButtonHoldHandled || (millis() - sFactoryButtonPressStartMs) < kForceFactoryHoldMs) return;
  sFactoryButtonHoldHandled = true;

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
    LogOut.println("[BOOTCTL] Runtime hold algilandi ama zaten factory partitionda");
    return;
  }

  if (switchBootToFactory()) {
    LogOut.println("[BOOTCTL] Runtime hold ile factory secildi; buton birakilinca yeniden baslatilacak");
    sFactoryRestartPending = true;
  }
}

static void processPendingFactoryRestart()
{
  if (!sFactoryRestartPending) return;
  if (digitalRead(kForceFactoryPin) == LOW) return;

  sFactoryRestartPending = false;
  LogOut.println("[BOOTCTL] Buton birakildi, factory icin yeniden baslatiliyor");
  delay(100);
  esp_restart();
}

static void ensureForceFactoryPinMode()
{
  static bool configured = false;
  if (configured) return;
  pinMode(kForceFactoryPin, INPUT_PULLUP);
  configured = true;
}

int factoryButtonPin()
{
  return kForceFactoryPin;
}

bool factoryButtonIsPressed()
{
  ensureForceFactoryPinMode();
  return digitalRead(kForceFactoryPin) == LOW;
}

uint32_t factoryButtonHoldMs()
{
  if (!sFactoryButtonPressActive || sFactoryButtonPressStartMs == 0) return 0;
  return millis() - sFactoryButtonPressStartMs;
}

bool factoryButtonRestartPending()
{
  return sFactoryRestartPending;
}

uint8_t factoryQuickResetStreak()
{
  return sQuickResetBootStreak;
}

bool factoryQuickResetClearArmed()
{
  return sQuickResetClearArmed;
}

uint32_t factoryQuickResetClearRemainingMs()
{
  if (!sQuickResetClearArmed) return 0;
  uint32_t elapsed = millis() - sQuickResetClearStartMs;
  if (elapsed >= kQuickResetClearAfterMs) return 0;
  return kQuickResetClearAfterMs - elapsed;
}

int factoryLastResetReason()
{
  return (int)sLastResetReason;
}

static void serviceFactoryButton()
{
  ensureForceFactoryPinMode();
  processPendingFactoryRestart();
  if (sFactoryRestartPending) return;
  pollForceFactoryByButtonRuntime();
}

static void initFactoryButton()
{
  ensureForceFactoryPinMode();
  handleForceFactoryByButton();
  if (sFactoryRestartPending) {
    processPendingFactoryRestart();
  }
}

static void handleRescueFallbackIfNeeded()
{
  Preferences prefs;
  if (!prefs.begin("bootctl", false)) return;

  sLastResetReason = esp_reset_reason();
  uint8_t failCount = prefs.getUChar("wdt_cnt", 0);
  failCount = isWdtResetReason(sLastResetReason) ? (uint8_t)(failCount + 1) : 0;
  prefs.putUChar("wdt_cnt", failCount);

  const esp_partition_t* running = esp_ota_get_running_partition();
  bool runningFactory =
    (running != nullptr && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY);
  uint8_t quickResetStreak = prefs.getUChar("rst_streak", 0);
  if (runningFactory) {
    quickResetStreak = 0;
  } else if (isQuickResetReason(sLastResetReason) && quickResetStreak < 255) {
    quickResetStreak++;
  }
  prefs.putUChar("rst_streak", quickResetStreak);
  sQuickResetBootStreak = quickResetStreak;
  sQuickResetClearArmed = (!runningFactory && quickResetStreak > 0);
  sQuickResetClearStartMs = millis();

  if (failCount >= kMaxConsecutiveWdtResets && switchBootToFactory()) {
    prefs.putUChar("wdt_cnt", 0);
    prefs.end();
    LogOut.println("[BOOTCTL] 3x WDT reset algilandi, factory secildi");
    delay(50);
    esp_restart();
  }

  if (!runningFactory &&
      quickResetStreak >= kQuickResetFactoryThreshold &&
      switchBootToFactory()) {
    prefs.putUChar("rst_streak", 0);
    prefs.end();
    sQuickResetBootStreak = 0;
    sQuickResetClearArmed = false;
    LogOut.printf("[BOOTCTL] %u hizli reset algilandi, factory secildi\n", quickResetStreak);
    delay(50);
    esp_restart();
  }

  prefs.end();
}

static void clearQuickResetStreakIfStable()
{
  if (!sQuickResetClearArmed) return;
  if ((millis() - sQuickResetClearStartMs) < kQuickResetClearAfterMs) return;

  Preferences prefs;
  if (!prefs.begin("bootctl", false)) return;
  prefs.putUChar("rst_streak", 0);
  prefs.end();

  sQuickResetBootStreak = 0;
  sQuickResetClearArmed = false;
  sQuickResetClearStartMs = 0;
  LogOut.println("[BOOTCTL] Hizli reset sayaci temizlendi");
}

// Bu dosya projenin merkez akisidir.
// Neyi nereden degistirecegini hizli bulmak icin:
// - pin degisimi: include/app_pins.h
// - CP / state esikleri: bu dosyadaki global ayarlar
// - pilot state mantigi: src/pilot/pilot.cpp
// - role gecikmesi: src/io/relay.cpp
// - OLED gorunumu: src/ui/oled_ui.cpp
// - web panel ve API: src/net/web_ui.cpp

// CP yorumlama ayarlari. Web panelinden de guncellenebilir.
float CP_DIVIDER_RATIO = 4.62f;
float TH_A_MIN = 11.0f;
float TH_B_MIN = 9.6f;
float TH_C_MIN = 7.8f;
float TH_D_MIN = 5.0f;
float TH_E_MIN = 1.0f;
float marginUp = 0.30f;
float marginDown = 0.30f;
int   stableCount = 3;      // CP debounce: ardisik ayni okuma sayisi (hizli yol, her biri 25 ms)
int   loopIntervalMs = 200; // Varsayilan olcum hizi 200 ms

float g_powerW = 0.0f;
float g_energyKWh = 0.0f;
uint32_t g_chargeSeconds = 0;
int g_phaseCount = 1;
float g_currentLimitA = 32.0f;
float g_targetCurrentLimitA = 32.0f;

// Hizli kontrol modu: 0=AUTO, 1=START, 2=STOP
int g_chargeMode = 0;
uint32_t g_manualStopAlertUntilMs = 0;
uint32_t g_manualStopAutoResumeAtMs = 0;

// Aktif seans bilgisi
bool g_sessionLive = false;
uint32_t g_sessionLiveStartSec = 0;
uint32_t g_sessionLiveSeconds = 0;
float g_sessionLiveEnergyKWh = 0.0f;

// Son seanslar cihaz acik kaldigi surece RAM'de tutulur.
uint32_t g_histStartSec[20] = {0};
uint32_t g_histDurationSec[20] = {0};
float g_histEnergyKWh[20] = {0.0f};
float g_histAvgPowerW[20] = {0.0f};
uint8_t g_histPhaseCount[20] = {0};
int g_histCount = 0;
int g_histHead = 0;

static float energyWh = 0.0f;
static uint32_t lastEnergyMs = 0;
static uint32_t chargeAccumMs = 0;
static uint32_t chargeSegmentStartMs = 0;
static uint32_t sessionStartMs = 0;
static uint32_t sessionActiveMs = 0;   // seansin gercek sarj suresi (duraklamalar sayilmaz)
static float sessionStartEnergyWh = 0.0f;
static uint8_t sessionPhaseMax = 1;

// Gecmis ekranini ve web API'sini sifirlamak icin kullanilir.
void resetHistoryData()
{
  for (int i = 0; i < 20; i++) {
    g_histStartSec[i] = 0;
    g_histDurationSec[i] = 0;
    g_histEnergyKWh[i] = 0.0f;
    g_histAvgPowerW[i] = 0.0f;
    g_histPhaseCount[i] = 0;
  }
  g_histCount = 0;
  g_histHead = 0;
}

void resetChargeData(bool clearHistory)
{
  // Anlik enerji ve aktif seans verilerini sifirlar.
  // Web tarafindaki "simdi sifirla" akisi burayi kullanir.
  energyWh = 0.0f;
  lastEnergyMs = 0;
  chargeAccumMs = 0;
  chargeSegmentStartMs = 0;
  sessionStartMs = 0;
  sessionActiveMs = 0;
  sessionStartEnergyWh = 0.0f;
  sessionPhaseMax = 1;

  g_powerW = 0.0f;
  g_energyKWh = 0.0f;
  g_chargeSeconds = 0;
  g_phaseCount = 1;

  g_sessionLive = false;
  g_sessionLiveStartSec = 0;
  g_sessionLiveSeconds = 0;
  g_sessionLiveEnergyKWh = 0.0f;

  if (clearHistory) {
    resetHistoryData();
  }
}


static bool lastPwmEnabled = false;
static int  lastDuty = -1;

// PWM yalnizca gercekten degisince yazilir; gereksiz ledc guncellemesi engellenir.
static void applyPwmIfChanged()
{
  if (pwmEnabled != lastPwmEnabled || pwmDutyPercent != lastDuty) {
    pilot_apply_pwm();
    lastPwmEnabled = pwmEnabled;
    lastDuty = pwmDutyPercent;
  }
}

// IEC 61851 duty -> akim limiti yaklasimi.
static float duty_to_amps(int dutyPercent)
{
  if (dutyPercent <= 0) return 0.0f;

  if (dutyPercent < 10) {
    return 0.0f;
  } else if (dutyPercent <= 85) {
    return dutyPercent * 0.6f;
  } else if (dutyPercent <= 96) {
    return (dutyPercent - 64) * 2.5f;
  }
  return 0.0f;
}

static int amps_to_duty(float amps)
{
  if (amps < 6.0f) amps = 6.0f;
  if (amps > 32.0f) amps = 32.0f;
  int dutyPercent = (int)lroundf(amps / 0.6f);
  if (dutyPercent < 10) dutyPercent = 10;
  if (dutyPercent > 85) dutyPercent = 85;
  return dutyPercent;
}

// ---------------------------------------------------------------------------
// Arac uyandirma (planli sarj / gec gelen izin) -- durum makinesi: src/pilot/wake_core.h
// Senaryo: arac B'de uzun sure PWM'siz bekledi; izin gelip PWM acildiginda arac uykuda oldugu icin
// B->C yapmiyor. Yontem (B1 / F / PWM5), sure, deneme sayisi NVS'de (wake_cfg, GET/POST /api/wake).
// Darbe YALNIZ state B'de, kontaktor kesin birakikken ve akim yokken yapilir (ark riski yok).
// Normal takip-sarj et akisi etkilenmez: tetik yalniz "PWM'siz B'de >= kArmMinIdleBMs bekledikten
// sonra PWM acildi" durumunda kurulur.
static evse_wake::Machine s_wake;
static uint32_t s_wakeCfgGen = 0;
static bool s_wakeCpLow = false;    // pilot_force_cp_low'a en son yazilan
static bool s_wakeCpDirty = false;  // CP cikisi bu turda yeniden yazilmali

const char* wake_runtime_phase() { return evse_wake::phase_name(s_wake.phase()); }
uint8_t wake_runtime_attempts() { return s_wake.attempts(); }

static void wake_set_cp_low(bool on)
{
  if (on == s_wakeCpLow) return;
  pilot_force_cp_low(on);
  s_wakeCpLow = on;
  s_wakeCpDirty = true;
}

static const char* wake_method_desc(evse_wake::Method m)
{
  switch (m) {
    case evse_wake::Method::B1Pulse:   return "PWM kapali, CP sabit +12V/+9V (B1)";
    case evse_wake::Method::FPulse:    return "CP sabit LOW -12V/0V (F)";
    case evse_wake::Method::Pwm5: return "PWM %5, sonra E/F, sonra nominal (ISO 15118-3)";
  }
  return "?";
}

static uint32_t wake_method_ms(const evse_wake::Config& c, evse_wake::Method m)
{
  switch (m) {
    case evse_wake::Method::B1Pulse:   return c.b1Ms;
    case evse_wake::Method::FPulse:    return c.fMs;
    case evse_wake::Method::Pwm5: return c.pwm5Ms + c.efMs;
  }
  return 0;
}

static void wake_log(const evse_wake::Output& o, const PilotMeasurements& m, float iA, uint32_t now)
{
  using evse_wake::Event;
  if (o.ev == Event::None) return;
  const evse_wake::Config& c = s_wake.config();
  const char* st = m.stateStable.c_str();
  char seq[40];
  switch (o.ev) {
    case Event::Armed:
      evse_wake::format_seq(c, seq, sizeof(seq));
      LogOut.printf("[WAKE] PWM %lu sn PWM'siz bekleme sonrasi acildi; %lu sn icinde C gelmezse uyandirma "
                    "(yontem=%s, en cok %u) st=%s CP=%.2fV I=%.2f A\n",
                    (unsigned long)(o.info / 1000), (unsigned long)(c.waitMs / 1000), seq,
                    (unsigned)c.maxAttempts, st, m.cpHigh, iA);
      break;
    case Event::NotArmed:
      LogOut.printf("[WAKE] PWM acildi ama PWM'siz bekleme %lu ms < %lu ms; uyandirma kurulmadi (st=%s I=%.2f A)\n",
                    (unsigned long)o.info, (unsigned long)evse_wake::kArmMinIdleBMs, st, iA);
      break;
    case Event::Deferred:
      LogOut.printf("[WAKE] Darbe %u/%u ertelendi: kontaktor birakik degil ya da akim var "
                    "(kontaktorAcik=%d I=%.2f A st=%s)\n",
                    (unsigned)(s_wake.attempts() + 1), (unsigned)c.maxAttempts,
                    relay_contacts_open() ? 1 : 0, iA, st);
      break;
    case Event::PulseStart:
      LogOut.printf("[WAKE] Arac B'de bekliyor: darbe %u/%u BASLADI yontem=%s (%s) sure=%lu ms "
                    "st=%s CP=%.2fV I=%.2f A t=%lu\n",
                    (unsigned)s_wake.attempts(), (unsigned)c.maxAttempts,
                    evse_wake::method_name(s_wake.method()), wake_method_desc(s_wake.method()),
                    (unsigned long)wake_method_ms(c, s_wake.method()), st, m.cpHigh, iA, (unsigned long)now);
      break;
    case Event::StepEF:
      LogOut.printf("[WAKE] PWM5 adimi bitti (%lu ms); E/F adimi BASLADI (%lu ms, CP sabit LOW) st=%s CP=%.2fV t=%lu\n",
                    (unsigned long)o.info, (unsigned long)c.efMs, st, m.cpHigh, (unsigned long)now);
      break;
    case Event::PulseEnd:
      LogOut.printf("[WAKE] Darbe %u/%u BITTI yontem=%s (%lu ms surdu), PWM nominale donuyor; st=%s CP=%.2fV "
                    "I=%.2f A t=%lu; %s\n",
                    (unsigned)s_wake.attempts(), (unsigned)c.maxAttempts,
                    evse_wake::method_name(s_wake.method()), (unsigned long)o.info, st, m.cpHigh, iA,
                    (unsigned long)now,
                    (s_wake.attempts() >= c.maxAttempts) ? "son darbe, C beklenecek"
                                                         : "C gelmezse sonraki darbe beklenecek");
      break;
    case Event::SettleEnd:
      LogOut.printf("[WAKE] F sonrasi oturma bitti, CP olcumu yeniden acik (deneme=%u st=%s CP=%.2fV I=%.2f A)\n",
                    (unsigned)s_wake.attempts(), st, m.cpHigh, iA);
      break;
    case Event::GaveUp:
      LogOut.printf("[WAKE] %lu darbe sonrasi arac sarja gecmedi; vazgecildi (PWM acik birakildi) st=%s CP=%.2fV I=%.2f A\n",
                    (unsigned long)o.info, st, m.cpHigh, iA);
      break;
    case Event::ResetCharging:
      LogOut.printf("[WAKE] Arac sarja gecti (deneme=%lu, st=%s)\n", (unsigned long)o.info, st);
      break;
    case Event::ResetLeftB:
      LogOut.printf("[WAKE] Arac B'den cikti, uyandirma iptal (deneme=%lu, st=%s)\n", (unsigned long)o.info, st);
      break;
    case Event::ResetNoPermit:
      LogOut.printf("[WAKE] Sarj izni kalkti, uyandirma iptal (deneme=%lu)\n", (unsigned long)o.info);
      break;
    case Event::ResetConfig:
      LogOut.printf("[WAKE] Ayar degisti/kapatildi, uyandirma iptal (deneme=%lu)\n", (unsigned long)o.info);
      break;
    default:
      break;
  }
}

// Fis cikinca (kararli A kenari) bir kez: seansa ait hicbir izin/mod sonraki takmaya tasinmaz.
static void onVehicleUnplugged()
{
  g_chargeMode = 0;                 // panel START/STOP ve bulut STOP modu biter
  g_manualStopAlertUntilMs = 0;
  g_manualStopAutoResumeAtMs = 0;
  cloud_on_vehicle_unplugged();     // bulut "start" onayi, guvenli durdurma, bulut STOP kilidi
  relay_force_off_now();            // role + latch RESET; A'da arac yok, akim yok (ark riski yok)
  s_wake.on_unplug();
  wake_set_cp_low(false);
  LogOut.println("[SEANS] Fis cikti (A): seans bitti; mod, bulut izni ve role kilidi sifirlandi");
}

// Hizli CP yolu (kPilotFastTickMs): CP olcumu + debounce burada; C->B / A / E-F'de kontaktor
// beklemeden acilir (relay_fast_guard). Debounce suresi = stableCount x kPilotFastTickMs
// (varsayilan 3 x 25 = 75 ms; eskiden 3 x 200 = 600 ms). IEC 61851-1 hedefi: C->B < 100 ms.
static constexpr uint32_t kPilotFastTickMs = 25;
static char s_prevStableState = 'A';

static void pilotFastTick()
{
  // Uyandirma F / E-F darbesi ve oturma suresinde CP olcumu dondurulur (son kararli state = B korunur).
  if (!s_wake.pilot_frozen()) pilot_update();
  PilotMeasurements m = pilot_get();
  char st = m.stateStable.length() > 0 ? m.stateStable[0] : 'F';
  // Fis cikti mi? Kararli (debounce'lu) A kenari: seans ve tum baslatma kalintilari sifirlanir.
  if (st == 'A' && s_prevStableState != 'A') onVehicleUnplugged();
  s_prevStableState = st;
  relay_fast_guard(st);
}

void setup()
{
  // Baslatma sirasi bilincli tutuldu:
  // 1) web/OTA, 2) OLED, 3) sensorler, 4) role, 5) pilot
  Serial.begin(115200);
  delay(200);
  // Wi-Fi log tamponunun (GET /api/log) ilk satiri: surum + reset nedeni + calisan bolum.
  {
    const esp_partition_t* runPart = esp_ota_get_running_partition();
    esp_reset_reason_t rr = esp_reset_reason();
#ifdef CURRENT_VERSION
    const char* fwVer = CURRENT_VERSION;
#else
    const char* fwVer = "?";
#endif
    LogOut.printf("[BOOT] RotosisEVSE v%s, reset=%s(%d), bolum=%s\n", fwVer, resetReasonName(rr), (int)rr,
                  runPart ? runPart->label : "?");
  }
  LogOut.println("BOOT OK");
  initFactoryButton();
  handleRescueFallbackIfNeeded();

  pinMode(STATE_LED_PIN, OUTPUT);
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(STATE_LED_PIN, LOW);
  digitalWrite(WIFI_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);


  // Web + OTA
  sched_init();   // planli sarj ayari (NVS), web_init /status'ta kullanir
  wake_cfg_init(); // arac uyandirma ayari (NVS), /api/wake
  web_init();
  cloud_init();   // sarj sunucusu istemcisi (anahtar yoksa kapali)
  OTA_Manager::begin(kOtaManifestUrl, kOtaAutoCheckIntervalMs, kGitHubFingerprint);

  // OLED
  oled_init();

  // Akim sensoru
  current_sensor_init();

  // Role
  relay_init();
  relay_set_auto_enabled(true);
  relay_set_min_switch_ms(100);   // C<->B testleri için

  // Test dirençleri

  // Pilot
  pilot_init();

  // Ilk PWM durumu
  pwmDutyPercent = amps_to_duty(g_targetCurrentLimitA);
  pwmEnabled = false;            // boot'ta A kabul, PWM kapalı
  applyPwmIfChanged();
}

void loop()
{
  // Ana dongu akisi:
  // 1) arka plan servisleri
  // 2) CP state olcumu
  // 3) enerji / seans hesabi
  // 4) ekran ve LED guncellemesi
  // 5) PWM ve role kararinin uygulanmasi

  serviceFactoryButton();
  clearQuickResetStreakIfStable();

  // Web sunucu dongusu
  web_loop();
  OTA_Manager::loop();

  // Akim sensoru dongusu
  current_sensor_loop();

  static uint32_t lastFast = 0;
  if (millis() - lastFast >= kPilotFastTickMs) {
    lastFast = millis();
    pilotFastTick();
  }

  static uint32_t last = 0;
  int safeLoopIntervalMs = loopIntervalMs;
  if (safeLoopIntervalMs < 20) safeLoopIntervalMs = 20;
  if (safeLoopIntervalMs > 2000) safeLoopIntervalMs = 2000;
  if (millis() - last < (uint32_t)safeLoopIntervalMs) return;
  last = millis();

  // CP olcumu ve A kenari hizli yolda (pilotFastTick); burada son kararli deger okunur.
  auto m = pilot_get();

  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);

  // Kablo: A değilse takılı kabul
  bool cableConnected = (m.stateStable != "A");

  // Sensor verisi burada okunur. OLED ve web tarafina giden anlik akim kaynagi burasidir.
  float ia = current_sensor_get_irms_a();
  float ib = current_sensor_get_irms_b();
  float ic = current_sensor_get_irms_c();
  bool relaySet = relay_get();
  bool chargingState = (m.stateStable == "C" || m.stateStable == "D");
  bool accountingEnabled = relaySet && pwmEnabled && chargingState;
  float rawIMax = ia;
  if (ib > rawIMax) rawIMax = ib;
  if (ic > rawIMax) rawIMax = ic;

  // Enerji ve akim sadece role cekili + PWM aktif + state C/D oldugunda gecerli sayilir.
  if (!accountingEnabled) {
    ia = 0.0f;
    ib = 0.0f;
    ic = 0.0f;
  }

  // Faz sayisi ve guc hesabi burada cikartilir.
  uint32_t nowMs = millis();
  float iMax = ia;
  if (ib > iMax) iMax = ib;
  if (ic > iMax) iMax = ic;

  const float phaseOnThresholdA = 0.90f;
  bool anyPhase = accountingEnabled && (iMax > phaseOnThresholdA);
  bool threePhase = accountingEnabled && ((ib > phaseOnThresholdA) || (ic > phaseOnThresholdA));
  float vLineLine = 400.0f;
  float vPhase = vLineLine / 1.732f;
  float powerW = 0.0f;
  // Seans baslatma / bitirme ve gecmise yazma mantigi burada yurur.
  if (anyPhase) {
    powerW = threePhase ? (vPhase * (ia + ib + ic)) : (vPhase * iMax);
  }
  float limitA = duty_to_amps((pwmDutyPercent > 0) ? pwmDutyPercent : amps_to_duty(g_targetCurrentLimitA));
  if (limitA <= 0.0f) limitA = g_targetCurrentLimitA;
  g_currentLimitA = limitA;

  if (anyPhase) {
    if (!g_sessionLive) {
      g_sessionLive = true;
      sessionStartMs = nowMs;
      sessionActiveMs = 0;
      sessionStartEnergyWh = energyWh;
      sessionPhaseMax = threePhase ? 3 : 1;
      g_sessionLiveStartSec = sessionStartMs / 1000;
    } else if (threePhase) {
      sessionPhaseMax = 3;
    }

    if (chargeSegmentStartMs == 0) {
      chargeSegmentStartMs = nowMs;
      lastEnergyMs = nowMs;
    }
    uint32_t dtMs = nowMs - lastEnergyMs;
    energyWh += (powerW * (dtMs / 3600000.0f));
    lastEnergyMs = nowMs;
    sessionActiveMs += dtMs;

    g_sessionLiveSeconds = sessionActiveMs / 1000;
    float liveWh = energyWh - sessionStartEnergyWh;
    if (liveWh < 0.0f) liveWh = 0.0f;
    g_sessionLiveEnergyKWh = liveWh / 1000.0f;
  } else {
    // Arac kilitli kapiyi acinca sarji bekletir (state B'ye doner). Fis takili ve PWM acik
    // oldugu surece seans DURAKLAMIS sayilir; akim tekrar gelince ayni seans surer.
    // Seans yalniz fis cikinca (state A) ya da STOP/izin kaldirma/hata ile biter.
    // Uyandirma darbesi (PWM kisa sure kapali) seansi bitirmez. A'ya inen her durum seansi bitirir.
    bool sessionPaused = g_sessionLive && cableConnected &&
                         (pwmEnabled || s_wake.active()) &&
                         m.stateStable != "E" && m.stateStable != "F";
    if (g_sessionLive && !sessionPaused) {
      uint32_t durSec = sessionActiveMs / 1000;
      float deltaWh = energyWh - sessionStartEnergyWh;
      if (deltaWh < 0.0f) deltaWh = 0.0f;
      float avgW = (durSec > 0) ? (deltaWh * 3600.0f / durSec) : 0.0f;

      int idx = g_histHead;
      g_histStartSec[idx] = sessionStartMs / 1000;
      g_histDurationSec[idx] = durSec;
      g_histEnergyKWh[idx] = deltaWh / 1000.0f;
      g_histAvgPowerW[idx] = avgW;
      g_histPhaseCount[idx] = sessionPhaseMax;
      g_histHead = (g_histHead + 1) % 20;
      if (g_histCount < 20) g_histCount++;

      g_sessionLive = false;
      g_sessionLiveSeconds = 0;
      g_sessionLiveEnergyKWh = 0.0f;
      g_sessionLiveStartSec = 0;
    }

    if (chargeSegmentStartMs != 0) {
      chargeAccumMs += (nowMs - chargeSegmentStartMs);
      chargeSegmentStartMs = 0;
    }
    lastEnergyMs = 0;
  }

  uint32_t chargeMs = chargeAccumMs;
  if (chargeSegmentStartMs != 0) {
    chargeMs += (nowMs - chargeSegmentStartMs);
  }
  uint32_t chargeSeconds = chargeMs / 1000;
  float energyKWh = energyWh / 1000.0f;

  g_powerW = powerW;
  g_energyKWh = energyKWh;
  g_chargeSeconds = chargeSeconds;
  g_phaseCount = threePhase ? 3 : 1;

  // OLED ekrani burada sadece okunabilir son verilerle beslenir.
  oled_draw(m.stateStable, ia, ib, ic, powerW, energyKWh, chargeSeconds, relaySet, staOk, cableConnected);
  // LED map:
  // - STATE_LED_PIN (GPIO8): C'de normal blink, D'de cift flash
  // - WIFI_LED_PIN  (GPIO17): Wi-Fi bagliyken ON
  // - ERROR_LED_PIN (GPIO18): state E/F iken ON
  static uint32_t ledTickMs = 0;
  static bool ledPhase = false;
  const uint32_t ledBlinkMs = 200;
  uint32_t ledNowMs = millis();
  if (ledNowMs - ledTickMs >= ledBlinkMs) {
    ledTickMs = ledNowMs;
    ledPhase = !ledPhase;
  }
  bool stateBlink = false;
  if (m.stateStable == "C") {
    stateBlink = ledPhase;
  } else if (m.stateStable == "D") {
    const uint32_t dCycleMs = 1200;
    const uint32_t dPhaseMs = ledNowMs % dCycleMs;
    stateBlink = (dPhaseMs < 120) || (dPhaseMs >= 240 && dPhaseMs < 360);
  }
  bool manualStopAlertOn = (g_manualStopAlertUntilMs != 0 && ((int32_t)(g_manualStopAlertUntilMs - ledNowMs) > 0));
  bool errorOn = (m.stateStable == "E" || m.stateStable == "F" || manualStopAlertOn);
  digitalWrite(STATE_LED_PIN, stateBlink ? HIGH : LOW);
  digitalWrite(WIFI_LED_PIN, staOk ? HIGH : LOW);
  digitalWrite(ERROR_LED_PIN, errorOn ? HIGH : LOW);

  // Manuel stop 60 saniye sonra otomatik AUTO moduna donsün.
  if (g_chargeMode == 2 &&
      g_manualStopAutoResumeAtMs != 0 &&
      (int32_t)(ledNowMs - g_manualStopAutoResumeAtMs) >= 0) {
    g_chargeMode = 0;
    g_manualStopAutoResumeAtMs = 0;
  }



  // IEC state'e gore hedef PWM durumu burada belirlenir.
  bool nextPwmEnabled = false;
  int nextDuty = 0;

  if (m.stateStable == "A" || m.stateStable == "E" || m.stateStable == "F") {
    // Araç yok veya hata: PWM Kapalı (Sabit 12V)
    nextPwmEnabled = false;
    nextDuty = 0;
  }
  else if (m.stateStable == "B") {
    // Araç bağlandı: PWM başlat
    nextPwmEnabled = true;
    nextDuty = amps_to_duty(g_targetCurrentLimitA);
  }
  else if (m.stateStable == "C" || m.stateStable == "D") {
    // Şarjda: PWM devam
    nextPwmEnabled = true;
    nextDuty = amps_to_duty(g_targetCurrentLimitA);
  }

  // Web arayuzundeki manuel START / STOP istegi burada ana kararin ustune yazilir.
  if (g_chargeMode == 2) { // STOP
    nextPwmEnabled = false;
    nextDuty = 0;
  } else if (g_chargeMode == 1) { // START
    if (m.stateStable == "B" || m.stateStable == "C" || m.stateStable == "D") {
      nextPwmEnabled = true;
      nextDuty = amps_to_duty(g_targetCurrentLimitA);
    }
  }

  // Bulut guvenli modu (requireCloudStart) ve planli sarj (yasakli saat araligi).
  // Ikisi de kapaliyken chargePermit hep true: mevcut davranis degismez.
  bool schedBlocked = sched_loop();
  bool chargePermit = cloud_loop(m.stateStable, ia, ib, ic, powerW, energyKWh, chargeSeconds, schedBlocked);
  if (!chargePermit || g_chargeMode == 2) {  // bulut STOP bu turda geldiyse de hemen kapat
    nextPwmEnabled = false;
    nextDuty = 0;
  }

  // Arac uyandirma: izinler sonrasi PWM isteniyor ama arac uzun beklemeden sonra B'de kaldiysa
  // secilen yontemle darbe (B1 / F / PWM5). Yalniz state B + kontaktor birakik + akim yokken.
  if (wake_cfg_gen() != s_wakeCfgGen) {
    s_wakeCfgGen = wake_cfg_gen();
    evse_wake::Output co;
    co.ev = s_wake.set_config(wake_cfg_get());
    wake_log(co, m, rawIMax, millis());
  }
  evse_wake::Input wi;
  wi.st = m.stateStable.length() > 0 ? m.stateStable[0] : 'F';
  wi.pwmWant = nextPwmEnabled;
  wi.contactsOpen = relay_contacts_open();
  wi.currentA = rawIMax;
  wi.now = millis();
  evse_wake::Output wo = s_wake.step(wi);
  wake_log(wo, m, rawIMax, wi.now);
  wake_set_cp_low(wo.forceCpLow);
  bool wakeHold = wo.pwmOff || wo.dutyOverride > 0;
  if (wo.pwmOff) {
    nextPwmEnabled = false;
    nextDuty = 0;
  } else if (wo.dutyOverride > 0 && nextPwmEnabled) {
    nextDuty = wo.dutyOverride;   // PWM5: %5 (kontaktor bu duty'de kapanmaz, relay_core::should_close)
  }

  // Sadece değişiklik varsa uygula
  if (nextPwmEnabled != pwmEnabled || nextDuty != pwmDutyPercent || s_wakeCpDirty) {
    s_wakeCpDirty = false;
    pwmEnabled = nextPwmEnabled;
    pwmDutyPercent = nextDuty;
    pilot_apply_pwm();

    LogOut.printf("DURUM DEĞİŞTİ: State=%s, PWM=%s, Duty=%%%d, CP=%s, I=%.2f A, wake=%s(%u/%u)%s\n",
                  m.stateStable.c_str(),
                  pwmEnabled ? "ON" : "OFF",
                  pwmDutyPercent,
                  s_wakeCpLow ? "LOW" : "normal",
                  rawIMax,
                  evse_wake::phase_name(s_wake.phase()),
                  (unsigned)s_wake.attempts(), (unsigned)s_wake.config().maxAttempts,
                  wakeHold ? " [uyandirma darbesi]" : "");
  }

  // Manuel stop modunda latch SET takibini durdur.
  // Izin yokken (bulut onayi / planli sarj) de latch SET atlanir. Acma her durumda relay_update_auto'da.
  // Uyandirma darbesi/oturma suresinde latch SET yapilmaz (PWM yokken kontaktor SET edilmez).
  if (g_chargeMode != 2 && chargePermit && !s_wake.holding()) {
    relay_handle_state_pulse(m.stateStable, pwmEnabled, pwmDutyPercent);
  }

  // Son role karari burada verilir (acma: akim ~0 / zaman asimi / acil, bkz. relay_core).
  relay_update_auto(m.stateStable, pwmEnabled, pwmDutyPercent);
}
