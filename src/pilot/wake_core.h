#pragma once
// Arac uyandirma durum makinesi (donanimdan bagimsiz; native testli: test/test_wake).
//
// Senaryo: arac B'de uzun sure PWM'siz bekledi (planli sarj / bulut onayi); izin gelip PWM acilinca
// arac uykuda oldugu icin B->C yapmiyor. kArmMinIdleBMs'den uzun PWM'siz B beklemesinden sonra PWM
// acildiginda kurulur; waitMs icinde C gelmezse secilen yontemle darbe verilir.
//
// Yontemler (deneme basina, Config::seq; liste bitince son yontem tekrarlanir):
//   B1   : PWM kesilir (CP sabit +12V/+9V, IEC 61851-1 B2->B1->B2 "EVSE yeniden hazir").
//   F    : CP sabit LOW (-12V/0V, state F) -- Phoenix Contact EV Charge Control: 3 sn F, sonra PWM.
//   PWM5 : PWM %5 (pwm5Ms), ardindan E/F (efMs >= T_step_EF), sonra nominal PWM
//          (ISO 15118-3 V2G3-M06-05 sirasi).
// Darbe YALNIZ durum B + kontaktor kesin acik + akim < kMaxIdleCurrentA iken baslar.
#include <stddef.h>
#include <stdint.h>

namespace evse_wake {

enum class Method : uint8_t { B1Pulse = 0, FPulse = 1, Pwm5 = 2 };  // (B1 Arduino binary.h makrosu)

constexpr uint8_t kMaxSeq = 6;
constexpr uint32_t kArmMinIdleBMs = 30000;  // PWM acilmadan once PWM'siz B'de en az bu kadar
constexpr uint32_t kSettleMs = 1000;        // F / E-F adimindan sonra CP olcumunu yok sayma
constexpr float kMaxIdleCurrentA = 0.5f;    // darbe oncesi akim bunun altinda olmali
constexpr int kPwm5Duty = 5;

struct Config {
  bool enabled = true;
  uint8_t seqLen = 1;
  Method seq[kMaxSeq] = {Method::B1Pulse, Method::B1Pulse, Method::B1Pulse, Method::B1Pulse, Method::B1Pulse, Method::B1Pulse};
  uint8_t maxAttempts = 3;
  uint32_t waitMs = 25000;   // PWM acik + B iken bu surede C gelmezse darbe (kWakeWaitMs)
  uint32_t b1Ms = 4000;      // B1 darbesi (PWM kapali) suresi
  uint32_t fMs = 3000;       // F darbesi suresi
  uint32_t pwm5Ms = 3000;    // PWM5 yolunda %5 suresi
  uint32_t efMs = 2000;      // PWM5 yolunda E/F suresi (T_step_EF)
};

// Varsayilan = v1.1.69 davranisi: B1 x3, 4 sn darbe, 25 sn bekleme.
Config default_config();
bool validate(const Config& c, const char** err);

const char* method_name(Method m);           // "B1" / "F" / "PWM5"
// "B1,B1,F" gibi liste (bosluk ve buyuk/kucuk harf serbest). Hatada false, c degismez.
bool parse_seq(const char* s, Config* c);
void format_seq(const Config& c, char* out, size_t outLen);
Method method_for_attempt(const Config& c, uint8_t attempt1);  // attempt1: 1'den baslar

// NVS blob'u (surum baytli).
constexpr size_t kBlobLen = 30;
void serialize(const Config& c, uint8_t out[kBlobLen]);
bool deserialize(const uint8_t* in, size_t len, Config* out);

enum class Phase : uint8_t { Idle, Waiting, PulseB1, PulseF, Pwm5, StepEF, Settle };
const char* phase_name(Phase p);

enum class Event : uint8_t {
  None,
  Armed,          // info = PWM'siz bekleme ms
  NotArmed,       // info = PWM'siz bekleme ms (kArmMinIdleBMs'den kisa)
  Deferred,       // guvenlik: kontaktor acik degil ya da akim var
  PulseStart,     // method() / attempts()
  StepEF,         // PWM5 bitti, E/F basladi; info = %5 suresi
  PulseEnd,       // info = darbe (ya da E/F adimi) suresi
  SettleEnd,
  GaveUp,         // info = deneme sayisi
  ResetCharging,  // arac C/D'ye gecti; info = deneme sayisi
  ResetLeftB,     // A/E/F; info = deneme sayisi
  ResetNoPermit,  // izin kalkti; info = deneme sayisi
  ResetConfig,    // ayar degisti / kapatildi; info = deneme sayisi
};

struct Input {
  char st;            // kararli CP durumu
  bool pwmWant;       // tum izinlerden sonra istenen PWM
  bool contactsOpen;  // kontaktor kesin acik
  float currentA;     // olculen en buyuk faz akimi
  uint32_t now;
};

struct Output {
  bool pwmOff = false;      // bu turda PWM kapali tut
  bool forceCpLow = false;  // CP sabit LOW (F / E-F)
  int dutyOverride = -1;    // >0: PWM acikken bu duty kullanilir (PWM5)
  Event ev = Event::None;
  uint32_t info = 0;
};

class Machine {
 public:
  explicit Machine(const Config& c = default_config()) : cfg_(c) {}
  // Ayar degisir; makine calisiyorsa Idle'a doner (ResetConfig doner, yoksa None).
  Event set_config(const Config& c);
  const Config& config() const { return cfg_; }
  Output step(const Input& in);
  // Fis cikti: tum durum sifirlanir.
  void on_unplug();

  Phase phase() const { return phase_; }
  uint8_t attempts() const { return attempts_; }
  Method method() const { return method_; }
  bool active() const { return phase_ != Phase::Idle; }
  // CP olcumu anlamsiz (F / E-F ve sonrasindaki oturma): kararli durum dondurulur.
  bool pilot_frozen() const { return phase_ == Phase::PulseF || phase_ == Phase::StepEF || phase_ == Phase::Settle; }
  // Darbe ya da oturma suresinde (kontaktor kapatilmamali).
  bool holding() const { return phase_ != Phase::Idle && phase_ != Phase::Waiting; }

 private:
  uint32_t reset_();
  Config cfg_;
  Phase phase_ = Phase::Idle;
  Method method_ = Method::B1Pulse;
  uint8_t attempts_ = 0;
  uint32_t t0_ = 0;
  uint32_t bNoPwmSince_ = 0;
  bool prevPwmWant_ = false;
  bool unsafeLogged_ = false;
};

}  // namespace evse_wake
