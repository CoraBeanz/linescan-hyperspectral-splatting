// ESP32 side of the scan-mirror firmware: the 50 us timer interrupt that
// steps the motor, the TMC2209 on UART2, settings in flash and the USB serial
// link to the host. Everything that decides what to do lives in
// lib/scanmirror, which the native tests run on a simulated rig.
#include <Arduino.h>
#include <Preferences.h>
#include <TMCStepper.h>
#include <esp_rom_sys.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>

#include "board_pins.h"
#include "sm_controller.h"
#include "sm_core.h"

namespace {

static_assert(kPinStep < 32 && kPinDir < 32 && kPinMoving < 32 && kPinTrig < 32,
              "the interrupt drives its outputs through the GPIO 0-31 registers");

sm::MirrorCore g_core;
// Guards g_core between the interrupt (core 0) and loop() (core 1).
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_dir_invert = false;
volatile bool g_timer_running = false;

// Where SAVE keeps the settings in flash (NVS).
constexpr char kPrefsNamespace[] = "scanmirror";
constexpr char kPrefsKey[] = "cfg";

inline void IRAM_ATTR setPin(uint8_t pin, bool high) {
  if (high) {
    GPIO.out_w1ts = 1u << pin;
  } else {
    GPIO.out_w1tc = 1u << pin;
  }
}

inline bool IRAM_ATTR readPin(uint8_t pin) {
  return pin < 32 ? ((GPIO.in >> pin) & 1u) != 0 : ((GPIO.in1.data >> (pin - 32)) & 1u) != 0;
}

// Every 50 us, on core 0: one MirrorCore tick, then the pins it asks for.
void IRAM_ATTR onTick() {
  static bool dir_level = false;
  const int64_t now = esp_timer_get_time();
  const bool hall = readPin(kPinHall);
  portENTER_CRITICAL_ISR(&g_mux);
  const sm::TickOut out = g_core.tick(now, hall);
  portEXIT_CRITICAL_ISR(&g_mux);

  if (out.step != 0) {
    const bool level = (out.step > 0) != g_dir_invert;
    if (level != dir_level) {
      setPin(kPinDir, level);
      dir_level = level;
      esp_rom_delay_us(1);  // DIR settles before the STEP edge
    }
    setPin(kPinStep, true);
  }
  setPin(kPinMoving, out.moving);
  setPin(kPinTrig, out.trig);
  if (out.step != 0) {
    esp_rom_delay_us(1);  // the TMC2209 needs STEP high for 100 ns or more
    setPin(kPinStep, false);
  }
}

// An interrupt runs on the core that set it up, so set the timer up from a
// short-lived task on core 0. loop() runs on core 1, so serial traffic and
// the TMC2209's UART waits never delay a step.
void timerSetupTask(void*) {
  hw_timer_t* timer = timerBegin(0, 80, true);  // 80 MHz APB clock / 80: 1 us per count
  timerAttachInterrupt(timer, &onTick, false);
  timerAlarmWrite(timer, sm::kTickUs, true);
  timerAlarmEnable(timer);
  g_timer_running = true;
  vTaskDelete(nullptr);
}

const char* resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

// ---------------------------------------------------------------- board

class EspPort : public sm::Port {
 public:
  int64_t nowUs() override { return esp_timer_get_time(); }
  void lock() override { portENTER_CRITICAL(&g_mux); }
  void unlock() override { portEXIT_CRITICAL(&g_mux); }

  void writeLine(const char* s, size_t n) override {
    Serial.write(reinterpret_cast<const uint8_t*>(s), n);
    Serial.write('\n');
  }

  bool saveConfig(const sm::Config& c) override {
    Preferences prefs;
    if (!prefs.begin(kPrefsNamespace, false)) return false;
    const bool ok = prefs.putBytes(kPrefsKey, &c, sizeof(c)) == sizeof(c);
    prefs.end();
    return ok;
  }

  bool loadConfig(sm::Config* c) override {
    Preferences prefs;
    if (!prefs.begin(kPrefsNamespace, true)) return false;  // nothing saved yet
    const bool ok = prefs.getBytesLength(kPrefsKey) == sizeof(*c) &&
                    prefs.getBytes(kPrefsKey, c, sizeof(*c)) == sizeof(*c);
    prefs.end();
    return ok;
  }

  void setDirInvert(bool invert) override { g_dir_invert = invert; }

  void reboot() override {
    Serial.flush();
    delay(20);
    ESP.restart();
  }
};

// The TMC2209 over its single-wire UART. TMCStepper waits a few ms for each
// register, which only ever holds up loop(), never the step interrupt.
class Tmc2209 : public sm::Driver {
 public:
  Tmc2209() : tmc_(&Serial2, kTmcRSense, kTmcAddress) {}

  bool configure(const sm::Config& c) override {
    if (tmc_.version() != 0x21 || tmc_.CRCerror) return false;  // nobody there, or not a TMC2209
    tmc_.GSTAT(0);                 // clear the reset and error flags
    tmc_.begin();                  // PDN_UART is the UART; microsteps come from MRES
    tmc_.I_scale_analog(false);    // current from IRUN/IHOLD, not from the VREF pot
    tmc_.en_spreadCycle(c.stealth == 0);
    tmc_.TPWMTHRS(0);              // when in StealthChop, stay in it at every speed
    tmc_.toff(3);
    tmc_.blank_time(24);
    tmc_.intpol(true);             // smooth each microstep into 256
    tmc_.mres(mres(c.usteps));
    tmc_.iholddelay(8);            // ease down to the hold current
    tmc_.TPOWERDOWN(20);           // after 0.44 s at standstill
    tmc_.rms_current(static_cast<uint16_t>(c.irun), c.ihold / 100.0f);

    // Writes are not acknowledged, so read two registers back.
    const uint32_t gconf = tmc_.GCONF();
    if (tmc_.CRCerror) return false;
    const uint32_t chopconf = tmc_.CHOPCONF();
    if (tmc_.CRCerror) return false;
    // GCONF: I_scale_analog (bit 0) off, en_SpreadCycle (bit 2) as asked,
    // pdn_disable (bit 6) and mstep_reg_select (bit 7) on.
    const uint32_t want_gconf = 0xC0u | (c.stealth == 0 ? 0x04u : 0u);
    return (gconf & 0xC5u) == want_gconf && ((chopconf >> 24) & 0xFu) == mres(c.usteps);
  }

  bool setFullHold(bool full, const sm::Config& c) override {
    tmc_.rms_current(static_cast<uint16_t>(c.irun), full ? 1.0f : c.ihold / 100.0f);
    return true;  // writes are not acknowledged
  }

  bool read(sm::DriverStatus* s) override {
    s->gstat = tmc_.GSTAT();
    if (tmc_.CRCerror) return false;
    if (s->gstat != 0) tmc_.GSTAT(0);  // write-1-to-clear
    s->drv_status = tmc_.DRV_STATUS();
    if (tmc_.CRCerror) return false;
    s->ioin = tmc_.IOIN();
    if (tmc_.CRCerror) return false;
    s->mscnt = tmc_.MSCNT();
    return !tmc_.CRCerror;
  }

  void enable(bool on) override { digitalWrite(kPinEn, on ? LOW : HIGH); }

 private:
  // MRES: 0 = 256 microsteps per full step ... 8 = full steps.
  static uint8_t mres(int32_t usteps) {
    uint8_t m = 8;
    while (usteps > 1 && m > 0) {
      usteps >>= 1;
      --m;
    }
    return m;
  }

  TMC2209Stepper tmc_;
};

EspPort g_port;
Tmc2209 g_driver;
sm::Controller g_ctl(g_core, g_port, g_driver);

// ---------------------------------------------------------------- serial input

constexpr size_t kMaxLine = 255;
char g_line[kMaxLine + 1];
size_t g_line_len = 0;
bool g_line_overflow = false;

// Collects one command per line ("\n", "\r" or both end it). Backspace works,
// so a plain serial terminal can drive it too.
void readSerial() {
  for (int budget = 256; budget > 0 && Serial.available() > 0; --budget) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (g_line_overflow) {
        static const char kTooLong[] = "ERR ? code=syntax msg=line longer than 255 characters";
        g_port.writeLine(kTooLong, sizeof(kTooLong) - 1);
      } else if (g_line_len > 0) {
        g_line[g_line_len] = '\0';
        g_ctl.handleLine(g_line);
      }
      g_line_len = 0;
      g_line_overflow = false;
    } else if (c == '\b' || c == 0x7F) {
      if (g_line_len > 0) --g_line_len;
    } else if (g_line_len < kMaxLine) {
      g_line[g_line_len++] = static_cast<char>(c);
    } else {
      g_line_overflow = true;
    }
  }
}

// LED: off while disabled, on while enabled, blinking on a driver fault.
void updateLed() {
  static int last = -1;
  const int on = g_ctl.faulted() ? static_cast<int>((millis() / 250) % 2) : g_ctl.enabled() ? 1 : 0;
  if (on != last) {
    digitalWrite(kPinLed, on ? HIGH : LOW);
    last = on;
  }
}

}  // namespace

void setup() {
  // Motor off before anything else.
  digitalWrite(kPinEn, HIGH);
  pinMode(kPinEn, OUTPUT);
  digitalWrite(kPinStep, LOW);
  pinMode(kPinStep, OUTPUT);
  digitalWrite(kPinDir, LOW);
  pinMode(kPinDir, OUTPUT);
  pinMode(kPinMoving, OUTPUT);
  pinMode(kPinTrig, OUTPUT);
  pinMode(kPinLed, OUTPUT);
  pinMode(kPinHall, INPUT_PULLUP);

  Serial.setRxBufferSize(1024);
  Serial.begin(kHostBaud);
  Serial2.begin(kTmcBaud, SERIAL_8N1, kPinTmcRx, kPinTmcTx);

  // Settings and the driver first, so the first tick already uses them.
  g_ctl.begin(resetReason());
  xTaskCreatePinnedToCore(timerSetupTask, "tick_setup", 4096, nullptr, 5, nullptr, 0);
  while (!g_timer_running) delay(1);
}

void loop() {
  readSerial();
  g_ctl.poll();
  updateLed();
}
