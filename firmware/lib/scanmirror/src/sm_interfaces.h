// What the controller needs from the board. src/main.cpp implements these
// for the ESP32; the unit tests implement them with a simulated rig.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "sm_config.h"

namespace sm {

struct DriverStatus {
  uint32_t ioin;        // IOIN register: the pins as the driver sees them
  uint8_t gstat;        // bit 0 reset, bit 1 drv_err, bit 2 uv_cp
  uint32_t drv_status;  // DRV_STATUS register
  uint16_t mscnt;       // microstep counter, 0..1023 per electrical cycle
};

// IOIN fields (register 0x06).
inline uint8_t ioinVersion(uint32_t ioin) { return static_cast<uint8_t>(ioin >> 24); }  // 0x21 on a TMC2209
constexpr uint32_t kIoinEnn = 1u << 0;  // the EN pin: 1 = motor outputs off

// DRV_STATUS bits (TMC2209 datasheet, register 0x6F).
constexpr uint32_t kDrvOtpw = 1u << 0;   // overtemperature pre-warning
constexpr uint32_t kDrvOt = 1u << 1;     // overtemperature shutdown
constexpr uint32_t kDrvS2g = 3u << 2;    // short to ground, coil A or B
constexpr uint32_t kDrvS2vs = 3u << 4;   // short across the low side, coil A or B
constexpr uint32_t kDrvOl = 3u << 6;     // open load (only valid while moving)
constexpr uint32_t kDrvStealth = 1u << 30;
constexpr uint32_t kDrvStst = 1u << 31;  // standstill
inline uint32_t drvCsActual(uint32_t drv_status) { return (drv_status >> 16) & 0x1F; }

constexpr uint8_t kGstatReset = 1;
constexpr uint8_t kGstatDrvErr = 2;
constexpr uint8_t kGstatUvCp = 4;

class Driver {
 public:
  virtual ~Driver() {}
  // Write every setting from cfg (microsteps, currents, chopper mode) and
  // check that the driver took them. False if it didn't answer.
  virtual bool configure(const Config& cfg) = 0;
  // Standstill current: the full run current (during scans) or cfg.ihold %.
  virtual bool setFullHold(bool full, const Config& cfg) = 0;
  // Read GSTAT (and clear it), DRV_STATUS, IOIN and MSCNT.
  virtual bool read(DriverStatus* st) = 0;
  // The EN pin: false keeps the motor outputs off.
  virtual void enable(bool on) = 0;
};

class Port {
 public:
  virtual ~Port() {}
  virtual int64_t nowUs() = 0;  // same clock as the timestamps MirrorCore gets
  // Lock out the timer interrupt around MirrorCore calls.
  virtual void lock() = 0;
  virtual void unlock() = 0;
  virtual void writeLine(const char* s, size_t n) = 0;  // without the newline
  virtual bool saveConfig(const Config& c) = 0;
  virtual bool loadConfig(Config* c) = 0;
  virtual void setDirInvert(bool invert) = 0;  // applied to the DIR pin
  virtual void reboot() = 0;
};

}  // namespace sm
