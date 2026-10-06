// Controller: the main-loop side. Parses commands, drives MirrorCore, runs
// the homing sequence, watches the TMC2209 and prints replies and events.
// The serial protocol it speaks is described in firmware/PROTOCOL.md.
#pragma once

#include <stdint.h>

#include "sm_cmdline.h"
#include "sm_config.h"
#include "sm_core.h"
#include "sm_interfaces.h"

namespace sm {

class Controller {
 public:
  Controller(MirrorCore& core, Port& port, Driver& drv) : core_(core), port_(port), drv_(drv) {}

  // Load the stored settings, probe the driver and print EV BOOT.
  void begin(const char* reset_reason);
  // Handle one received line (without its newline); the buffer is modified.
  void handleLine(char* line);
  // Drain the core's events, run homing and the driver watchdog. Call often.
  void poll();

  const Config& config() const { return cfg_; }
  bool homed() const { return homed_; }
  bool enabled() const { return state_ != kDisabled && state_ != kFault; }
  bool faulted() const { return state_ == kFault; }
  const char* stateName() const;

 private:
  enum State : uint8_t { kDisabled, kIdle, kMoving, kStopping, kHoming, kScanning, kFault };
  enum DrvState : uint8_t { kDrvUnknown, kDrvOk, kDrvNoResp, kDrvBypass };
  enum HomeStep : uint8_t { kHomeOff, kHomeLeave, kHomeSeek, kHomeBackoff, kHomeApproach, kHomeCross, kHomePark };

  // Commands. Each parses and checks everything before acting.
  void cmdInfo();
  void cmdPing();
  void cmdStatus();
  void cmdEnable();
  void cmdDisable();
  void cmdHome();
  void cmdMove();
  void cmdStop();
  void cmdScan();
  void cmdNudge();
  void cmdPeriod();
  void cmdZero();
  void cmdCfg();
  void cmdSave();
  void cmdDefaults();
  void cmdDrv();
  void cmdReboot();
  void cmdHelp();

  // Replies: exactly one per command.
  bool argsDone();  // false (and ERR sent) if a key was not recognised
  bool requireState(bool ok);
  void replyOk(LineBuf& b);
  void replyErr(const char* code, const char* msg);
  void send(const LineBuf& b);

  void onEvent(const Event& e);
  void homeStart();
  void homeOnHall(const Event& e);
  void homeOnMoveDone(const Event& e);
  void homeMove(int32_t target, const Profile& p);
  void homeStop();
  void homeFail(const char* reason);
  void checkDriver(int64_t now);
  void judgeDriver(const DriverStatus& st, int64_t now);
  void fault(const char* code, const char* msg);
  void stopEverything(const char* why);
  void applyCoreConfig();
  bool applyDriverConfig();
  void restoreHold();

  Profile profile(int32_t vmax, int32_t vstart, int32_t accel) const;
  Profile slewProfile() const { return profile(cfg_.vmax, cfg_.vstart, cfg_.accel); }
  CoreSnapshot snapshot();
  uint16_t newTag();
  bool busy() const {
    return state_ == kMoving || state_ == kStopping || state_ == kHoming || state_ == kScanning;
  }

  MirrorCore& core_;
  Port& port_;
  Driver& drv_;
  Config cfg_ = {};
  Args args_;

  State state_ = kDisabled;
  DrvState drv_state_ = kDrvUnknown;
  bool homed_ = false;
  const char* fault_code_ = "none";
  uint16_t tag_seq_ = 0;
  uint16_t move_tag_ = 0;
  uint16_t stop_tag_ = 0;
  bool scan_sweep_ = false;
  int32_t scan_step_ = 0;
  bool hold_full_ = false;
  int64_t last_drv_check_ = 0;
  bool otpw_warned_ = false;
  uint32_t dropped_reported_ = 0;

  HomeStep home_step_ = kHomeOff;
  uint16_t home_tag_ = 0;
  int64_t home_deadline_ = 0;
  int home_tries_ = 0;
  bool home_seen_ = false;
  bool home_have_b_ = false;
  int32_t home_seek_edge_ = 0;
  int32_t home_edge_a_ = 0;
  int32_t home_edge_b_ = 0;
  int32_t home_shift_ = 0;
};

}  // namespace sm
