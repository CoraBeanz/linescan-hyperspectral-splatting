// Pin map for an ESP32 DevKit (esp32dev) wired to a BIGTREETECH TMC2209
// module. The wiring table in firmware/README.md follows this file.
#pragma once

#include <stdint.h>

// TMC2209 step/dir inputs. EN is active low: the motor is powered only while
// it is low. Fit a 10k pull-up from EN to 3.3 V so the motor stays off while
// the ESP32 boots and its pins float.
constexpr uint8_t kPinStep = 26;
constexpr uint8_t kPinDir = 25;
constexpr uint8_t kPinEn = 27;

// TMC2209 single-wire UART on PDN_UART: RX straight to the pin, TX through a
// 1k resistor (the TMC2209 and the ESP32 take turns driving the one wire).
constexpr uint8_t kPinTmcRx = 16;
constexpr uint8_t kPinTmcTx = 17;
constexpr uint32_t kTmcBaud = 115200;
constexpr uint8_t kTmcAddress = 0;  // MS1 and MS2 low
constexpr float kTmcRSense = 0.11f;  // BTT TMC2209 sense resistors, ohms

// A3144 hall switch, open collector: low when the magnet is at the sensor.
// The internal pull-up is weak for a cable up the arm; add 10k to 3.3 V.
constexpr uint8_t kPinHall = 33;

// Timing outputs for a scope, logic analyser or the Jetson's GPIO (3.3 V).
constexpr uint8_t kPinMoving = 18;  // high while the mirror moves or settles
constexpr uint8_t kPinTrig = 19;    // short pulse when a scan line is ready

constexpr uint8_t kPinLed = 2;  // the DevKit's blue LED

// USB serial link to the host.
constexpr uint32_t kHostBaud = 921600;
