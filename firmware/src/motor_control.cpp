// Created by Jörg Ebeling on 2025-10-18.
// Copyright (c) 2025 Jörg Ebeling for OpenMower/Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons
// Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't
// try to sell the design or products based on it without getting my consent
// first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "motor_control.hpp"

#include <modm/architecture.hpp>
#include <modm/platform.hpp>

#include "board.hpp"
#include "config.h"

using namespace Board;

namespace motor_control {

// Internal state.
namespace {
volatile float duty_ = 0.0f;
float duty_setpoint_ = 0.0f;
volatile uint32_t sa_ticks_ = 0;
volatile uint32_t sa_tacho_ = 0;
volatile uint16_t rpm_ = 0;
uint8_t motor_stopped_cycles_ = 0;
}  // namespace

/**
 * @brief SA (Hall sensor) Capture/Compare ISR.
 *
 * Timer configuration:
 * - 16-bit up-counter (0xFFFF max)
 *
 * Overflow handling: When timer overflows between captures, the difference
 * calculation automatically handles it via 32-bit arithmetic.
 */
MODM_ISR(TIM1_CC) {
  static uint8_t skip_initial = 10;
  static uint16_t last_cc_value = 0;
  uint16_t current_cc_value = Timer1::getCompareValue<motor::SATimChan>();

  // Didn't got rid of initial spurious interrupts yet?
  if (skip_initial > 0) {
    skip_initial--;
    last_cc_value = current_cc_value;
    return;
  }

  Timer1::acknowledgeInterruptFlags(motor::SACaptureFlag);
  sa_tacho_++;

  if (Timer1::getInterruptFlags() & Timer1::InterruptFlag::Update) {
    Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
    // Timer overflow occurred between captures
    sa_ticks_ = current_cc_value + (0xffff - last_cc_value);
  } else {
    // Normal case: no overflow
    sa_ticks_ = current_cc_value - last_cc_value;
  }
  last_cc_value = current_cc_value;

  // RPM calculation with new timer configuration:
  // Timer frequency = 24MHz / 220 = 109'090 Hz = 9.1667 μs per tick
  // Hall sensors: 4 poles * 2 edges per pole = 8 edges per revolution
  //
  // Time per revolution = sa_ticks_ * 9.1667 μs * 8 edges = sa_ticks_ × 73.333 μs
  // RPM = 60 / (time per revolution in minutes) = 60 / (sa_ticks_ / 109'090 / 60)
  // Simplified: RPM = (109'090 × 60) / (sa_ticks_ × 8)

  // Break down calculation for clarity:
  // timer_frequency = Board::SystemClock::Timer1 / (SA_TIMER_PRESCALER + 1);
  // seconds_per_minute = 60;
  // edges_per_revolution = 8;
  // schwund = 2;
  // rpm_divisor = (timer_frequency * seconds_per_minute) / edges_per_revolution;
  constexpr uint32_t rpm_divisor = ((Board::SystemClock::Timer1 / (SA_TIMER_PRESCALER + 0)) * 60) / 8 * 2;

  if (sa_ticks_ > 0) {
    rpm_ = rpm_divisor / sa_ticks_;
  } else {
    rpm_ = 0;
  }
}

void Init() {
  // SA (Hall) input - Capture/Compare timer.
  Timer1::connect<motor::SATimChan>();
  Timer1::enable();
  Timer1::setMode(Timer1::Mode::UpCounter);
  Timer1::setPrescaler(SA_TIMER_PRESCALER);
  Timer1::setOverflow(0xFFFF);
  Timer1::configureInputChannel<motor::SATimChan>(Timer1::InputCaptureMapping::InputOwn,
                                                  Timer1::InputCapturePrescaler::Div1,
                                                  Timer1::InputCapturePolarity::Both, SA_TIMER_INPUT_FILTER);
  Timer1::enableInterrupt(motor::SACaptureInterrupt);
  Timer1::enableInterruptVector(motor::SACaptureInterrupt, true, 21);
  Timer1::applyAndReset();
  Timer1::start();
}

void SetDutySetpoint(float duty) {
  duty_setpoint_ = duty;
}

float GetDuty() {
  return duty_;
}

// Update motor state based on faults, shutdown signal and current tacho.
// Handles also BRK release once safely stopped.
void UpdateMotorState(bool has_faults, bool is_shutdown, uint32_t last_tacho) {
  float new_duty = (has_faults || is_shutdown) ? 0.0f : duty_setpoint_;

  // Handle safe stop and BRK release
  if (new_duty == 0.0f) {
    UpdateMotorStoppedState(last_tacho);
  }

  if (new_duty == duty_) return;

  if (new_duty == 0.0f) {
    // Stop motor.
    motor::RS::set();   // !RS off
    motor::Brk::set();  // Brake
  } else {
    // Start motor.
    motor::Brk::reset();  // Release brake
    motor::RS::reset();   // !RS on
  }
  duty_ = new_duty;
  motor_stopped_cycles_ = 0;
}

uint32_t GetSaTacho() {
  return sa_tacho_;
}

uint32_t GetSaTicks() {
  return sa_ticks_;
}

uint16_t GetRpm() {
  return rpm_;
}

bool hasMotorSafelyStopped() {
  return (motor_stopped_cycles_ > NUM_STATUS_CYCLES_MOTOR_STOPPED);
}

bool UpdateMotorStoppedState(uint32_t last_tacho) {
  if (sa_tacho_ == last_tacho) {
    // Motor appears stopped (same tacho value) but at very low RPM tacho might not change every cycle
    if (hasMotorSafelyStopped()) {
      sa_ticks_ = 0;
      rpm_ = 0;
      motor::Brk::reset();  // Release brake
      return true;
    } else {
      motor_stopped_cycles_++;
    }
  } else {
    motor_stopped_cycles_ = 0;
  }
  return false;
}

}  // namespace motor_control
