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
uint8_t motor_stopped_cycles_ = 0;
}  // namespace

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
  // RPM calculation constant
  constexpr uint32_t rpm_divisor = (Board::SystemClock::Timer1 * SA_TIMER_PRESCALER) / SA_TIMER_PRESCALER;

  if (sa_ticks_ > 0) {
    return rpm_divisor / sa_ticks_;
  }
  return 0;
}

bool hasMotorSafelyStopped() {
  return (motor_stopped_cycles_ > NUM_STATUS_CYCLES_MOTOR_STOPPED);
}

bool UpdateMotorStoppedState(uint32_t last_tacho) {
  if (sa_tacho_ == last_tacho) {
    // Motor appears stopped (same tacho value) but at very low RPM tacho might not change every cycle
    if (hasMotorSafelyStopped()) {
      sa_ticks_ = 0;
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

// Called from ISR - must be accessible from interrupt context.
void UpdateSaTacho() {
  sa_tacho_++;
}

void UpdateSaTicks(uint32_t ticks) {
  sa_ticks_ = ticks;
}

}  // namespace motor_control
