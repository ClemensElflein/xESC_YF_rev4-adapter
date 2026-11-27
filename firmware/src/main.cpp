// Created by Apehaenger on 2024-06-14.
// Refactored by Jörg Ebeling on 2025-10-18.
//
// This file is part of the openmower project.
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

#include <modm/platform.hpp>
#include <numeric>

#include "AdcSampler.hpp"
#include "board.hpp"
#include "config.h"
#include "debug.h"
#include "disable_nrst.hpp"
#include "hardware/hw_version.hpp"
#include "host_comm.hpp"
#include "led_controller.hpp"
#include "motor_control.hpp"
#include "xesc_yfr4_datatypes.h"

using namespace Board;
using namespace std::chrono_literals;

#define MILLIS modm::Clock::now().time_since_epoch().count()

// Global state.
bool wrong_hw = false;           // True if FW doesn't match detected HW
bool open_load_at_boot = false;  // True if open load was detected during boot (VM switch off)

// LED GPIO objects.
ModmGpio<LedGreen> led_green_gpio;
ModmGpio<LedRed_v1> led_red_v1_gpio;
ModmGpio<LedRed_v2> led_red_v2_gpio;

// LED Controllers.
LedController status_led(&led_green_gpio);
#ifdef HW_V1
LedController error_led(&led_red_v1_gpio);
#else
LedController error_led(&led_red_v2_gpio);
#endif

// Status and settings packets.
XescYFR4StatusPacket status = {};

/**
 * @brief Update fault detection and LED indicators.
 */
void UpdateFaults() {
  static uint32_t last_fault_millis = 0;
  uint32_t faults = 0;
  bool is_shutdown = host::Shutdown::read();
  const auto& settings = host_comm::GetSettings();
  float cs_current = AdcSampler::getValue(AdcSampler::Sensors::CurrentSense);
  static uint8_t overcurrent_limit_count = 0;

  // FAULT_WRONG_HW_VERSION - Check if wrong hardware version firmware is running
  if (wrong_hw) {
    faults |= FAULT_WRONG_HW_VERSION;
  }

  // FAULT_UNINITIALIZED
  if (!host_comm::AreSettingsValid()) {
    faults |= FAULT_UNINITIALIZED;
  }

  // FAULT_OPEN_LOAD, detected at boot whilst the VM switch was still off.
  // One-shot: from the next cycle on, the regular ON/OFF-state detection below
  // takes over (and clears the fault again once a motor is connected).
  if (open_load_at_boot) {
    faults |= FAULT_OPEN_LOAD;
    open_load_at_boot = false;
  }

  // FAULT_WATCHDOG
  if (host_comm::HasWatchdogTimedOut()) {
    faults |= FAULT_WATCHDOG;
  }

  // FAULT_OVERTEMP_PCB
  if (AdcSampler::getValue(AdcSampler::Sensors::Temp) > (std::min(HW_LIMIT_PCB_TEMP, settings.max_pcb_temp))) {
    faults |= FAULT_OVERTEMP_PCB;
  }

#ifdef HW_V1
  // HW_V1: TPS1H200A via /FAULT pin
  if (!vm_switch::Fault::read()) {
    if (vm_switch::In::isSet()) {  // VM-Switch is "on"
      if (cs_current < 0.001f) {
        faults |= FAULT_OPEN_LOAD;
      } else {
        // Disabled because not clear how to detect
        // faults |= FAULT_OVERTEMP_PCB;
      }
    } else if (!is_shutdown) {
      faults |= FAULT_OPEN_LOAD;
    }
  }
#else
  // HW_V2: TPS1H100B fault detection via CS pin voltage
  // The TPS1H100B signals faults on the CS pin that we're already reading via ADC
  // According to datasheet:
  // - Normal operation: V_CS in linear region (0-4V), proportional to current
  // - Fault conditions: V_CS pulled to V_CS,h (>= 4.3V, above ADC range)
  //   * Short to GND / Current limit
  //   * Thermal shutdown / Thermal swing
  // - Open load (ON-state): V_CS ≈ 0 (very low current)

  // Check for V_CS,h condition (fault signaling)
  // V_CS,h is >= 4.3V, but our ADC reads max 3.3V (VDDA)
  // If CS voltage is at or near VDDA, the actual voltage is likely higher (V_CS,h)
  static const float ICS_H_THRESHOLD = Board::adc::CalculateCurrent(3.2f);  // 97% of typical 3.3V VDDA

  // Open load is evaluated on the raw CS voltage and relative to its calibrated
  // zero offset (see config.h). Doing so keeps the detection independent of
  // R_CS (R17) and the ADC offset, which is why the ON-state threshold no longer
  // is a fixed current value.
  const float cs_offset_voltage = AdcSampler::getCsVoltage() - AdcSampler::getCsZeroVoltage();

  if (vm_switch::In::isSet() && !is_shutdown) {
    // Motor should be running - check for faults
    if (cs_current >= ICS_H_THRESHOLD) {
      // V_CS,h detected - could be:
      // - Short to GND / Current limit
      // - Thermal shutdown
      // - Thermal swing
      // The TPS1H100B auto-recovers when fault clears
      faults |= FAULT_OVERCURRENT;  // Use existing fault code for now
    } else {
      // Open load: No (or nearly no) load current, so the CS voltage stays at
      // its zero offset. Compared as voltage, as the offset is a fixed voltage
      // whereas cs_current scales with R_CS.
      if (cs_offset_voltage < OPEN_LOAD_ON_DELTA_V) {
        faults |= FAULT_OPEN_LOAD;
      }
    }
  } else if (!vm_switch::In::isSet() && !is_shutdown) {
    // Motor off - check for open load in off-state
    // In off-state with open load: CS pin pulled to V_CS,h, which R16/D5 clamp
    // to a level clearly above the linear current-sense range
    if (AdcSampler::getCsVoltage() >= OPEN_LOAD_OFF_LEVEL_V) {
      faults |= FAULT_OPEN_LOAD;
    } else if (AdcSampler::getCsVoltage() < OPEN_LOAD_ZERO_MAX_V) {
      // VM switch off and no open load -> no load current can flow, so the CS
      // level is a valid zero reference. Keeps the ON-state detection calibrated
      // even if the motor gets connected after a boot open load.
      AdcSampler::setCsZeroVoltage(AdcSampler::getCsVoltage());
    }
  }
#endif

  // Handle FAULT_OVERCURRENT only if VMC is enabled and FAULT_OPEN_LOAD not already happen
  const float overcurrent_limit = std::min(hardware::getHardwareCurrentLimit(), settings.motor_current_limit);
  if (vm_switch::In::isSet() && !(faults & FAULT_OPEN_LOAD) && cs_current > overcurrent_limit) {
    if (overcurrent_limit_count++ >= 10) {  // Require 10 consecutive overcurrent readings (200ms at 20ms status update)
      faults |= FAULT_OVERCURRENT;
    }
  } else {
    overcurrent_limit_count = 0;
  }

  // Enable/disable VMC based on faults, shutdown state or safely stopped motor
  // Do this after open-load or over-current evaluation, to give adc a chance to read the new values
  if (!(faults & FAULT_OPEN_LOAD) && !host::Shutdown::read()) {
    vm_switch::In::set();
    Timer1::enableInterrupt(motor::SACaptureInterrupt);
  } else if (motor_control::hasMotorSafelyStopped()) {
    Timer1::disableInterrupt(motor::SACaptureInterrupt);
    vm_switch::In::reset();
  }

  // Update LED indicators and status.fault_code
  if (faults) {
    status.fault_code = faults;
    last_fault_millis = MILLIS;

    // Update LED indicators based on fault priority.
    if (faults & FAULT_UNINITIALIZED) {
      status_led.Blink();  // 1Hz blink
    } else {
      status_led.Off();
    }

    if (faults & FAULT_WRONG_HW_VERSION) {
      error_led.On();
    } else if (faults & FAULT_OPEN_LOAD) {
      error_led.Blink(125U, 125U, 0);  // Quick blink (4Hz)
    } else if (faults & (FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT)) {
      error_led.Blink(250U, 250U, 0);  // Fast blink (2Hz)
    } else if (faults & FAULT_WATCHDOG) {
      error_led.Blink(0, false);  // 1Hz blink
    }
  } else {
    // No faults - update status LED.
    if (is_shutdown) {
      status_led.Blink(
          {.on_time_ms = 100U, .off_time_ms = 1800U, .blink_cycle_limit = 1, .complete = true});  // Short pulse
    } else {
      status_led.On();
    }

    // Reset fault indication if cleared.
    if (status.fault_code != 0) {
      // Reset faults only if MIN_FAULT_TIME_MILLIS passed, or if it was a dedicated watchdog fault
      if (MILLIS - last_fault_millis > MIN_FAULT_TIME_MILLIS || status.fault_code == FAULT_WATCHDOG) {
        error_led.Off();
        status.fault_code = 0;
      }
    }
  }
}

/**
 * @brief Update status and handle faults - called every STATUS_CYCLE.
 */
void UpdateStatus() {
  static bool first_update = true;

  // Update fault detection and LED status.
  UpdateFaults();

  // Update motor state based on setpoint, faults and last tacho (20ms ago)
  motor_control::UpdateMotorState((status.fault_code != 0), host::Shutdown::read(), status.tacho);

  // Update status packet
  status.seq++;
  status.duty_cycle = motor_control::GetDuty();
  status.tacho = motor_control::GetSaTacho();
  status.tacho_absolute = motor_control::GetSaTacho();
  status.rpm = motor_control::GetRpm();
  status.temperature_pcb = AdcSampler::getValue(AdcSampler::Sensors::Temp);
  status.current_input = AdcSampler::getValue(AdcSampler::Sensors::CurrentSense);

#if (defined PROTO_DEBUG_COMMS || defined PROTO_DEBUG_MOTOR || defined PROTO_DEBUG_ADC)
  MODM_LOG_INFO << "TX status.fault_code=0x" << modm::hex << status.fault_code << modm::ascii;
#ifdef PROTO_DEBUG_MOTOR
  MODM_LOG_INFO << ", tacho=" << status.tacho << ", ticks=" << motor_control::GetSaTicks() << ", rpm=" << status.rpm;
#endif
#ifdef PROTO_DEBUG_ADC
  MODM_LOG_DEBUG << " Dirty=" << AdcSampler::getCacheDirty()
                 << " VRef=" << AdcSampler::getValue(AdcSampler::Sensors::VRef) << "V"
                 << " Temp=" << status.temperature_pcb << " CurrentSense=" << status.current_input << "A";
#endif
  MODM_LOG_INFO << modm::endl << modm::flush;
#endif

  // Disable ADC free running mode and manually start conversion to get fresh data for next status update
  if (first_update) {
    AdcSampler::switchToSingleShot();
    first_update = false;
  }
  // Before sending the status, trigger a new conversion for next cycle
  AdcSampler::triggerConversion();

  // Send status packet
  host_comm::SendMessage(&status, sizeof(status));
}

/**
 *
 * @brief Status Timer ISR - called every STATUS_CYCLE ms.
 */
MODM_ISR(TIM14) {
  Timer14::acknowledgeInterruptFlags(Timer14::InterruptFlag::Update);
  UpdateStatus();
}

int main() {
  // Initialize common hardware.
  Board::initialize();

  // Select the hardware profile (R17/current limit) from the OTP hardware
  // version once, before the ADC interrupt starts using it.
  hardware::initActiveProfile();

#ifdef PROTO_DEBUG
  // Print hardware version info.
  MODM_LOG_INFO << "xESC_YF_rev4 - ";
  hardware::printVersionInfo(modm::log::info);
  MODM_LOG_INFO << modm::endl;
#endif

  // Check for hardware mismatch and initialize cross-version LEDs if needed.
  // SAFETY: We only initialize the GPIO for the OTHER hardware version
  // to avoid touching unknown GPIOs that might control motors or dangerous
  // hardware!
  if (!hardware::checkHardwareVersionMatchesBinary()) {
    wrong_hw = true;
#ifdef HW_V1
    // V1 binary on V2 hardware -> Initialize V2 LED GPIO.
    led_red_v2_gpio.SetOutput(Gpio::OutputType::PushPull);
    error_led.SetGpio(&led_red_v2_gpio);
#ifdef PROTO_DEBUG
    MODM_LOG_ERROR << "CRITICAL: V1 firmware on V2 hardware detected!" << modm::endl;
#endif
#else
    // V2 binary on V1 hardware -> Initialize V1 LED GPIO.
    led_red_v1_gpio.SetOutput(Gpio::OutputType::PushPull);
    error_led.SetGpio(&led_red_v1_gpio);
#ifdef PROTO_DEBUG
    MODM_LOG_ERROR << "CRITICAL: V2 firmware on V1 hardware detected!" << modm::endl;
#endif
#endif
  } else {
#ifdef HW_V1
    led_red_v1_gpio.SetOutput(Gpio::OutputType::PushPull);
#else
    led_red_v2_gpio.SetOutput(Gpio::OutputType::PushPull);
#endif
  }
  led_green_gpio.SetOutput(Gpio::OutputType::PushPull);

  // Hardware-specific initialization.
  if (!wrong_hw) {
    vm_switch::In::setOutput(Gpio::OutputType::PushPull);
    vm_switch::In::reset();  // VM-Switch, VMC = off
    vm_switch::DiagEnable::setOutput(Gpio::OutputType::PushPull);
    vm_switch::DiagEnable::reset();  // VM-Switch diagnostics disable because
                                     // Fault input get shared with NRST!!

    motor::SA::setInput(Gpio::InputType::Floating);  // Motor SA (Hall)
    motor::Brk::setOutput(Gpio::OutputType::PushPull);
    motor::Brk::reset();  // Motor BRK
    motor::RS::setOutput(Gpio::OutputType::PushPull);
    motor::RS::set();  // Motor !RS (Rapid/Rotor Start)

#ifdef HW_V1
    disable_nrst();                                       // Check NRST pin. Will flash if wrong and reset.
    vm_switch::DiagEnable::set();                         // V1 - safe only after disable_nrst()
    vm_switch::Fault::setInput(Gpio::InputType::PullUp);  // VM-Switch !FAULT signal (LowActive). Take
                                                          // attention to FAULT doesn't get triggered
                                                          // before NRST check
#else                                                     // HW_V2
    vm_switch::DiagEnable::set();  // V2+ - always safe to enable diagnostics
#endif
  }

  // Initialize subsystems
  AdcSampler::init();
  motor_control::Init();
  host_comm::Init();

  // Calibrate the current-sense zero offset and check for an open load (motor
  // not connected) whilst the VM switch is still off, i.e. no load current can
  // flow. This has to happen before the status timer starts, as VMC gets
  // enabled again in the very first status cycle (see UpdateFaults()).
  // HW_V1 keeps its legacy TPS1H200A /FAULT pin detection untouched.
#ifndef HW_V1
  if (!wrong_hw) {
    static constexpr uint8_t CS_CALIBRATION_SAMPLES = 4;
    static constexpr uint32_t CS_CALIBRATION_TIMEOUT_MILLIS = 100;
    AdcSampler::switchToSingleShot();

    float cs_sum = 0.0f;
    float cs_max = 0.0f;
    uint8_t cs_samples = 0;
    for (uint8_t i = 0; i < CS_CALIBRATION_SAMPLES; ++i) {
      AdcSampler::triggerConversion();
      const uint32_t start = MILLIS;
      while (AdcSampler::getCacheDirty() && (MILLIS - start < CS_CALIBRATION_TIMEOUT_MILLIS)) {
      }
      const float cs_voltage = AdcSampler::getCsVoltage();
      cs_sum += cs_voltage;
      cs_samples++;
      if (cs_voltage > cs_max) {
        cs_max = cs_voltage;
      }
    }

    if (cs_max >= OPEN_LOAD_OFF_LEVEL_V) {
      // Open load in OFF state, e.g. motor cable not connected at boot.
      // Fault gets kept alive by the OFF-state check in UpdateFaults().
      open_load_at_boot = true;
    } else if ((cs_max < OPEN_LOAD_ZERO_MAX_V) && (cs_samples > 0)) {
      AdcSampler::setCsZeroVoltage(cs_sum / cs_samples);
    }
    // else: Implausible CS level -> keep the zero offset at 0 and leave it to
    //       the regular checks instead of calibrating against a bogus reference

#ifdef PROTO_DEBUG
    MODM_LOG_INFO << "CS zero offset=" << AdcSampler::getCsZeroVoltage()
                  << "V, boot open load=" << (open_load_at_boot ? "yes" : "no") << modm::endl
                  << modm::flush;
#endif
  }
#endif  // !HW_V1

  // Prepare status message.
  status.message_type = XESCYFR4_MSG_TYPE_STATUS;
  status.fw_version_major = 0;
  status.fw_version_minor = 4;
  status.direction = 0;  // Motor has only one direction

  // Status Timer.
  Timer14::enable();
  Timer14::setMode(Timer14::Mode::UpCounter);
  Timer14::setPeriod<Board::SystemClock>(STATUS_CYCLE);
  Timer14::enableInterrupt(Timer14::Interrupt::Update);
  Timer14::enableInterruptVector(true, 26);
  Timer14::applyAndReset();
  Timer14::start();

  // Boot-up success indication (3 quick blinks).
  status_led.QuickBlink(3, true);
  if (wrong_hw) {
    error_led.On();
  } else {
    error_led.QuickBlink(3, true);
  }

  // Main loop.
  while (true) {
    host_comm::ProcessUartData();
    status_led.Update();
    error_led.Update();
  }
}
