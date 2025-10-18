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
bool wrong_hw = false;  // True if FW doesn't match detected HW

// ADC data buffer (required by AdcSampler).
std::array<uint16_t, AdcSampler::sequence.size()> AdcSampler::_data = {};

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
  uint32_t faults = 0;
  bool is_shutdown = host::Shutdown::read();

  // FAULT_WRONG_HW_VERSION - Check if wrong hardware version firmware is
  // running.
  if (wrong_hw) {
    faults |= FAULT_WRONG_HW_VERSION;
  }

  // FAULT_UNINITIALIZED.
  if (!host_comm::AreSettingsValid()) {
    faults |= FAULT_UNINITIALIZED;
  }

  // FAULT_WATCHDOG.
  if (MILLIS - host_comm::GetLastWatchdogMillis() > WATCHDOG_TIMEOUT_MILLIS) {
    faults |= FAULT_WATCHDOG;
  }

  // FAULT_OVERCURRENT.
  if (status.current_input > static_cast<double>(HW_LIMIT_CURRENT)) {
    faults |= FAULT_OVERCURRENT;
  }

  // FAULT_OVERTEMP_PCB.
  const auto& settings = host_comm::GetSettings();
  if (status.temperature_pcb >
      static_cast<double>(std::min(HW_LIMIT_PCB_TEMP, settings.max_pcb_temp))) {
    faults |= FAULT_OVERTEMP_PCB;
  }

#ifdef HW_V1
  // VMS FAULTs (V1 hardware only).
  if (!vm_switch::Fault::read()) {
    if (vm_switch::In::isSet()) {
      // VM-Switch is "on"
      // Disable VM-Switch diagnostics when switched on.
      // TODO: Review if this is correct behavior.
    } else if (!is_shutdown) {
      faults |= FAULT_OPEN_LOAD;
    }
  }
#endif

  // Update fault code and LED indicators.
  if (faults) {
    status.fault_code = faults;

    // Update LED indicators based on fault priority.
    if (faults & FAULT_UNINITIALIZED) {
      status_led.Blink();  // 1Hz blink
    } else {
      status_led.Off();
    }

    if (faults & FAULT_WRONG_HW_VERSION) {
      error_led.On();  // Steady on (clear indication)
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
      status_led.Blink(100U, 1800U, 1);  // Short pulse
    } else {
      status_led.On();
    }

    // Reset fault indication if cleared.
    if (status.fault_code != 0) {
      error_led.Off();
      status.fault_code = 0;
    }
  }
}

/**
 * @brief Update status and handle faults - called every STATUS_CYCLE.
 */
void UpdateStatus() {
  status.seq++;

  // Update fault detection and LED status.
  UpdateFaults();

  bool has_faults = (status.fault_code != 0);
  bool is_shutdown = host::Shutdown::read();

  // Enable/disable VMC based on faults and shutdown state
  if (!(status.fault_code & FAULT_OPEN_LOAD) && !is_shutdown) {
    vm_switch::In::set();
    // FIXME once migrated:
    // Timer1::enableInterrupt(Timer1::Interrupt::CaptureCompare4);
  } else if (motor_control::UpdateMotorStoppedState(status.tacho)) {
    // FIXME once migrated:
    // Timer1::disableInterrupt(Timer1::Interrupt::CaptureCompare4);
    vm_switch::In::reset();
  }

  // Update motor state based on setpoint and faults.
  motor_control::SetDutySetpoint(host_comm::GetDutySetpoint());
  motor_control::UpdateMotorState(has_faults, is_shutdown);

  // Handle motor stopped detection.
  if (motor_control::GetDuty() == 0.0f) {
    motor_control::UpdateMotorStoppedState(status.tacho);
  } else {
    motor_control::ResetMotorStoppedCycles();
  }

  // Update status packet.
  status.duty_cycle = motor_control::GetDuty();
  status.tacho = motor_control::GetSaTacho();
  status.tacho_absolute = motor_control::GetSaTacho();
  status.rpm = motor_control::GetRpm();
  status.temperature_pcb = AdcSampler::getInternalTemp();
  status.current_input = AdcSampler::getVoltage(AdcSampler::Sensors::CurSense) /
                         (CUR_SENSE_GAIN * R_SHUNT);

#if (defined PROTO_DEBUG_COMMS || defined PROTO_DEBUG_MOTOR || \
     defined PROTO_DEBUG_ADC)
  MODM_LOG_INFO << "TX status.fault_code=" << status.fault_code;
#ifdef PROTO_DEBUG_MOTOR
  MODM_LOG_INFO << ", tacho=" << status.tacho
                << ", ticks=" << motor_control::GetSaTicks()
                << ", rpm=" << status.rpm;
#endif
#ifdef PROTO_DEBUG_ADC
  MODM_LOG_DEBUG << " VRef=" << AdcSampler::getInternalVref_u() << "mV, "
                 << AdcSampler::getInternalVref_f() << "mV"
                 << " Temp=" << AdcSampler::getInternalTemp()
                 << " CurrentSense="
                 << AdcSampler::getVoltage(AdcSampler::Sensors::CurSense)
                 << "V, "
                 << AdcSampler::getVoltage(AdcSampler::Sensors::CurSense) /
                        (CUR_SENSE_GAIN * R_SHUNT)
                 << "A";
#endif
  MODM_LOG_INFO << modm::endl << modm::flush;
#endif

  host_comm::SendMessage(&status, sizeof(status));
}

/**
 * @brief Status Timer ISR - called every STATUS_CYCLE ms.
 */
MODM_ISR(TIM14) {
  Timer14::acknowledgeInterruptFlags(Timer14::InterruptFlag::Update);
  UpdateStatus();
}

/**
 * @brief SA (Hall sensor) Capture/Compare ISR.
 */
MODM_ISR(TIM1_CC) {
  static uint16_t last_cc_value = 0;
  uint16_t temp_cc_value = Timer1::getCompareValue<motor::SATimChan>();
  uint32_t temp_ticks;

  Timer1::acknowledgeInterruptFlags(motor::SACaptureFlag);
  motor_control::UpdateSaTacho();

  if (Timer1::getInterruptFlags() & Timer1::InterruptFlag::Update) {
    Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
    temp_ticks = temp_cc_value + (0xffff - last_cc_value);
  } else {
    temp_ticks = temp_cc_value - last_cc_value;
  }

  last_cc_value = temp_cc_value;
  motor_control::UpdateSaTicks(temp_ticks);
}

int main() {
  // Initialize common hardware.
  Board::initialize();

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
  if (!hardware::checkFlashMatchesBinary()) {
    wrong_hw = true;
#ifdef HW_V1
    // V1 binary on V2 hardware -> Initialize V2 LED GPIO.
    led_red_v2_gpio.SetOutput(Gpio::OutputType::PushPull);
    error_led.SetGpio(&led_red_v2_gpio);
    MODM_LOG_ERROR << "CRITICAL: V1 firmware on V2 hardware detected!"
                   << modm::endl;
#else
    // V2 binary on V1 hardware -> Initialize V1 LED GPIO.
    led_red_v1_gpio.SetOutput(Gpio::OutputType::PushPull);
    error_led.SetGpio(&led_red_v1_gpio);
    MODM_LOG_ERROR << "CRITICAL: V2 firmware on V1 hardware detected!"
                   << modm::endl;
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
    disable_nrst();  // Check NRST pin. Will flash if wrong and reset.
    vm_switch::DiagEnable::set();  // V1 - safe only after disable_nrst()
    vm_switch::Fault::setInput(
        Gpio::InputType::PullUp);  // VM-Switch !FAULT signal (LowActive). Take
                                   // attention to FAULT doesn't get triggered
                                   // before NRST check
#else                              // HW_V2
    vm_switch::DiagEnable::set();  // V2+ - always safe to enable diagnostics
#endif
  }

  // Initialize subsystems
  AdcSampler::init();
  motor_control::Init();
  host_comm::Init();

  // Prepare status message.
  status.message_type = XESCYFR4_MSG_TYPE_STATUS;
  status.fw_version_major = 0;
  status.fw_version_minor = 3;
  status.direction = 0;  // Motor has only one direction

#ifdef FALSE
  // SA (Hall) input - Capture/Compare timer.
  Timer1::connect<motor::SATimChan>();
  Timer1::enable();
  Timer1::setMode(Timer1::Mode::UpCounter);
  Timer1::setPrescaler(SA_TIMER_PRESCALER);
  Timer1::setOverflow(0xFFFF);
  Timer1::configureInputChannel<motor::SATimChan>(
      Timer1::InputCaptureMapping::InputOwn,
      Timer1::InputCapturePrescaler::Div1,  // Must match
                                            // SA_TIMER_INPUT_PRESCALER
      Timer1::InputCapturePolarity::Rising, SA_TIMER_MIN_TICKS);
  Timer1::enableInterrupt(motor::SACaptureInterrupt);
  Timer1::enableInterruptVector(motor::SACaptureInterrupt, true, 21);
  Timer1::applyAndReset();
  Timer1::start();
#endif

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
    error_led.Blink(
        {.on_time_ms = 1000, .off_time_ms = 1000});  // Slow blink (0.5Hz)
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