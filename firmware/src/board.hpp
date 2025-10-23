// Created by Apehaenger on 2024-06-14.
//
// This file is part of the openmower project.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based
// on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef MODM_XESCYFR4_BOARD_HPP
#define MODM_XESCYFR4_BOARD_HPP

#include <modm/platform.hpp>

#include "config.h"

#ifdef PROTO_DEBUG
#include <modm/debug/logger.hpp>
#endif

using namespace modm::platform;

namespace Board {
/// @ingroup modm_board_xescyfr4
/// @{
using namespace modm::literals;

struct SystemClock {
  static constexpr uint32_t Frequency = Rcc::HsiFrequency;  // 48MHz generated from internal RC
  static constexpr uint32_t Ahb = Frequency;
  static constexpr uint32_t Apb = 24_MHz;

  static constexpr uint32_t Adc1 = Apb;

  static constexpr uint32_t Spi1 = Apb;

  static constexpr uint32_t Usart1 = Apb;
  static constexpr uint32_t Usart2 = Apb;

  static constexpr uint32_t I2c1 = Apb;

  static constexpr uint32_t Timer1 = Apb;
  static constexpr uint32_t Timer2 = Apb;
  static constexpr uint32_t Timer3 = Apb;
  static constexpr uint32_t Timer14 = Apb;
  static constexpr uint32_t Timer16 = Apb;
  static constexpr uint32_t Timer17 = Apb;
  static constexpr uint32_t Iwdg = Rcc::LsiFrequency;
  static constexpr uint32_t Rtc = Rcc::LsiFrequency;

  static bool inline enable() {
    Rcc::enableLowSpeedInternalClock();
    Rcc::enableRealTimeClock(Rcc::RealTimeClockSource::Lsi);

    // 48MHz generated from internal RC
    Rcc::enableInternalClock();
    Rcc::setHsiSysDivider(Rcc::HsiSysDivider::Div1);
    // set flash latency for 48MHz
    Rcc::setFlashLatency<Frequency>();
    // switch system clock to PLL output
    Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
    Rcc::setApbPrescaler(Rcc::ApbPrescaler::Div2);  // 24 MHz for APB
    // update frequencies for busy-wait delay functions
    Rcc::updateCoreFrequency<Frequency>();

    return true;
  }
};

using LedGreen = GpioC15;

// Hardware version specific LEDs (always define both for cross-version error indication)
using LedRed_v1 = GpioC14;
using LedRed_v2 = GpioB6;

// Alias for the correct red LED based on compiled hardware version
#ifdef HW_V1
using LedRed = LedRed_v1;
#else
using LedRed = LedRed_v2;
#endif
/// @}

namespace host {
/// @ingroup modm_board_xescyfr4
/// @{
using Tx = GpioOutputA9;
using Rx = GpioInputA10;
using Uart = BufferedUart<UsartHal1, UartTxBuffer<32>, UartRxBuffer<32>>;
#ifdef HW_V1
using Shutdown = GpioInputA8;
#else
using Shutdown = GpioInputC14;
#endif
/// @}
}  // namespace host

namespace proto_uart {
/// @ingroup modm_board_xescyfr4
/// @{
using Tx = GpioOutputA4;
using Rx = GpioInputA5;
using Uart = BufferedUart<UsartHal2, UartTxBuffer<32>>;
/// @}
}  // namespace proto_uart

// VM-Switch
namespace vm_switch {
/// @ingroup modm_board_xescyfr4
/// @{
#ifdef HW_V1
using In = GpioOutputA0;          // IN (High = VMC-on)
using DiagEnable = GpioOutputB6;  // DIAG_EN (High = Diagnostics on)
using Fault = GpioInputF2;        // !FAULT (Low = Fault detected)
#else
using In = GpioOutputA1;          // IN (High = VMC-on)
using DiagEnable = GpioOutputA2;  // DIAG_EN (High = Diagnostics on)
#endif
/// @}
}  // namespace vm_switch

// Motor
namespace motor {
/// @ingroup modm_board_xescyfr4
/// @{
#ifdef HW_V1
using SA = GpioInputB7;                                                         // Motor SA (Hall)
using SATimChan = SA::Ch4;                                                      // SA timer channel
static constexpr auto SACaptureInterrupt = Timer1::Interrupt::CaptureCompare4;  // SA timer capture/compare interrupt
static constexpr auto SACaptureFlag = Timer1::InterruptFlag::CaptureCompare4;   // SA timer capture/compare flag
using Brk = GpioOutputA3;                                                       // Motor BRK
using RS = GpioOutputA6;  // Motor !RS (Rapid/Rotor Start) (LowActive)
#else
using SA = GpioInputA0;                                                         // Motor SA (Hall)
using SATimChan = SA::Ch1;                                                      // SA timer channel
static constexpr auto SACaptureInterrupt = Timer1::Interrupt::CaptureCompare1;  // SA timer capture/compare interrupt
static constexpr auto SACaptureFlag = Timer1::InterruptFlag::CaptureCompare1;   // SA timer capture/compare flag
using Brk = GpioOutputA6;                                                       // Motor BRK
using RS = GpioOutputA8;  // Motor !RS (Rapid/Rotor Start) (LowActive)
#endif
/// @}
}  // namespace motor

namespace adc {
/// @ingroup modm_board_xescyfr4
/// @{

#ifdef HW_V1
// HW_V1: INA139NA current sense amplifier constants
// Current calculation: I = V_adc / (CurSenseGain * RShunt)
static constexpr float CurSenseGain = 40.0f;                     // Gain resistor: 40kΩ
static constexpr float RShunt = 0.075f;                          // Shunt resistor: 0.075Ω
static constexpr float CurSenseDivisor = CurSenseGain * RShunt;  // Pre-computed: 3.0

static constexpr auto CurSenseChan = Adc1::Channel::In2;
using CurSense = GpioInputA2;
using CurSenseAdc = CurSense::In2;  // ADC signal type for connect<>
#else
// HW_V2: TPS1H100B integrated current sense with current mirror
// The TPS1H100B sources I_out/K through RCS to GND
// V_CS = (I_out / K) × R_CS  =>  I_out = (V_CS × K) / R_CS
// K = 500 (current sense ratio for Version B)
// R_CS = 1kΩ (external sense resistor to GND)
// Current calculation: I_out = V_CS × K / R_CS = V_CS × 500 / 1000 = V_CS × 0.5
static constexpr float CurSenseK = 500.0f;                         // Current sense ratio (Version B)
static constexpr float CurSenseRCS = 1000.0f;                      // Sense resistor to GND: 1kΩ
static constexpr float CurSenseKDivRCS = CurSenseK / CurSenseRCS;  // Pre-computed: 0.5

static constexpr auto CurSenseChan = Adc1::Channel::In3;
using CurSense = GpioInputA3;
using CurSenseAdc = CurSense::In3;  // ADC signal type for connect<>
#endif

/**
 * @brief Calculate current from ADC voltage reading (hardware-specific)
 * @param voltage_adc ADC voltage in volts
 * @return Current in amperes
 */
inline float CalculateCurrent(float voltage_adc) {
#ifdef HW_V1
  // HW_V1: INA139NA current sense amplifier
  // I = V_adc / (Gain × R_shunt) = V_adc / 3.0
  return voltage_adc / CurSenseDivisor;
#else
  // HW_V2: TPS1H100B with current mirror
  // I_out = V_CS × (K / R_CS) = V_CS × 0.5
  return voltage_adc * CurSenseKDivRCS;
#endif
}

/// @}
}  // namespace adc

/// @ingroup modm_board_xescyfr4
/// @{
#ifdef PROTO_DEBUG
// Forward declaration - definition in board.cpp
extern modm::IODeviceWrapper<proto_uart::Uart, modm::IOBuffer::BlockIfFull> LoggerDevice;
#endif

inline void initialize() {
  SystemClock::enable();
  SysTickTimer::initialize<SystemClock>();

  // Remap and init host uart
  GpioA9::remap();   // Remap A9 -> A11
  GpioA10::remap();  // Remap A10 -> A12
  host::Uart::connect<host::Tx::Tx, host::Rx::Rx>();
  host::Uart::initialize<SystemClock, 115200_Bd>();
  host::Shutdown::setInput(Gpio::InputType::Floating);

#ifdef PROTO_DEBUG
  proto_uart::Uart::connect<proto_uart::Tx::Tx, proto_uart::Rx::Rx>();
  proto_uart::Uart::initialize<SystemClock, PROTO_DEBUG_BAUD>();
  MODM_LOG_INFO << modm::endl << "Board initialized" << modm::endl << modm::flush;
#endif
}
/// @}

}  // namespace Board

#ifdef PROTO_DEBUG
// Forward declarations for logger streams (definitions in board.cpp)
namespace modm::log {
extern Logger debug;
extern Logger info;
extern Logger warning;
extern Logger error;
}  // namespace modm::log
#endif

#endif  // MODM_XESCYFR4_BOARD_HPP
