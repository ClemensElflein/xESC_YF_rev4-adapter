// Created by Apehaenger on 2024-06-14.
//
// This file is part of the openmower project.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
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

using namespace modm::platform;

namespace Board {
/// @ingroup modm_board_xescyfr4
/// @{
using namespace modm::literals;

struct SystemClock {
    static constexpr uint32_t Frequency = 48_MHz;  // 48MHz generated from internal RC
    static constexpr uint32_t Ahb = Frequency;
    static constexpr uint32_t Apb = Frequency;

    static constexpr uint32_t Crc = Ahb;
    static constexpr uint32_t Flash = Ahb;
    static constexpr uint32_t Exti = Ahb;
    static constexpr uint32_t Rcc = Ahb;
    static constexpr uint32_t Timer1 = Apb;
    static constexpr uint32_t Timer3 = Apb;
    static constexpr uint32_t Timer14 = Apb;
    static constexpr uint32_t Timer16 = Apb;
    static constexpr uint32_t Timer17 = Apb;
    static constexpr uint32_t Usart1 = Ahb;
    static constexpr uint32_t Usart2 = Ahb;

    static bool inline enable() {
        Rcc::enableInternalClock();                       // 48MHz generated from internal RC
        Rcc::setHsiSysDivider(Rcc::HsiSysDivider::Div1);  // = 48MHz HSISYS
        Rcc::setFlashLatency<Frequency>();
        Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);  // = 48MHz HCLK
        Rcc::setApbPrescaler(Rcc::ApbPrescaler::Div1);  // = 48MHz PCLK/APB Timer Clocks
        Rcc::updateCoreFrequency<Frequency>();          // update frequencies for busy-wait delay functions

        return true;
    }
};

using LedGreen = GpioC15;
using LedRed = GpioC14;
using Leds = SoftwareGpioPort<LedGreen, LedRed>;
/// @}

namespace host_uart {
/// @ingroup modm_board_xescyfr4
/// @{
using Tx = GpioOutputA9;
using Rx = GpioInputA10;
using Uart = BufferedUart<UsartHal1, UartTxBuffer<32>, UartRxBuffer<32>>;
/// @}
}  // namespace host_uart

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
using In = GpioOutputA0;          // IN (High = VMC-on)
using DiagEnable = GpioOutputB6;  // DIAG_EN (High = Diagnostics on)
using Fault = GpioInputF2;        // !FAULT (Low = Fault detected)
/// @}
}  // namespace vm_switch

// VM-Switch
namespace motor {
/// @ingroup modm_board_xescyfr4
/// @{
using SA = GpioInputB7;    // Motor SA (Hall)
using Brk = GpioOutputA3;  // Motor BRK
using RS = GpioOutputA6;   // Motor !RS (Rapid/Rotor Start) (LowActive)
/// @}
}  // namespace motor

inline void
initialize() {
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();

    // Remap host_uart GPIOs
    GpioA9::remap();   // Remap A9 -> A11
    GpioA10::remap();  // Remap A10 -> A12

    // Init GPIOs
    Leds::setOutput(modm::Gpio::Low);
    vm_switch::In::setOutput(modm::Gpio::Low);            // VM-Switch, VMC = off
    vm_switch::DiagEnable::setOutput(modm::Gpio::Low);    // VM-Switch diagnostics disable because Fault input get shared with  NRST!!
    vm_switch::Fault::setInput(Gpio::InputType::PullUp);  // VM-Switch !FAULT signal (LowActive). Take attention to FAULT doesn't get triggered before NRST check
    motor::SA::setInput(Gpio::InputType::Floating);       // Motor SA (Hall)
    motor::Brk::setOutput(modm::Gpio::Low);               // Motor BRK
    motor::RS::setOutput(modm::Gpio::High);               // Motor !RS (Rapid/Rotor Start)
}

}  // namespace Board

#endif  // MODM_XESCYFR4_BOARD_HPP
