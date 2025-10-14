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
        static constexpr uint32_t Frequency = 48_MHz; // 48MHz generated from internal RC
        static constexpr uint32_t Ahb = Frequency;
        static constexpr uint32_t Apb = Frequency;

        static constexpr uint32_t Adc1 = Frequency;
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
            Rcc::enableInternalClock();                      // 48MHz generated from internal RC
            Rcc::setHsiSysDivider(Rcc::HsiSysDivider::Div1); // = 48MHz HSISYS
            Rcc::setFlashLatency<Frequency>();
            Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1); // = 48MHz HCLK
            Rcc::setApbPrescaler(Rcc::ApbPrescaler::Div1); // = 48MHz PCLK/APB Timer Clocks
            Rcc::updateCoreFrequency<Frequency>();         // update frequencies for busy-wait delay functions
            return true;
        }
    };

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
    } // namespace host

    namespace proto_uart {
        /// @ingroup modm_board_xescyfr4
        /// @{
        using Tx = GpioOutputA4;
        using Rx = GpioInputA5;
        using Uart = BufferedUart<UsartHal2, UartTxBuffer<32>>;
        /// @}
    } // namespace proto_uart

    // VM-Switch
    namespace vm_switch {
        /// @ingroup modm_board_xescyfr4
        /// @{
#ifdef HW_V1
        using In = GpioOutputA0;         // IN (High = VMC-on)
        using DiagEnable = GpioOutputB6; // DIAG_EN (High = Diagnostics on)
        using Fault = GpioInputF2;       // !FAULT (Low = Fault detected)
#else
        using In = GpioOutputA1;         // IN (High = VMC-on)
        using DiagEnable = GpioOutputA2; // DIAG_EN (High = Diagnostics on)
#endif
        /// @}
    } // namespace vm_switch

    // Motor
    namespace motor {
        /// @ingroup modm_board_xescyfr4
        /// @{
#ifdef HW_V1
        using SA = GpioInputB7;    // Motor SA (Hall)
        using SATimChan = SA::Ch4; // SA timer channel
        static constexpr auto SACaptureInterrupt = Timer1::Interrupt::CaptureCompare4; // SA timer capture/compare interrupt
        static constexpr auto SACaptureFlag = Timer1::InterruptFlag::CaptureCompare4;   // SA timer capture/compare flag
        using Brk = GpioOutputA3;  // Motor BRK
        using RS = GpioOutputA6;   // Motor !RS (Rapid/Rotor Start) (LowActive)
#else
        using SA = GpioInputA0;    // Motor SA (Hall)
        using SATimChan = SA::Ch1; // SA timer channel
        static constexpr auto SACaptureInterrupt = Timer1::Interrupt::CaptureCompare1; // SA timer capture/compare interrupt
        static constexpr auto SACaptureFlag = Timer1::InterruptFlag::CaptureCompare1;   // SA timer capture/compare flag
        using Brk = GpioOutputA6;  // Motor BRK
        using RS = GpioOutputA8;   // Motor !RS (Rapid/Rotor Start) (LowActive)
#endif
        /// @}
    } // namespace motor

    /// @ingroup modm_board_xescyfr4
    /// @{
#ifdef HW_V1
    static constexpr auto CurSenseChan = Adc1::Channel::In2;
    using CurSense = GpioInputA2;
#else
    static constexpr auto CurSenseChan = Adc1::Channel::In3;
    using CurSense = GpioInputA3;
#endif

#ifdef PROTO_DEBUG
    // Create an IODeviceWrapper around the Uart Peripheral we want to use
    modm::IODeviceWrapper<proto_uart::Uart, modm::IOBuffer::BlockIfFull> LoggerDevice;
#endif

    inline void
        initialize() {
        SystemClock::enable();
        SysTickTimer::initialize<SystemClock>();

        vm_switch::In::setOutput(Gpio::OutputType::PushPull);
        vm_switch::In::reset(); // VM-Switch, VMC = off
        vm_switch::DiagEnable::setOutput(Gpio::OutputType::PushPull);
        vm_switch::DiagEnable::reset(); // VM-Switch diagnostics disable because Fault input get shared with NRST!!
#ifdef HW_V1
        vm_switch::Fault::setInput(Gpio::InputType::PullUp); // VM-Switch !FAULT signal (LowActive). Take attention to FAULT doesn't get triggered before NRST check
#endif

        motor::SA::setInput(Gpio::InputType::Floating); // Motor SA (Hall)
        motor::Brk::setOutput(Gpio::OutputType::PushPull);
        motor::Brk::reset(); // Motor BRK
        motor::RS::setOutput(Gpio::OutputType::PushPull);
        motor::RS::set(); // Motor !RS (Rapid/Rotor Start)

        // Remap and init host uart
        GpioA9::remap();  // Remap A9 -> A11
        GpioA10::remap(); // Remap A10 -> A12
        host::Uart::connect<host::Tx::Tx, host::Rx::Rx>();
        host::Uart::initialize<SystemClock, 115200_Bd>();
        host::Shutdown::setInput(Gpio::InputType::Floating);

#ifdef PROTO_DEBUG
        proto_uart::Uart::connect<proto_uart::Tx::Tx, proto_uart::Rx::Rx>();
        proto_uart::Uart::initialize<SystemClock, PROTO_DEBUG_BAUD>();
        MODM_LOG_INFO << modm::endl
            << "Board initialized" << modm::endl
            << modm::flush;
#endif
    }
    /// @}

} // namespace Board

#ifdef PROTO_DEBUG
// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(Board::LoggerDevice);
modm::log::Logger modm::log::info(Board::LoggerDevice);
modm::log::Logger modm::log::warning(Board::LoggerDevice);
modm::log::Logger modm::log::error(Board::LoggerDevice);
#endif

#endif // MODM_XESCYFR4_BOARD_HPP
