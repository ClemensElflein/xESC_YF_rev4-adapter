/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MODM_STM32_CUSTOM_STM32C0X_HPP
#define MODM_STM32_CUSTOM_STM32C0X_HPP

#include <modm/architecture/interface/clock.hpp>
#include <modm/platform.hpp>
// #include <modm/debug/logger.hpp>
/// @ingroup modm_board_custom_stm32c0x modm_board_custom_stm32c0x
// #define MODM_BOARD_HAS_LOGGER

using namespace modm::platform;

namespace Board {
/// @ingroup modm_board_custom_stm32c0x modm_board_custom_stm32c0x
/// @{
using namespace modm::literals;

/// STM32C011 running at 48MHz generated from internal RC
struct SystemClock {
    static constexpr uint32_t Frequency = 48_MHz;
    static constexpr uint32_t Ahb = Frequency;
    static constexpr uint32_t Apb = Frequency;

    static constexpr uint32_t Aes = Ahb;
    static constexpr uint32_t Rng = Ahb;
    static constexpr uint32_t Crc = Ahb;
    static constexpr uint32_t Flash = Ahb;
    static constexpr uint32_t Exti = Ahb;
    static constexpr uint32_t Rcc = Ahb;
    static constexpr uint32_t Dmamux = Ahb;
    static constexpr uint32_t Dma = Ahb;

    static constexpr uint32_t Dbg = Apb;
    static constexpr uint32_t Timer17 = Apb;
    static constexpr uint32_t Timer16 = Apb;
    static constexpr uint32_t Timer15 = Apb;
    static constexpr uint32_t Usart1 = Apb;
    static constexpr uint32_t Spi1 = Apb;
    static constexpr uint32_t I2s1 = Apb;
    static constexpr uint32_t Timer1 = Apb;
    static constexpr uint32_t Adc1 = Apb;
    static constexpr uint32_t Comp = Apb;
    static constexpr uint32_t ItLine = Apb;
    static constexpr uint32_t VrefBuf = Apb;
    static constexpr uint32_t SysCfg = Apb;
    static constexpr uint32_t Tamp = Apb;
    static constexpr uint32_t Bkp = Apb;
    static constexpr uint32_t Ucpd2 = Apb;
    static constexpr uint32_t Ucpd1 = Apb;
    static constexpr uint32_t LpTimer2 = Apb;
    static constexpr uint32_t LpUart1 = Apb;
    static constexpr uint32_t LpTimer1 = Apb;
    static constexpr uint32_t HdmiCec = Apb;
    static constexpr uint32_t Dac = Apb;
    static constexpr uint32_t Pwr = Apb;
    static constexpr uint32_t I2c2 = Apb;
    static constexpr uint32_t I2c1 = Apb;
    static constexpr uint32_t Usart4 = Apb;
    static constexpr uint32_t Usart3 = Apb;
    static constexpr uint32_t Usart2 = Apb;
    static constexpr uint32_t Spi2 = Apb;
    static constexpr uint32_t Iwdg = Rcc::LsiFrequency;
    static constexpr uint32_t Wwdg = Apb;
    static constexpr uint32_t Rtc = Apb;
    static constexpr uint32_t Timer14 = Apb;
    static constexpr uint32_t Timer7 = Apb;
    static constexpr uint32_t Timer6 = Apb;
    static constexpr uint32_t Timer3 = Apb;
    static constexpr uint32_t Timer2 = Apb;

    static bool inline enable() {
        Rcc::enableInternalClock();  // 48MHz
        /* // (internal clock / 1_M) * 8_N / 2_R = 128MHz / 2 = 64MHz
        const Rcc::PllFactors pllFactors{
                .pllM = 1,
                .pllN = 8,
                .pllR = 2,
        };
        Rcc::enablePll(Rcc::PllSource::InternalClock, pllFactors); */
        Rcc::setFlashLatency<Frequency>();
        // switch system clock to PLL output
        // Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
        Rcc::setApbPrescaler(Rcc::ApbPrescaler::Div1);
        // update frequencies for busy-wait delay functions
        Rcc::updateCoreFrequency<Frequency>();

        return true;
    }
};

using LedGn = GpioC15;
using LedRd = GpioC14;

using Leds = SoftwareGpioPort<LedGn, LedRd>;
/// @}

/*namespace stlink
{
/// @ingroup modm_board_custom_stm32c0x modm_board_custom_stm32c0x
/// @{
using Rx = GpioInputA3;
using Tx = GpioOutputA2;
using Uart = BufferedUart<UsartHal2, UartTxBuffer<2048>>;
/// @}
}*/

/// @ingroup modm_board_custom_stm32c0x modm_board_custom_stm32c0x
/// @{
// using LoggerDevice = modm::IODeviceWrapper< stlink::Uart, modm::IOBuffer::BlockIfFull >;

inline void
initialize() {
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();

    // stlink::Uart::connect<stlink::Tx::Tx, stlink::Rx::Rx>();
    // stlink::Uart::initialize<SystemClock, 115200_Bd>();

    LedGn::setOutput();
    LedRd::setOutput();
}
/// @}

}  // namespace Board

#endif  // MODM_STM32_CUSTOM_STM32C0X_HPP
