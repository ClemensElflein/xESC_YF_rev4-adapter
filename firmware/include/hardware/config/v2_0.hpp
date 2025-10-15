#pragma once
#include "base_types.hpp"

namespace hardware::versions {
    constexpr VersionInfo v2_0_version{ .major = 2, .minor = 0 };

    using V2_0_LedGreen = modm::platform::GpioC15;
    using V2_0_LedRed = modm::platform::GpioB6;

    using V2_0_LedConfig = LedConfig<V2_0_LedGreen, V2_0_LedRed>;
    constexpr V2_0_LedConfig v2_0_led{};

    using V2_0_HostConfig = HostConfig<
        modm::platform::GpioInputC14,   // Shutdown
        modm::platform::GpioOutputA9,   // Tx
        modm::platform::GpioInputA10,   // Rx  
        modm::platform::UsartHal1       // UART HAL
    >;
    constexpr V2_0_HostConfig v2_0_host{};

    constexpr HardwareConfig<V2_0_LedConfig, V2_0_HostConfig> v2_0{
        .version = v2_0_version,
        .led = v2_0_led,
        .host = v2_0_host
    };
}