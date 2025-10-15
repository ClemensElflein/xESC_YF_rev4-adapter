#pragma once
#include "base_types.hpp"

namespace hardware::versions {
    constexpr VersionInfo v1_0_version{ .major = 1, .minor = 0 };

    using V1_0_LedGreen = modm::platform::GpioC15;
    using V1_0_LedRed = modm::platform::GpioC14;

    using V1_0_LedConfig = LedConfig<V1_0_LedGreen, V1_0_LedRed>;
    constexpr V1_0_LedConfig v1_0_led{};


    using V1_0_HostConfig = HostConfig<
        modm::platform::GpioInputA8,    // Shutdown
        modm::platform::GpioOutputA9,   // Tx  
        modm::platform::GpioInputA10,   // Rx
        modm::platform::UsartHal1       // UART HAL
    >;
    constexpr V1_0_HostConfig v1_0_host{};

    constexpr HardwareConfig<V1_0_LedConfig, V1_0_HostConfig> v1_0{
        .version = v1_0_version,
        .led = v1_0_led,
        .host = v1_0_host
    };
}