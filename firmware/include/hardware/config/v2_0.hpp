#pragma once
#include "base_types.hpp"

namespace hardware::versions {
    constexpr VersionInfo v2_0_version{ .major = 2, .minor = 0 };

    using V2_0_LedGreen = modm::platform::GpioC15;
    using V2_0_LedRed = modm::platform::GpioB6;

    using V2_0_LedConfig = LedConfig<V2_0_LedGreen, V2_0_LedRed>;
    constexpr V2_0_LedConfig v2_0_led{};

    constexpr HardwareConfig<V2_0_LedConfig> v2_0{
        .version = v2_0_version,
        .led = v2_0_led
    };
}