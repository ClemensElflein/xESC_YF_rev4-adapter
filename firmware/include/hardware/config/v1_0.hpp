#pragma once
#include "base_types.hpp"

namespace hardware::versions {
    constexpr VersionInfo v1_0_version{ .major = 1, .minor = 0 };

    using V1_0_LedConfig = LedConfig<modm::platform::GpioC15, modm::platform::GpioC14>;
    constexpr V1_0_LedConfig v1_0_led{};

    constexpr HardwareConfig<V1_0_LedConfig> v1_0{
        .version = v1_0_version,
        .led = v1_0_led
    };
}