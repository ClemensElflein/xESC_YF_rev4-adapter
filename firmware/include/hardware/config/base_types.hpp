#pragma once
#include <modm/architecture.hpp>

namespace hardware {
    // Hardware configuration magic string for flash storage
    static constexpr const char HARDWARE_CONFIG_MAGIC[] = "ESCYFR4HWV";
    static constexpr size_t HARDWARE_CONFIG_MAGIC_SIZE = sizeof(HARDWARE_CONFIG_MAGIC);

    struct VersionInfo {
        uint8_t major;
        uint8_t minor;
    };

    // Template-based LED configuration preserving full GPIO type information
    template<typename GreenGpio, typename RedGpio>
    struct LedConfig {
        using Green = GreenGpio;
        using Red = RedGpio;
    };

    // Template-based hardware configuration
    template<typename LedConfigType>
    struct HardwareConfig {
        using LedConfig = LedConfigType;  // Type alias for template access
        
        const VersionInfo& version;
        LedConfigType led;
    };
}