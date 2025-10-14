#pragma once
#include "config_manager.hpp"

namespace hardware {
    /**
     * @brief Initialize hardware-specific components based on detected hardware version
     * @tparam HwConfig Hardware configuration type (v1_0, v2_0, etc.)
     * @param hwConfig Hardware configuration instance
     */
    template<typename HwConfig>
    void initializeHardwareSpecific(const HwConfig& hwConfig) {
        // Initialize hardware-specific LEDs
        using GreenLed = typename decltype(hwConfig.led)::Green;
        using RedLed = typename decltype(hwConfig.led)::Red;

        // Configure GPIO pins
        GreenLed::setOutput();
        RedLed::setOutput();

        // Set initial state - green on during startup
        GreenLed::set();
        RedLed::reset();

        // Motor controls are version-dependent and handled by board.hpp
        // We don't need to initialize them here as Board::initialize() handles it
        // The motor namespace already handles version-specific pin assignments
    }

    /**
     * @brief Initialize hardware based on flash configuration
     * @param flashConfig Flash configuration read from OTP area
     */
    inline void initializeHardware(const FlashHardwareConfig& flashConfig) {
        dispatchHardwareConfig(flashConfig, [](const auto& hwConfig) {
            initializeHardwareSpecific(hwConfig);
            });
    }
}