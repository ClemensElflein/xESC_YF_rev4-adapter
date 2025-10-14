#pragma once
#include "LedSeq.hpp"
#include "config/v1_0.hpp"
#include "config/v2_0.hpp"
#include "disable_nrst.hpp"

namespace hardware {

    /**
     * @brief Hardware-specific controller that runs the main application logic
     *
     * This template class is instantiated with the specific hardware configuration detected at runtime,
     * providing compile-time optimized code for each hardware version.
     */
    template<typename HardwareConfig>
    class HardwareController {
    public:
        explicit HardwareController(const HardwareConfig& config)
            : hw_config(config) {
            // Initialize GPIO pins using the template types
            LedConfigType::Green::setOutput();
            LedConfigType::Red::setOutput();
        }

        /**
         * @brief Run the main application loop
         * This replaces the main loop logic from main.cpp
         */
        void run();

        /**
         * @brief Get reference to green LED sequencer
         */
        auto& getGreenLed() { return ledseq_green; }

        /**
         * @brief Get reference to red LED sequencer
         */
        auto& getRedLed() { return ledseq_red; }

        /**
         * @brief Disable NRST pin functionality
         */
        void disableNrstPin() {
            ::disable_nrst(ledseq_green, ledseq_red);
        }

    private:
        const HardwareConfig& hw_config;

        // Extract LedConfig type from HardwareConfig 
        using LedConfigType = typename HardwareConfig::LedConfig;

        // Hardware-specific LED sequencers using the LED types from config
        LedSeq<typename LedConfigType::Green> ledseq_green;
        LedSeq<typename LedConfigType::Red> ledseq_red;
    };

    // Type aliases for convenience using the complete HardwareConfig
    using V1Controller = HardwareController<decltype(versions::v1_0)>;
    using V2Controller = HardwareController<decltype(versions::v2_0)>;
}