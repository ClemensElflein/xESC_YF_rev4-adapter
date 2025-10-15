#pragma once
#include "LedSeq.hpp"
#include "config/v1_0.hpp"
#include "config/v2_0.hpp"
#include "disable_nrst.hpp"
#include "host_comm.hpp"

namespace hardware {

    /**
     * @brief Hardware-specific controller that runs the main application logic
     *
     * This template class is instantiated with the specific hardware configuration detected at runtime,
     * providing compile-time optimized code for each hardware version.
     */
    template<const auto& Config>
    class HardwareController {
    public:
        // Public type alias so HostComm can access the config type
        using HardwareConfig = std::remove_cvref_t<decltype(Config)>;

        explicit HardwareController() : host_comm(*this) {
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

        /**
         * @brief Jump to system bootloader
         */
        void jumpSystemBootloader();

        /**
         * @brief Get reference to host communication
         */
        auto& getHostComm() { return host_comm; }

    private:
        // Extract LedConfig type from HardwareConfig 
        using LedConfigType = typename HardwareConfig::LedConfig;

        // Hardware-specific LED sequencers using the LED types from config
        // Must be declared before host_comm since host_comm uses references to them
        LedSeq<typename LedConfigType::Green> ledseq_green;
        LedSeq<typename LedConfigType::Red> ledseq_red;

        // Hardware-specific host communication (takes reference to *this for LED and config access)
        HostComm<HardwareController> host_comm;
    };

    // Type aliases for convenience - now using the config objects directly as template parameters
    using V1Controller = HardwareController<versions::v1_0>;
    using V2Controller = HardwareController<versions::v2_0>;
}