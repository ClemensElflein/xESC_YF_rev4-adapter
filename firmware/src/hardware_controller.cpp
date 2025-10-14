#include "hardware/hardware_controller.hpp"
#include "xesc_yfr4_datatypes.h"
#include "COBS.h"
#ifdef CRC // Fix modm CRC macro conflict
#undef CRC
#endif
#include "CRC.h"

using namespace std::chrono_literals;

// Global variables remain in main.cpp for legacy function compatibility

namespace hardware {

    template<typename HardwareConfig>
    void HardwareController<HardwareConfig>::run() {
        // LED error pattern - ready for use when needed
        [[maybe_unused]] auto LEDSEQ_ERROR_LL_COMM = [this]() {
            ledseq_red.blink({ .on = 20, .off = 30, .limit_blink_cycles = 1, .post_pause = 0, .fulfill = true });
            };

        // Hardware-specific main application loop
        // Provides hardware-optimized LED control and application logic

        // Boot-up indication: 3 * quick green & red blink
        ledseq_green.blink({ .limit_blink_cycles = 3, .fulfill = true }); // Default = 200ms ON, 200ms OFF
        ledseq_red.blink({ .limit_blink_cycles = 3, .fulfill = true });   // Default = 200ms ON, 200ms OFF

        while (true) {
            //handle_host_rx_buffer();

            // Hardware-specific LED sequencer updates
            ledseq_green.loop();
            ledseq_red.loop();

            // Future expansion points:
            // - Hardware-specific UART communication
            // - Hardware-optimized ADC sampling  
            // - Version-specific motor control
            // - Hardware-dependent feature implementations

            modm::delay(1ms);
        }
    }

    // Explicit template instantiations for the hardware versions we support
    template class HardwareController<decltype(versions::v1_0)>;
    template class HardwareController<decltype(versions::v2_0)>;
}