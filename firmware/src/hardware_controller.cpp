#include "hardware/hardware_controller.hpp"
#include "xesc_yfr4_datatypes.h"
#include "COBS.h"
#ifdef CRC // Fix modm CRC macro conflict
#undef CRC
#endif
#include "CRC.h"
#include "AdcSampler.hpp"
#include "board.hpp"
#include "config.h"

using namespace std::chrono_literals;

// Global variables remain in main.cpp for legacy function compatibility

namespace hardware {

    template<const auto& Config>
    void HardwareController<Config>::run() {
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
            // Handle incoming UART data
            host_comm.handleRxData();

            // Hardware-specific LED sequencer updates
            ledseq_green.loop();
            ledseq_red.loop();

            // Future expansion points:
            // - Hardware-specific UART communication
            // - Hardware-optimized ADC sampling  
            // - Version-specific motor control
            // - Hardware-dependent feature implementations

            //modm::delay(1ms);
        }
    }

    template<const auto& Config>
    void HardwareController<Config>::jumpSystemBootloader() {
        void (*SysMemBootJump)(void);

        // Disable Timer & interrupts
        Timer14::disableInterrupt(Timer14::Interrupt::Update);
        Timer14::disable();
        Timer1::disableInterrupt(Timer1::Interrupt::CaptureCompare4);
        Timer1::disable();
        AdcSampler::disable();

        __disable_irq();    // Disable all interrupts
        SysTick->CTRL = 0;  // Disable Systick timer
        Rcc::setHsiSysDivider(Rcc::HsiSysDivider::Div4);  // Default HsiSysDivider = boot frequency (12MHz)

        // Clear Interrupt Enable Register & Interrupt Pending Register
        for (size_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }
        __enable_irq();  // Re-enable all interrupts

        // Set up the jump to boot loader address + 4
        SysMemBootJump = (void (*)(void))(*((uint32_t*)((BOOTLOADER_ADDR + 4))));

        // Set the main stack pointer to the boot loader stack
        __set_MSP(*(uint32_t*)BOOTLOADER_ADDR);

        // Call the function to jump to boot loader location
        SysMemBootJump();

        // Jump is done successfully, we should never reach here!
        // Use LED sequencers for indication
        ledseq_green.on();
        ledseq_red.off();
        while (1) {
            // Infinite loop - we should never get here
        }
    }

    // Explicit template instantiations for the hardware versions we support
    template class HardwareController<versions::v1_0>;
    template class HardwareController<versions::v2_0>;
}