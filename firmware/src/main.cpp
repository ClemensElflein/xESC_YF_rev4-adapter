// Created by Apehaenger on 2024-06-14.
//
// This file is part of the openmower project.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <modm/debug/logger.hpp>
#include <modm/platform.hpp>

#include "COBS.h"
#include "LedSeq.hpp"
#include "board.hpp"
#include "config.h"
#ifdef CRC  // FIXME remove modm/STM32 CRC or use it
#undef CRC
#endif
#include "CRC.h"
#include "xesc_yfr4_datatypes.h"

using namespace Board;
using namespace std::chrono_literals;
using namespace std;

// UART print debugging via proto_uart
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG
// Create an IODeviceWrapper around the Uart Peripheral we want to use
modm::IODeviceWrapper<proto_uart::Uart, modm::IOBuffer::BlockIfFull> loggerDevice;
// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

#define MILLIS modm::Clock::now().time_since_epoch().count()

// modm::ui::Led red([](uint8_t brightness) {
// Timer3::setCompareValue<Board::LedRd::Ch2>(modm::ui::table22_16_256[brightness]);
//});

LedSeq<LedGreen> ledseq_green;
LedSeq<LedRed> ledseq_red;
#define LEDSEQ_ERROR_LL_COMM ledseq_red.blink({.on = 20, .off = 30, .limit_blink_cycles = 1, .post_pause = 0, .fulfill = true})

#define COBS_BUFFER_SIZE 200
static uint8_t buffer_rx[COBS_BUFFER_SIZE];  // Serial RX buffer for COBS encoded data up to COBS end marker
static unsigned int idx_buffer_rx = 0;
COBS cobs;

volatile unsigned long last_watchdog_millis = 0;
volatile unsigned long last_fault_millis = 0;

// The current duty for the motor (-1.0 - 1.0)
volatile float duty = 0.0;
// The absolute value to cap the requested duty to stay within current limits
//(0.0 - 1.0) volatile float current_limit_duty = 0.0;*/
// The requested duty
float duty_setpoint = 0.0;
// init with out of range duty, so we have to update
// volatile float last_duty = 1000.0f;

volatile uint32_t tacho = 0;
volatile unsigned int sa_cycles = 0;  // SA cycles since last status

XescYFR4StatusPacket status = {};
XescYFR4SettingsPacket settings = {};
bool settings_valid = false;

void sendMessage(void *message, size_t size) {
    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of
    if (size < 4) {
        LEDSEQ_ERROR_LL_COMM;
        return;
    }
    uint8_t *data_pointer = (uint8_t *)message;

    // Calc CRC
    uint16_t crc = CRC::Calculate((uint8_t *)message, size - 2, CRC::CRC_16_CCITTFALSE());
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    host_uart::Uart::write((uint8_t *)message, size);
}

void updateFaults() {
    uint32_t faults = 0;

    // FAULT_UNINITIALIZED
    if (!settings_valid) {
        faults |= FAULT_UNINITIALIZED;
    }

    // FAULT_WATCHDOG
    // TODO: use/add STMs IWDG?
    if (MILLIS - last_watchdog_millis > WATCHDOG_TIMEOUT_MILLIS) {
        faults |= FAULT_WATCHDOG;
    }

    // VMS FAULTs
    if (!vm_switch::Fault::read()) {
        if (vm_switch::In::isSet()) {                          // VM-Switch is "on"
            faults |= FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT;  // Not fully clear if VMS Thermal Error and/or Overload/short
        } else {
            faults |= FAULT_INVALID_HALL;  // Alt-use as OPEN_LOAD
        }
    }

    if (faults) {
        status.fault_code = faults;
        last_fault_millis = MILLIS;

        // Green LED
        if (faults & FAULT_UNINITIALIZED) {  // Open VMC
            ledseq_green.blink({.on = 500, .off = 500});
        } else {
            ledseq_green.off();
        }
        // Red LED by priority
        if (faults & FAULT_INVALID_HALL) {              // Open VMC
            ledseq_red.blink({.on = 125, .off = 125});  // Quick blink (4Hz)
        } else if (faults & (FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT)) {
            ledseq_red.blink({.on = 250, .off = 250});  // Fast blink (2Hz)
        } else if (faults & FAULT_WATCHDOG) {
            ledseq_red.blink({.on = 500, .off = 500});
        }
    } else if (faults == 0) {
        ledseq_green.on();
        if (status.fault_code != 0) {
            // Reset faults only if MIN_FAULT_TIME_MILLIS passed, or if it was a dedicated watchdog fault
            if (MILLIS - last_fault_millis > MIN_FAULT_TIME_MILLIS || status.fault_code == FAULT_WATCHDOG) {
                ledseq_red.off();
                status.fault_code = 0;
            }
        }
    }
}

/**
 * @brief send_status
 * Get called every STATUS_UPDATE_MICROS (by hardware timer)
 */
void send_status() {
    unsigned int sa_cycles_bak = sa_cycles;  // Buffer SA cycles because the next cycle might happen within the next 3ms
    sa_cycles = 0;
    status.seq++;
    tacho += sa_cycles_bak;
    updateFaults();

    /* Some SA cycle infos
     *
     * My stock motor stats:
     *   Nom RPM @ 24VDC = 3488rpm = 4.3ms/SA cycle whereas 4 cycles = 360°
     */
    uint32_t now = MILLIS;
    /*if (now >= next_rpm_calc_millis) {
        uint32_t diff_millis = RPM_CALC_CYCLE_MILLIS + (now - next_rpm_calc_millis);
        next_rpm_calc_millis = now + RPM_CALC_CYCLE_MILLIS;
        unsigned int diff_tacho = tacho - last_rpm_calc_tacho;  // FIXME: Handle overflow?!
        last_rpm_calc_tacho = tacho;
        rpm = (60000.0f / diff_millis) * diff_tacho / 4;
    }*/

    MODM_LOG_INFO << "now " << now << "ms, SA " << motor::SA::read() << ", sa_cycles_bak " << sa_cycles_bak << ", tacho " << (uint32_t)tacho << modm::endl
                  << modm::flush;

    status.duty_cycle = duty;
    status.tacho = tacho;
    status.tacho_absolute = tacho;

    sendMessage(&status, sizeof(status));
}

// Interrupt Handler
MODM_ISR(TIM14) {
    Timer14::acknowledgeInterruptFlags(Timer14::InterruptFlag::Update);
    send_status();
}

void commitMotorState() {
    float new_duty;
    if (status.fault_code) {
        new_duty = 0.0f;
    } else {
        new_duty = duty_setpoint;
    }
    if (new_duty == duty) {
        return;
    }

    if (new_duty == 0.0f) {  // Stop motor
        motor::RS::set();    // !RS off
        motor::Brk::set();   // TODO: Release BRK once RPM=0?
        // digitalWrite(PIN_VMS_IN, LOW); // TODO: Switch off once RPM=0?
    } else {                   // Start motor
        vm_switch::In::set();  // TODO Wire & health check result
        motor::Brk::reset();   // Release break
        motor::RS::reset();    // !RS on
    }
    duty = new_duty;  // FIXME: Move to BRK release or RPM measurement?
}

/**
 * @brief buffer_rx has a complete COBS encoded packet (incl. COBS end marker)
 *
 */
void PacketReceived() {
    static uint8_t pkt_buffer[COBS_BUFFER_SIZE];  // COBS decoded packet buffer

    size_t pkt_size = cobs.decode(buffer_rx, idx_buffer_rx - 1, (uint8_t *)pkt_buffer);

    // calculate the CRC only if we have at least three bytes (two CRC, one data)
    if (pkt_size < 3) {
        LEDSEQ_ERROR_LL_COMM;
        return;
    }

    // Check CRC
    uint16_t crc = CRC::Calculate(pkt_buffer, pkt_size - 2, CRC::CRC_16_CCITTFALSE());
    if (pkt_buffer[pkt_size - 1] != ((crc >> 8) & 0xFF) ||
        pkt_buffer[pkt_size - 2] != (crc & 0xFF)) {
        LEDSEQ_ERROR_LL_COMM;
        return;
    }

    switch (pkt_buffer[0]) {
        case XESCYFR4_MSG_TYPE_CONTROL: {
            if (pkt_size != sizeof(struct XescYFR4ControlPacket)) {
                LEDSEQ_ERROR_LL_COMM;
                return;
            }
            // Got control packet
            last_watchdog_millis = MILLIS;
            XescYFR4ControlPacket *packet = (XescYFR4ControlPacket *)pkt_buffer;
            duty_setpoint = packet->duty_cycle;
            commitMotorState();
        } break;
        case XESCYFR4_MSG_TYPE_SETTINGS: {
            if (pkt_size != sizeof(struct XescYFR4SettingsPacket)) {
                settings_valid = false;
                LEDSEQ_ERROR_LL_COMM;
                return;
            }
            XescYFR4SettingsPacket *packet = (XescYFR4SettingsPacket *)pkt_buffer;
            settings = *packet;
            settings_valid = true;
        } break;

        default:
            // Wrong/unknown packet type
            LEDSEQ_ERROR_LL_COMM;
            break;
    }
}

/**
 * @brief Handle data in LowLevel-UART RX buffer and wait (buffer) for COBS end marker
 */
void handleLowLevelRxBuffer() {
    uint8_t data;
    while (host_uart::Uart::read(data)) {
        // MODM_LOG_DEBUG << "RX data " << data << modm::endl;
        buffer_rx[idx_buffer_rx] = data;
        idx_buffer_rx++;
        if (idx_buffer_rx >= COBS_BUFFER_SIZE) {  // Buffer is full, but no separator. Reset
            LEDSEQ_ERROR_LL_COMM;
            idx_buffer_rx = 0;
            return;
        }

        if (data == 0) {  // COBS end marker
            MODM_LOG_DEBUG << "buffer_rx[" << idx_buffer_rx << "]:";
            for (unsigned int i = 0; i < idx_buffer_rx; ++i) {
                MODM_LOG_DEBUG << " " << buffer_rx[i];
            }
            MODM_LOG_DEBUG << modm::endl
                           << modm::flush;
            PacketReceived();
            idx_buffer_rx = 0;
            return;
        }
    }
}

int main() {
    Board::initialize();

    // Initialize prototyping-UART for MODM_LOG_*
    proto_uart::Uart::connect<proto_uart::Tx::Tx, proto_uart::Rx::Rx>();
    proto_uart::Uart::initialize<SystemClock, 460800_Bd>();

    MODM_LOG_INFO << modm::endl
                  << modm::endl
                  << "Board initialized" << modm::endl;

    // Check NRST_MODE if GPIO mode (NRST pin disabled)
    if (!(FLASH->OPTR & FLASH_OPTR_NRST_MODE_1) || (FLASH->OPTR & FLASH_OPTR_NRST_MODE_0)) {
        LedRed::set();  // LED red = on
        for (;;);
    }

    // Initialize Host (LowLevel) UART communication
    // TODO: Is there a way to use a INT driven onDataReceived() handler?
    host_uart::Uart::connect<host_uart::Tx::Tx, host_uart::Rx::Rx>();
    host_uart::Uart::initialize<SystemClock, 115200_Bd>();

    // Status Timer
    Timer14::enable();
    Timer14::setMode(Timer14::Mode::UpCounter);
    Timer14::setPeriod<Board::SystemClock>(20ms);
    Timer14::applyAndReset();
    Timer14::enableInterrupt(Timer14::Interrupt::Update);
    Timer14::enableInterruptVector(true, 26);

    // LED blink code "Boot up successful"
    ledseq_green.blink({.limit_blink_cycles = 3, .fulfill = true});  // Default = 200ms ON, 200ms OFF
    ledseq_red.blink({.limit_blink_cycles = 3, .fulfill = true});    // Default = 200ms ON, 200ms OFF

    // Now that all is set-up and NRST is also disabled
    vm_switch::In::setOutput(modm::Gpio::High);          // VM-Switch, VMC = on
    vm_switch::DiagEnable::setOutput(modm::Gpio::High);  // VM-Switch diagnostics enable
    Timer14::start();

    while (true) {
        handleLowLevelRxBuffer();
        ledseq_green.loop();
        ledseq_red.loop();
    }
}
