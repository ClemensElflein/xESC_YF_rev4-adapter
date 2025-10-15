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

#include <modm/platform.hpp>

#include <modm/platform.hpp>

#include "COBS.h"
#include "LedSeq.hpp"
#include "board.hpp"
#include "config.h"
#include "hardware/hw_version.hpp"
#ifdef CRC // FIXME remove modm/STM32 CRC or use it
#undef CRC
#endif
#include "AdcSampler.hpp"
#include "CRC.h"
#include "disable_nrst.hpp"
#include "jump_system_bootloader.hpp"
#include "xesc_yfr4_datatypes.h"
#include "debug.h"

using namespace Board;
using namespace std::chrono_literals;
using namespace std;

#define MILLIS modm::Clock::now().time_since_epoch().count()

// LED sequencer
LedSeq<LedGreen> ledseq_green;
LedSeq<LedRed> ledseq_red;
#define LEDSEQ_ERROR_LL_COMM ledseq_red.blink({.on = 20, .off = 30, .limit_blink_cycles = 1, .post_pause = 0, .fulfill = true})

// Host comms (COBS)
#define COBS_BUFFER_SIZE 100                // Should be at least size of biggest RX/TX message + some COBS overhead bytes
static uint8_t buffer_rx[COBS_BUFFER_SIZE]; // Serial RX buffer for COBS encoded data up to COBS end marker
static unsigned int buffer_rx_idx = 0;
static uint8_t buffer_tx[COBS_BUFFER_SIZE]; // Serial TX buffer for COBS encoded messages
COBS cobs;

// Misc
std::array<uint16_t, AdcSampler::sequence.size()> AdcSampler::_data = {}; // Definition of AdcSampler's private _data buffer (initialized with 0)

volatile uint32_t last_watchdog_millis = 0;
volatile uint32_t last_fault_millis = 0;

// The current duty for the motor (-1.0 - 1.0)
volatile float duty = 0.0;
// The absolute value to cap the requested duty to stay within current limits
//(0.0 - 1.0) volatile float current_limit_duty = 0.0;*/
// The requested duty
float duty_setpoint = 0.0;
// init with out of range duty, so we have to update
// volatile float last_duty = 1000.0f;

// SA (Hall) Input - CaptureCompare Timer Configuration
volatile uint32_t sa_ticks = 0; // SA Timer Ticks between 2 SA Signals. Has to be larger than tick register due to overflow
volatile uint32_t sa_tacho = 0; // SA Tacho ticks (1 ticks/360)

XescYFR4StatusPacket status = {};
XescYFR4SettingsPacket settings = {};
bool settings_valid = false;

// ============================================================================
// LEGACY CODE - Temporary stub for sendMessage (TODO: migrate to HostComm)
// ============================================================================
void sendMessage(void* message, size_t size) {
    // TODO: Integrate with HardwareController's HostComm
    // For now, this is a no-op stub to allow compilation
    (void)message;
    (void)size;
}

void update_faults() {
    uint32_t faults = 0;

    // FAULT_WRONG_HW_VERSION - Check if wrong hardware version firmware is running
    if (!hardware::checkFlashMatchesBinary()) {
        faults |= FAULT_WRONG_HW_VERSION;
    }

    // FAULT_UNINITIALIZED
    if (!settings_valid) {
        faults |= FAULT_UNINITIALIZED;
    }

    // FAULT_WATCHDOG
    if (MILLIS - last_watchdog_millis > WATCHDOG_TIMEOUT_MILLIS) {
        faults |= FAULT_WATCHDOG;
    }

    // FAULT_OVERCURRENT
    if (status.current_input > (double)HW_LIMIT_CURRENT) {
        faults |= FAULT_OVERCURRENT;
    }

    // FAULT_OVERTEMP_PCB
    if (status.temperature_pcb > (double)min(HW_LIMIT_PCB_TEMP, settings.max_pcb_temp)) {
        faults |= FAULT_OVERTEMP_PCB;
    }

#ifdef HW_V1
    // VMS FAULTs
    if (!vm_switch::Fault::read()) {
        if (vm_switch::In::isSet()) { // VM-Switch is "on"
            // Disable VM-Switch diagnostics when switched on,
            // because there's something miss-understood by me, or wrong with my PCB design
            // faults |= FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT;  // Not fully clear if VMS Thermal Error and/or Overload/short
        } else if (!host::Shutdown::read()) {
            faults |= FAULT_OPEN_LOAD;
        }
    }
#endif

    if (faults) {
        status.fault_code = faults;
        last_fault_millis = MILLIS;

        // Green LED
        if (faults & FAULT_UNINITIALIZED) { // Open VMC
            ledseq_green.blink({ .on = 500, .off = 500 });
        } else {
            ledseq_green.off();
        }
        // Red LED by priority
        if (faults & FAULT_WRONG_HW_VERSION) {
            ledseq_red.on(); // Red LED steady on (clear indication)
        } else if (faults & FAULT_OPEN_LOAD) {
            ledseq_red.blink({ .on = 125, .off = 125 }); // Quick blink (4Hz)
        } else if (faults & (FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT)) {
            ledseq_red.blink({ .on = 250, .off = 250 }); // Fast blink (2Hz)
        } else if (faults & FAULT_WATCHDOG) {
            ledseq_red.blink({ .on = 500, .off = 500 });
        }
    } else if (faults == 0) {
        if (host::Shutdown::read()) {
            ledseq_green.blink({ .on = 100, .off = 1800, .limit_blink_cycles = 1, .fulfill = true }); // Default = 200ms ON, 200ms OFF
        } else {
            ledseq_green.on();
        }

        if (status.fault_code != 0) {
            // Reset faults only if MIN_FAULT_TIME_MILLIS passed, or if it was a dedicated watchdog fault
            if (MILLIS - last_fault_millis > MIN_FAULT_TIME_MILLIS || status.fault_code == FAULT_WATCHDOG) {
                ledseq_red.off();
                status.fault_code = 0;
            }
        }
    }
}

void set_motor_state() {
    float new_duty = (status.fault_code || host::Shutdown::read()) ? 0.0f : duty_setpoint;
    if (new_duty == duty) {
        return;
    }
    if (new_duty == 0.0f) {
        // Stop motor
        motor::RS::set();  // !RS off
        motor::Brk::set(); // Break
    } else {
        // Start motor
        motor::Brk::reset(); // Release break
        motor::RS::reset();  // !RS on
    }
    duty = new_duty;
}

/**
 * @brief update_status() get called every STATUS_CYCLE (by hardware timer)
 */
void update_status() {
    static uint8_t motor_stopped_cycles = 0; // No tacho change cycle counter
    static uint16_t rpm;

    status.seq++;
    update_faults();

    // Enable/disable VMC
    if (!(status.fault_code & FAULT_OPEN_LOAD) && !host::Shutdown::read()) { // Motor connected && no Shutdown signal
        vm_switch::In::set();
        Timer1::enableInterrupt(Timer1::Interrupt::CaptureCompare4);
    } else if (motor_stopped_cycles > NUM_STATUS_CYCLES_MOTOR_STOPPED) { // Check if at least NUM_STATUS_CYCLES_MOTOR_STOPPED times
        Timer1::disableInterrupt(Timer1::Interrupt::CaptureCompare4);
        vm_switch::In::reset();
    }

    set_motor_state();

    if (duty == 0.0f) {
        // Check if motor stopped rotating
        if (sa_tacho == status.tacho) { // Tacho equals last-tacho value
            // Motor (looks like) stopped (but same tacho values might also happen due to an INT/tacho error or due to slow RPM)
            if (motor_stopped_cycles > NUM_STATUS_CYCLES_MOTOR_STOPPED) { // Check if at least NUM_STATUS_CYCLES_MOTOR_STOPPED times
                sa_ticks = 0;
                rpm = 0;
                motor::Brk::reset(); // Release break
            } else {
                motor_stopped_cycles++;
            }
        } else {
            motor_stopped_cycles = 0;
        }
    }

    if (sa_ticks) {
        /* Calc RPM based on sa_ticks
         * RPM = 60 / ( (1/TimClock) * TimPrescaler * (CapCompTicks * 4 (Cycles/360°) / InputPrescaler) ) =>
         * RPM = (15 * TimClock * InputPrescaler) / (TimPrescaler * CapCompTicks)
         */
        rpm = (15 * SystemClock::Timer1 * SA_TIMER_INPUT_PRESCALER) / SA_TIMER_PRESCALER / sa_ticks;
    }

#ifdef PROTO_DEBUG
    MODM_LOG_INFO << "now=" << MILLIS << "ms status.fault_code=" << status.fault_code;
#ifdef PROTO_DEBUG_MOTOR
    MODM_LOG_DEBUG << " sa_tacho=" << (uint32_t)sa_tacho
        << " sa_ticks=" << (uint32_t)sa_ticks
        << " rpm=" << rpm;
#endif
#ifdef PROTO_DEBUG_ADC
    MODM_LOG_DEBUG << " VRef=" << AdcSampler::getInternalVref_u() << "mV, " << AdcSampler::getInternalVref_f() << "mV"
        << " Temp=" << AdcSampler::getInternalTemp()
        << " CurrentSense=" << AdcSampler::getVoltage(AdcSampler::Sensors::CurSense) << "V"
        << ", " << AdcSampler::getVoltage(AdcSampler::Sensors::CurSense) / (CUR_SENSE_GAIN * R_SHUNT) << "A"
        << " CurrentSense_2=" << AdcSampler::getVoltage(AdcSampler::Sensors::CurSense2) << "V"
        << ", " << AdcSampler::getVoltage(AdcSampler::Sensors::CurSense2) / (CUR_SENSE_2_GAIN * R_SHUNT) << "A";
#endif
    MODM_LOG_INFO
        << modm::endl
        << modm::flush;
#endif

    status.duty_cycle = duty;
    status.tacho = sa_tacho;
    status.tacho_absolute = sa_tacho;
    status.rpm = rpm;
    status.temperature_pcb = AdcSampler::getInternalTemp();
    status.current_input = AdcSampler::getVoltage(AdcSampler::Sensors::CurSense) / (CUR_SENSE_GAIN * R_SHUNT);

    sendMessage(&status, sizeof(status));
}

// Status Timer ISR
MODM_ISR(TIM14) {
    Timer14::acknowledgeInterruptFlags(Timer14::InterruptFlag::Update);
    update_status();
}

/**
 * @brief buffer_rx has a complete COBS encoded packet (incl. COBS end marker)
 *
 */
void PacketReceived() {
    static uint8_t pkt_buffer[COBS_BUFFER_SIZE]; // COBS decoded packet buffer

#ifdef PROTO_DEBUG_HOST_RX
    MODM_LOG_DEBUG << "RX[" << buffer_rx_idx << " bytes] COBS:";
    for (unsigned int i = 0; i < buffer_rx_idx; ++i) MODM_LOG_DEBUG << " " << HEX_BYTE(buffer_rx[i]);
    MODM_LOG_DEBUG << modm::endl;
#endif

    size_t pkt_size = cobs.decode(buffer_rx, buffer_rx_idx - 1, (uint8_t*)pkt_buffer);

#ifdef PROTO_DEBUG_HOST_RX
    MODM_LOG_DEBUG << "    [" << pkt_size << " bytes] Decoded:";
    for (size_t i = 0; i < pkt_size; ++i) MODM_LOG_DEBUG << " " << HEX_BYTE(pkt_buffer[i]);
    MODM_LOG_DEBUG << modm::endl << modm::flush;
#endif

    // calculate the CRC only if we have at least three bytes (two CRC, one data)
    if (pkt_size < 3) {
#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "    ERROR: Packet too short" << modm::endl << modm::flush;
#endif
        LEDSEQ_ERROR_LL_COMM;
        return;
    }

    // Check CRC
    uint16_t crc = CRC::Calculate(pkt_buffer, pkt_size - 2, CRC::CRC_16_CCITTFALSE());
    if (pkt_buffer[pkt_size - 1] != ((crc >> 8) & 0xFF) ||
        pkt_buffer[pkt_size - 2] != (crc & 0xFF)) {
#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "    ERROR: CRC mismatch (expected: "
            << HEX_BYTE(crc >> 8) << HEX_BYTE(crc & 0xFF)
            << ", got: " << HEX_BYTE(pkt_buffer[pkt_size - 2]) << HEX_BYTE(pkt_buffer[pkt_size - 1])
            << ")" << modm::endl << modm::flush;
#endif            
        LEDSEQ_ERROR_LL_COMM;
        return;
    }

    switch (pkt_buffer[0]) {
    case XESCYFR4_MSG_TYPE_CONTROL:
    {
        if (pkt_size != sizeof(struct XescYFR4ControlPacket)) {
            LEDSEQ_ERROR_LL_COMM;
            return;
        }
#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "    Type: CONTROL" << modm::endl << modm::flush;
#endif
        // Got control packet
        last_watchdog_millis = MILLIS;
        XescYFR4ControlPacket* packet = (XescYFR4ControlPacket*)pkt_buffer;
        duty_setpoint = packet->duty_cycle;
        set_motor_state();
    }
    break;
    case XESCYFR4_MSG_TYPE_SETTINGS:
    {
        if (pkt_size != sizeof(struct XescYFR4SettingsPacket)) {
            settings_valid = false;
            LEDSEQ_ERROR_LL_COMM;
            return;
        }
#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "    Type: SETTINGS" << modm::endl << modm::flush;
#endif
        XescYFR4SettingsPacket* packet = (XescYFR4SettingsPacket*)pkt_buffer;
        settings = *packet;
        settings_valid = true;
    }
    break;

    default:
        // Wrong/unknown packet type
#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "    ERROR: Unknown packet type 0x" << HEX_BYTE(pkt_buffer[0])
            << modm::endl << modm::flush;
#endif
        LEDSEQ_ERROR_LL_COMM;
        break;
    }
}

/**
 * @brief Handle data in LowLevel-UART RX buffer and wait (buffer) for COBS end marker
 */
void handle_host_rx_buffer() {
    uint8_t data;
    while (host::Uart::read(data)) {
        buffer_rx[buffer_rx_idx++] = data;
        if (buffer_rx_idx >= COBS_BUFFER_SIZE) { // Buffer is full, but no COBS end marker. Reset
            LEDSEQ_ERROR_LL_COMM;
            buffer_rx_idx = 0;
            return;
        }

        if (data == 0) { // COBS end marker
            PacketReceived();
            buffer_rx_idx = 0;
            return;
        }

        // Bootloader trigger string?
        if (buffer_rx_idx == sizeof BOOTLOADER_TRIGGER_STR && strcmp((const char*)buffer_tx, BOOTLOADER_TRIGGER_STR)) {
            //jump_system_bootloader();
        }
    }
}

/**
 * @brief ISR execute when a Capture Compare interruption is triggered via motor::sa pin.
 */
MODM_ISR(TIM1_CC) {
    static uint16_t last_cc_value = 0;
    uint16_t temp_cc_value = Timer1::getCompareValue<motor::SATimChan>();
    uint32_t temp_ticks;

    Timer1::acknowledgeInterruptFlags(motor::SACaptureFlag);
    sa_tacho++;

    if (Timer1::getInterruptFlags() & Timer1::InterruptFlag::Update) {
        Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
        // MODM_LOG_DEBUG << "UPD" << modm::endl;
        temp_ticks = temp_cc_value + (0xffff - last_cc_value);
    } else {
        temp_ticks = temp_cc_value - last_cc_value;
    }
    // MODM_LOG_DEBUG << temp_cc_value << "-" << last_cc_value << "=" << (uint16_t)temp_ticks << modm::endl;
    // MODM_LOG_DEBUG << (uint16_t)sa_ticks << modm::endl;

    last_cc_value = temp_cc_value;
    if (temp_ticks < SA_TIMER_MIN_TICKS) {
        return;
    }
    sa_ticks = temp_ticks;
}

int main() {
    // Initialize common hardware stuff
    Board::initialize();

#ifdef PROTO_DEBUG
    // Print hardware version info
    MODM_LOG_INFO << "xESC_YF_rev4 - ";
    hardware::printVersionInfo(modm::log::info);
    MODM_LOG_INFO << modm::endl;
#endif

    // Hardware-specific initialization
#ifdef HW_V1
    disable_nrst(); // Check NRST pin. Will flash if wrong and reset
    vm_switch::DiagEnable::set(); // V1 firmware on V1 hardware - safe to enable
#else  // HW_V2
    vm_switch::DiagEnable::set(); // V2+ hardware - always safe to enable diagnostics
#endif

    AdcSampler::init(); // Init & run AdcSampler

    // Prepare status msg
    status.message_type = XESCYFR4_MSG_TYPE_STATUS;
    status.fw_version_major = 0;
    status.fw_version_minor = 2;
    status.direction = 0; // Our motor has only one direction

#ifdef FALSE
    // SA (Hall) input - Capture/Compare timer
    Timer1::connect<motor::SATimChan>();
    Timer1::enable();
    Timer1::setMode(Timer1::Mode::UpCounter);
    Timer1::setPrescaler(SA_TIMER_PRESCALER);
    Timer1::setOverflow(0xFFFF);
    Timer1::configureInputChannel<motor::SATimChan>(Timer1::InputCaptureMapping::InputOwn,
        Timer1::InputCapturePrescaler::Div1, // has to match SA_TIMER_INPUT_PRESCALER
        Timer1::InputCapturePolarity::Rising,
        SA_TIMER_MIN_TICKS); // Adapt filter when changing prescaler
    Timer1::enableInterrupt(motor::SACaptureInterrupt);
    Timer1::enableInterruptVector(motor::SACaptureInterrupt, true, 21);
    Timer1::applyAndReset();
    Timer1::start();

    // Status Timer
    Timer14::enable();
    Timer14::setMode(Timer14::Mode::UpCounter);
    Timer14::setPeriod<Board::SystemClock>(STATUS_CYCLE);
    Timer14::enableInterrupt(Timer14::Interrupt::Update);
    Timer14::enableInterruptVector(true, 26);
    Timer14::applyAndReset();
    Timer14::start();
#endif

    // LED blink code "Boot up successful"
    ledseq_green.blink({ .limit_blink_cycles = 3, .fulfill = true }); // Default = 200ms ON, 200ms OFF
    ledseq_red.blink({ .limit_blink_cycles = 3, .fulfill = true });   // Default = 200ms ON, 200ms OFF

    while (true) {
        handle_host_rx_buffer();
        ledseq_green.loop();
        ledseq_red.loop();
    }
}
