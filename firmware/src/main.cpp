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

#include <modm/driver/adc/adc_sampler.hpp>
#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>

#include "COBS.h"
#include "LedSeq.hpp"
#include "board.hpp"
#include "config.h"
#ifdef CRC  // FIXME remove modm/STM32 CRC or use it
#undef CRC
#endif
#include "CRC.h"
#include "disable_nrst.h"
#include "xesc_yfr4_datatypes.h"

using namespace Board;
using namespace std::chrono_literals;
using namespace std;

#define MILLIS modm::Clock::now().time_since_epoch().count()

// LED sequencer
LedSeq<LedGreen> ledseq_green;
LedSeq<LedRed> ledseq_red;
#define LEDSEQ_ERROR_LL_COMM ledseq_red.blink({.on = 20, .off = 30, .limit_blink_cycles = 1, .post_pause = 0, .fulfill = true})

// Host comms (COBS)
#define COBS_BUFFER_SIZE 100                 // Should be at least size of biggest RX/TX message + some COBS overhead bytes
static uint8_t buffer_rx[COBS_BUFFER_SIZE];  // Serial RX buffer for COBS encoded data up to COBS end marker
static unsigned int idx_buffer_rx = 0;
static uint8_t buffer_tx[COBS_BUFFER_SIZE];  // Serial TX buffer for COBS encoded messages
COBS cobs;

// ADC
uint8_t adc_rr = 0;            // Analog round robin counter
uint16_t vref;                 // VRef of ADC
double cur_sense, cur_sense2;  // Current Sense ADC values (mA)

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
volatile uint32_t sa_ticks = 0;  // SA Timer Ticks between 2 SA Signals. Has to be larger than tick register due to overflow
volatile uint32_t sa_tacho = 0;  // SA Tacho ticks (1 ticks/360)

XescYFR4StatusPacket status = {};
XescYFR4SettingsPacket settings = {};
bool settings_valid = false;

void sendMessage(void *message, size_t size) {
    // Packages have to be at least 1 byte of type + 1 byte of data + 2 bytes of CRC
    if (size < 4) {
        LEDSEQ_ERROR_LL_COMM;
        return;
    }
    uint8_t *data_pointer = (uint8_t *)message;

    // Calc CRC
    uint16_t crc = CRC::Calculate(data_pointer, size - 2, CRC::CRC_16_CCITTFALSE());
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    /*
    #ifdef PROTO_DEBUG
        MODM_LOG_DEBUG << "before encoding " << size << "byte:" << modm::endl;
        uint8_t *temp = data_pointer;
        for (size_t i = 0; i < size; i++) {
            MODM_LOG_DEBUG << *temp << " ";
            temp++;
        }
        MODM_LOG_DEBUG << modm::endl;
    #endif */

    // Encode message
    size_t encoded_size = cobs.encode((uint8_t *)message, size, buffer_tx);
    buffer_tx[encoded_size] = 0;
    encoded_size++;

    /*
    #ifdef PROTO_DEBUG
        MODM_LOG_DEBUG << "encoded size " << encoded_size << "byte:" << modm::endl;
        for (size_t i = 0; i < encoded_size; i++) {
            MODM_LOG_DEBUG << buffer_tx[i] << " ";
        }
        MODM_LOG_DEBUG << modm::endl;
    #endif */

    // write() byte as long as the TX buffer isn't full
    for (size_t i = 0; i < encoded_size;)
        if (host_uart::Uart::write(buffer_tx[i])) i++;
}

void update_faults() {
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
        if (vm_switch::In::isSet()) {  // VM-Switch is "on"
            // Disable VM-Switch diagnostics when switched on,
            // because there's something miss-understood by me, or wrong with my PCB design
            // faults |= FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT;  // Not fully clear if VMS Thermal Error and/or Overload/short
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

void set_motor_state() {
    float new_duty = status.fault_code ? 0.0f : duty_setpoint;
    if (new_duty == duty) {
        return;
    }
    if (new_duty == 0.0f) {   // Stop motor
        motor::RS::set();     // !RS off
        motor::Brk::set();    // Break
    } else {                  // Start motor
        motor::Brk::reset();  // Release break
        motor::RS::reset();   // !RS on
    }
    duty = new_duty;
}

/**
 * @brief update_status() get called every STATUS_CYCLE (by hardware timer)
 */
void update_status() {
    static uint8_t motor_stopped_cycles = 0;  // No tacho change cycle counter
    uint16_t rpm;

    status.seq++;
    update_faults();

    if (!(status.fault_code & FAULT_INVALID_HALL)) {  // FAULT_INVALID_HALL = alt-use as OPEN_LOAD
        vm_switch::In::set();
    }

    set_motor_state();

    if (duty == 0.0f) {
        // Check if motor stopped rotating
        if (sa_tacho == status.tacho) {  // Tacho equals last-tacho value
            // Motor (looks like) stopped (but same tacho values might happen due to an INT/tacho error or due to slow RPM)
            if (motor_stopped_cycles > NUM_STATUS_CYCLES_MOTOR_STOPPED) {  // Check it at least NUM_STATUS_CYCLES_MOTOR_STOPPED times
                sa_ticks = 0;
                rpm = 0;
                motor::Brk::reset();  // Release break
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
    uint32_t now = MILLIS;
    MODM_LOG_INFO << "now=" << now << "ms status.fault_code=" << status.fault_code
                  << " vms_in=" << vm_switch::In::isSet()
                  << " sa_tacho=" << (uint32_t)sa_tacho
                  << " sa_ticks=" << (uint32_t)sa_ticks
                  << " rpm=" << rpm << modm::endl
                  << modm::flush;
#endif

    status.duty_cycle = duty;
    status.tacho = sa_tacho;
    status.tacho_absolute = sa_tacho;

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
            set_motor_state();
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
void handle_host_rx_buffer() {
    uint8_t data;
    while (host_uart::Uart::read(data)) {
        buffer_rx[idx_buffer_rx] = data;
        idx_buffer_rx++;
        if (idx_buffer_rx >= COBS_BUFFER_SIZE) {  // Buffer is full, but no separator. Reset
            LEDSEQ_ERROR_LL_COMM;
            idx_buffer_rx = 0;
            return;
        }

        if (data == 0) {  // COBS end marker
            /*
#ifdef PROTO_DEBUG
            MODM_LOG_DEBUG << "buffer_rx[" << idx_buffer_rx << "]:";
            for (unsigned int i = 0; i < idx_buffer_rx; ++i) {
                MODM_LOG_DEBUG << " " << buffer_rx[i];
            }
            MODM_LOG_DEBUG << modm::endl
                           << modm::flush;
#endif */
            PacketReceived();
            idx_buffer_rx = 0;
            return;
        }
    }
}

/**
 * @brief Handle ADC round-robin sampling
 *
 */
void handleAdc() {
    if ((ADC1->CR & ADC_CR_ADSTART) && !Adc1::isConversionFinished()) {  // ADC conversion already running and not finished yet
        return;
    }

    switch (adc_rr) {
        case 0:
            Adc1::disableOversampling();  // VRef sampling result in wrong result if oversampled
            vref = Adc1::readInternalVoltageReference();
#ifdef PROTO_DEBUG
            MODM_LOG_DEBUG << "adc_rr " << adc_rr << " VRef=" << vref << modm::endl;
#endif
            break;
        case 1:
            status.temperature_pcb = Adc1::readTemperature(vref);
#ifdef PROTO_DEBUG
            MODM_LOG_DEBUG << "adc_rr " << adc_rr << " Temp=" << status.temperature_pcb << modm::endl;
#endif
            break;
        case 2:
            if (Adc1::isConversionFinished()) {
                uint16_t adcValue = Adc1::getValue();
                double voltage = adcValue * vref / 4096;
#ifdef PROTO_DEBUG
                MODM_LOG_DEBUG << "adc_rr " << adc_rr << " cur_sense value=" << adcValue << " voltage=" << voltage << modm::endl;
#endif
            } else {
                Adc1::enableOversampling(Adc1::OversampleRatio::x16, Adc1::OversampleShift::Div1);
                Adc1::startConversion();
                return;
            }
            break;
        case 3:
            if (Adc1::isConversionFinished()) {
                uint16_t adcValue = Adc1::getValue();
                double voltage = adcValue * vref / 4096;
#ifdef PROTO_DEBUG
                MODM_LOG_DEBUG << "adc_rr " << adc_rr << " cur_sense_2 value=" << adcValue << " voltage=" << voltage << modm::endl;
#endif
            } else {
                Adc1::startConversion();
                return;
            }
            break;
    }

    adc_rr = (adc_rr + 1) % 4;
}

/**
 * @brief ISR execute when a Capture Compare interruption is triggered via motor::sa pin.
 */
MODM_ISR(TIM1_CC) {
    static uint16_t last_cc_value = 0;
    uint16_t temp_cc_value = Timer1::getCompareValue<motor::SA::Ch4>();
    uint32_t temp_ticks;

    Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::CaptureCompare4);
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
    Board::initialize();
    disable_nrst();  // Check NRST pin, flash if wrong and reset
    // Now thats ensured that NRST is disabled...
    vm_switch::DiagEnable::set();  // VM-Switch diagnostics enable

    /*
        // Init ADC1 with oversampling
        Adc1::connect<AdcCurSense::In2, AdcCurSense2::In1>();
        Adc1::initialize<Board::SystemClock, Adc1::ClockMode::Asynchronous, 375000>();
        Adc1::setResolution(Adc1::Resolution::Bits12);
        Adc1::setRightAdjustResult();
        Adc1::setSampleTime(Adc1::SampleTime::Cycles160_5);
        //Adc1::disableFreeRunningMode();
    */

    // Prepare status msg
    status.message_type = XESCYFR4_MSG_TYPE_STATUS;
    status.fw_version_major = 0;
    status.fw_version_minor = 2;
    status.direction = 0;  // Our motor has only one direction

    // SA (Hall) input - Capture/Compare timer
    Timer1::connect<motor::SA::Ch4>();
    Timer1::enable();
    Timer1::setMode(Timer1::Mode::UpCounter);
    Timer1::setPrescaler(SA_TIMER_PRESCALER);
    Timer1::setOverflow(0xFFFF);
    Timer1::configureInputChannel<motor::SA::Ch4>(Timer1::InputCaptureMapping::InputOwn,
                                                  Timer1::InputCapturePrescaler::Div1,  // has to match SA_TIMER_INPUT_PRESCALER
                                                  Timer1::InputCapturePolarity::Rising, 2);
    Timer1::enableInterrupt(Timer1::Interrupt::CaptureCompare4);
    Timer1::enableInterruptVector(Timer1::Interrupt::CaptureCompare4, true, 500);  // Adapt filter when changing prescaler
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

    // LED blink code "Boot up successful"
    ledseq_green.blink({.limit_blink_cycles = 3, .fulfill = true});  // Default = 200ms ON, 200ms OFF
    ledseq_red.blink({.limit_blink_cycles = 3, .fulfill = true});    // Default = 200ms ON, 200ms OFF

    while (true) {
        handle_host_rx_buffer();
        // handleAdc();
        ledseq_green.loop();
        ledseq_red.loop();
    }
}
