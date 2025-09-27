// Created by Jörg Ebeling on 2024-08-13.
// Copyright (c) 2024 Jörg Ebeling for OpenMower/Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons
// Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't
// try to sell the design or products based on it without getting my consent
// first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @brief A simple and static Adc-Sampler class which handles all ADC-Sensors
 */

#pragma once

#include <modm/platform.hpp>

#include "board.hpp"

using namespace modm::literals;

class AdcSampler : public modm::platform::Adc1 {
public:
    enum Sensors : uint8_t {
        VRef = 0,
        CurSense,   // = In2
        Temp,
    };

    // Has to match Sensors order. FIXME: Somehow static
    static constexpr std::array<Channel, 3> sequence{
        Channel::InternalReference,
        Board::CurSenseChan,
        Channel::Temperature,
    };

    /**
     * @brief Initialize, connect, configure and start free running ADC1
     *
     */
    static void init() {
#ifdef PROTO_DEBUG_ADC
        MODM_LOG_INFO << "AdcSampler::init" << modm::endl
            << modm::flush;
#endif
        // Target frequency consideration:
        // Let's get: 4 Channels * 160.5 sampling rate * 32 Oversamples every about 20ms (50Hz) = 1.03 MHz
        initialize<Board::SystemClock, ClockMode::Asynchronous, 750_kHz>();  // Frequency is slower but more accurate
        //connect<Board::AdcCurSense::In2>();
#ifdef HW_V1
    connect<Board::CurSense::In2>();
#else
    connect<Board::CurSense::In3>();
#endif
        setSampleTime(SampleTime::Cycles160_5);
        setResolution(ADC_RESOLUTION);
        setRightAdjustResult();
        enableOversampling(OversampleRatio::x32, OversampleShift::Div32);
        setChannels(sequence);
        enableInterruptVector(15);
        enableInterrupt(Interrupt::EndOfConversion);
        AdcInterrupt1::attachInterruptHandler(sequence_handler);
        enableFreeRunningMode();
        startConversion();
    };

    static void disable() {
        disableInterrupt(Interrupt::EndOfConversion);
        disableFreeRunningMode();
        stopConversion();
    }

    /**
     * @brief Get internal measured VRef
     *
     * @return float
     */
    static float getInternalVref_f() {
        return (float)(VDDA_CAL * uint32_t(*VREFINT_CAL)) / _data.at(Sensors::VRef) / 1000.0f;
    }

    /**
     * @brief Get internal measured VRef
     *
     * @return uint16_t
     */
    static uint16_t getInternalVref_u() {
        return (VDDA_CAL * uint32_t(*VREFINT_CAL)) / _data.at(Sensors::VRef);
    }

    /**
     * @brief Get internal measured junction temp
     *
     * @return uint16_t
     */
    static uint16_t getInternalTemp() {
        const int32_t value = ((_data.at(Sensors::Temp) * getInternalVref_u() / VDDA_CAL) - int32_t(*TS_CAL1)) * 1000;
        return (value / TS_AVG_SLOPE) + TS_CAL1_TEMP;
    }

    /**
     * @brief Get the voltage of the given ADC sensor.
     * Attention: For the internal VRef and Temp sensors, see getInternal...() functions.
     *
     * @param sensor
     * @return float
     */
    static float getVoltage(const Sensors sensor) {
        return (getInternalVref_f() * _data.at(sensor)) / ADC_NUM_CODES;
    }

private:
    static std::array<uint16_t, sequence.size()> _data;  // ADC data buffer indexed in the order of sequence

    static void sequence_handler() {
        static uint8_t seq_idx = 0;
        auto flag = getInterruptFlags();

        /*// Debug ADC IRQ Flags
        if (flag & InterruptFlag::EndOfConversion)
            MODM_LOG_DEBUG << "C";

        if (flag & InterruptFlag::EndOfSequence)
            MODM_LOG_DEBUG << "S";

        if (flag & InterruptFlag::Overrun)
            MODM_LOG_DEBUG << "O";

        MODM_LOG_DEBUG << ", ";*/

        if (flag & InterruptFlag::EndOfConversion) {
            acknowledgeInterruptFlags(InterruptFlag::EndOfConversion);  // Always ack EOC
            if (!(flag & InterruptFlag::Overrun)) {                     // In the case of an OVR, we can't trust our seq_idx
                if (seq_idx < sequence.size()) {                        // Ensure array bounds
                    _data.at(seq_idx) = Adc1::getValue();
                    seq_idx++;
                }
            }
        }

        if (flag & InterruptFlag::EndOfSequence) {
            acknowledgeInterruptFlags(InterruptFlag::EndOfSequence | InterruptFlag::Overrun);  // Also (and earliest) ack a possible overrun
            seq_idx = 0;
        }
    }
};
