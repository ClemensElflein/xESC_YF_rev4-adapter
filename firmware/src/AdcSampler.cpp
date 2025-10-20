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

#include "AdcSampler.hpp"

using namespace modm::literals;

// ADC data buffer (required by AdcSampler).
std::array<uint16_t, AdcSampler::sequence.size()> AdcSampler::_data = {};
std::array<float, AdcSampler::sequence.size()> AdcSampler::_cached_values = {};

/**
 * @brief Initialize, connect, configure and start free running ADC1
 *
 * Target: ~10Hz update rate with best possible accuracy
 * - Sample Time: 160.5 cycles (maximum for STM32C011)
 * - Oversampling: 64x with Div4 (Shift=2) → 14-bit result for good SNR
 * - ADC Clock: 750 kHz
 *
 * Update rate calculation:
 * 3 channels × (160.5 + 12.5) cycles × 64 oversamples / 750 kHz ≈ 14 ms per sequencer cycle (~71 Hz)
 */
void AdcSampler::init() {
#ifdef PROTO_DEBUG_ADC
  MODM_LOG_INFO << "AdcSampler::init" << modm::endl << modm::flush;
#endif
  // Initialize ADC with optimal accuracy settings
  initialize<Board::SystemClock, ClockMode::Asynchronous, 750_kHz>();
  connect<Board::adc::CurSenseAdc>();      // Hardware version independent (defined in board.hpp)
  setSampleTime(SampleTime::Cycles160_5);  // Maximum sample time for best accuracy
  setResolution(ADC_RESOLUTION);
  setRightAdjustResult();
  // 64x oversampling with Div4 (2-bit shift) → 14-bit effective (2 extra bits)
  enableOversampling(OversampleRatio::x64, OversampleShift::Div4);

  setChannels(sequence);
  enableInterruptVector(15);
  enableInterrupt(Interrupt::EndOfConversion);
  AdcInterrupt1::attachInterruptHandler(sequence_handler);
  enableFreeRunningMode();
  startConversion();
}

void AdcSampler::disable() {
  disableInterrupt(Interrupt::EndOfConversion);
  disableFreeRunningMode();
  stopConversion();
}

/**
 * @brief Get internal measured junction temp
 *
 * @return uint16_t
 */
// uint16_t AdcSampler::getInternalTemp() {
//   const int32_t value = ((_data.at(Sensors::Temp) * getInternalVref_u() / VDDA_CAL) - int32_t(*TS_CAL1)) * 1000;
//   return (value / TS_AVG_SLOPE) + TS_CAL1_TEMP;
// }

/**
 * @brief Get the (cached) value of the given sensor in his related unit (V, A, °C)
 *
 * @param sensor
 * @return float
 */
float AdcSampler::getValue(const Sensors sensor) {
  return _cached_values[sensor];
}

void AdcSampler::sequence_handler() {
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
      if (seq_idx < sequence.size()) {                          // Ensure array bounds
        _data.at(seq_idx++) = Adc1::getValue();
      }
    }
  }

  if (flag & InterruptFlag::EndOfSequence) {
    acknowledgeInterruptFlags(InterruptFlag::EndOfSequence |
                              InterruptFlag::Overrun);  // Also (and earliest) ack a possible overrun
    seq_idx = 0;

    // Update cached values after each ADC sequence
    uint32_t vref_int = (VDDA_CAL * uint32_t(*VREFINT_CAL)) / _data.at(Sensors::VRef);

    _cached_values[Sensors::VRef] = (static_cast<uint32_t>(VDDA_CAL) << 2) *
                                    (static_cast<uint32_t>(*VREFINT_CAL) << 2) / _data.at(Sensors::VRef) / 1000.0f;

    // V_CS
    // Skipped expensive float calculation in favor of integer math
    //_cached_values[Sensors::CurrentSense] = _cached_values[Sensors::VRef] * _data.at(Sensors::CurrentSense) /
    //                                        (static_cast<uint32_t>(ADC_NUM_CODES) << 4);  // Why <<4 and not <<2?!
    _cached_values[Sensors::CurrentSense] = float(vref_int * _data.at(Sensors::CurrentSense) / ADC_NUM_CODES) / 1000.0f;

    // I_CS see board.hpp for constexpr
#ifdef HW_V1
    // HW_V1: INA139NA current sense amplifier
    // I = V_adc / (Gain × R_shunt)
    _cached_values[Sensors::CurrentSense] /= Board::adc::CurSenseDivisor;
#else
    // HW_V2: TPS1H100B with current mirror
    // I_out = V_CS × (K / R_CS)
    _cached_values[Sensors::CurrentSense] *= Board::adc::CurSenseKDivRCS;
#endif

    // Junction temperature in °C see reference manual for formula
    _cached_values[Sensors::Temp] =
        ((((_data.at(Sensors::Temp) * vref_int / VDDA_CAL) - int32_t(*TS_CAL1)) * 1000) / TS_AVG_SLOPE) + TS_CAL1_TEMP;
  }
}
