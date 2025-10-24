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
bool AdcSampler::_cache_dirty = false;

/**
 * @brief Initialize, connect, configure and start free running ADC1
 *
 * Target: Complete within 20ms status cycle with optimized accuracy
 * - Sample Time: 160.5 cycles (max. for STM32C011)
 * - Oversampling: 32x with Div2 (Shift=1) → 16-bit effective result
 * - ADC Clock: 1.5 MHz
 *
 * Update rate calculation:
 * 3 channels × (160.5 + 12.5) cycles × 32 oversamples / 1.5 MHz = 11.1 ms per sequencer cycle
 * This leaves ample time for floating-point calculations within the 20ms cycle
 */
void AdcSampler::init() {
#ifdef PROTO_DEBUG_ADC
  MODM_LOG_INFO << "AdcSampler::init" << modm::endl << modm::flush;
#endif
  initialize<Board::SystemClock, ClockMode::Asynchronous, 1.5_MHz>();
  connect<Board::adc::CurSenseAdc>();      // Hardware version independent (defined in board.hpp)
  setSampleTime(SampleTime::Cycles160_5);  // Good balance of accuracy and speed
  setResolution(ADC_RESOLUTION);
  setRightAdjustResult();
  enableOversampling(OversampleRatio::x32, OversampleShift::Div2);

  setChannels(sequence);
  enableInterruptVector(15);
  enableInterrupt(Interrupt::EndOfConversion);
  AdcInterrupt1::attachInterruptHandler(sequence_handler);
  enableFreeRunningMode();
  _cache_dirty = true;
  startConversion();
}

void AdcSampler::disable() {
  disableInterrupt(Interrupt::EndOfConversion);
  disableFreeRunningMode();
  stopConversion();
}

bool AdcSampler::getCacheDirty() {
  return _cache_dirty;
}

/**
 * @brief Get the (cached) value of the given sensor in his related unit (V, A, °C)
 *
 * @param sensor
 * @return float
 */
float AdcSampler::getValue(const Sensors sensor) {
  return _cached_values[sensor];
}

void AdcSampler::switchToSingleShot() {
  disableFreeRunningMode();
  stopConversion();
}

void AdcSampler::triggerConversion() {
  _cache_dirty = true;
  startConversion();
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

    // Update cached values after each ADC sequence.
    // The expensive float calcs could be moved to outside of the INT routine if needed
    // due to timing constraints, but for now it also fits to here (20ms cycle)

    // Some oversampled constants
    constexpr uint32_t vdda_cal_ovs = VDDA_CAL << 2;
    const uint32_t vrefint_cal_ovs = *VREFINT_CAL << 2;

    // V_REF
    uint32_t vref_mv = (vdda_cal_ovs * vrefint_cal_ovs) / _data.at(Sensors::VRef);
    _cached_values[Sensors::VRef] = vref_mv / 1000.0f;

    // V_CS
    float v_cs = vref_mv * _data.at(Sensors::CurrentSense) / (ADC_NUM_CODES << 4) / 1000.0f;

    // I_CS
    _cached_values[Sensors::CurrentSense] = Board::adc::CalculateCurrent(v_cs);

    // Junction temperature in °C
    uint32_t sense_data = static_cast<uint32_t>(_data.at(Sensors::Temp)) * vref_mv / VDDA_CAL;
    _cached_values[Sensors::Temp] = ((sense_data - *TS_CAL1) / TS_AVG_SLOPE) + TS_CAL1_TEMP;

    _cache_dirty = false;
  }
}
