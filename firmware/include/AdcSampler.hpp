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
    VRef = 0,      // Internal Reference Voltage
    CurrentSense,  // Current Sense
    Temp,          // Junction Temperature
  };

  // Has to match Sensors order. FIXME: Somehow static
  static constexpr std::array<Channel, 3> sequence{
      Channel::InternalReference,
      Board::adc::CurSenseChan,
      Channel::Temperature,
  };

  static void init();  // Initialize, connect, configure and start free running ADC1
  static void disable();
  static float getValue(const Sensors sensor);  // Get (cached) value of sensor in his related unit (V, A, °C)
  static bool getCacheDirty();
  static void switchToSingleShot();  // Switch from free-running to single-shot mode
  static void triggerConversion();   // Manually trigger ADC conversion (single-shot mode)

  // Current-sense voltage (raw, as seen at the ADC pin) and its calibrated zero
  // offset. Both in volts. Needed because CurrentSense (A) scales with R_CS (R17),
  // whereas the CS pin/ADC offset is a fixed voltage. See getCsZeroVoltage().
  static float getCsVoltage();                  // Get last converted current-sense voltage
  static float getCsZeroVoltage();              // Get calibrated current-sense zero offset voltage
  static void setCsZeroVoltage(float voltage);  // Store calibrated zero offset voltage

 private:
  static std::array<uint16_t, sequence.size()> _data;        // ADC data buffer indexed in the order of sequence
  static std::array<float, sequence.size()> _cached_values;  // In their related unit (V, A, °C)
  static bool _cache_dirty;       // Indicate that the _cached_values need update because new ADC sequence started
  static float _cs_voltage;       // Last converted current-sense voltage (V)
  static float _cs_zero_voltage;  // Calibrated current-sense zero offset (V)

  static void sequence_handler();
};
