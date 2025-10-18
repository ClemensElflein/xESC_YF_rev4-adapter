// Created by Jörg Ebeling on 2025-10-18.
// Copyright (c) 2025 Jörg Ebeling for OpenMower/Clemens Elflein. All rights reserved.
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

#include "fault_management.hpp"
#include "board.hpp"
#include "config.h"
#include "hardware/hw_version.hpp"
#include <modm/platform.hpp>
#include <algorithm>

using namespace Board;

#define MILLIS modm::Clock::now().time_since_epoch().count()

namespace fault_management {

    // Internal state.
    namespace {
        uint32_t fault_code_ = 0;
        uint32_t last_fault_millis_ = 0;
        LedController* status_led_ = nullptr;
        LedController* error_led_ = nullptr;
    }  // namespace

    void Init() {
        fault_code_ = 0;
        last_fault_millis_ = 0;
    }

    void SetLedControllers(LedController* status_led, LedController* error_led) {
        status_led_ = status_led;
        error_led_ = error_led;
    }

    void UpdateFaults(XescYFR4StatusPacket& status,
        const XescYFR4SettingsPacket& settings,
        bool settings_valid,
        uint32_t last_watchdog_millis,
        bool is_shutdown) {
        uint32_t faults = 0;

        // FAULT_WRONG_HW_VERSION - Check if wrong hardware version firmware is running.
        if (!hardware::checkFlashMatchesBinary()) {
            faults |= FAULT_WRONG_HW_VERSION;
        }

        // FAULT_UNINITIALIZED.
        if (!settings_valid) {
            faults |= FAULT_UNINITIALIZED;
        }

        // FAULT_WATCHDOG.
        if (MILLIS - last_watchdog_millis > WATCHDOG_TIMEOUT_MILLIS) {
            faults |= FAULT_WATCHDOG;
        }

        // FAULT_OVERCURRENT.
        if (status.current_input > static_cast<double>(HW_LIMIT_CURRENT)) {
            faults |= FAULT_OVERCURRENT;
        }

        // FAULT_OVERTEMP_PCB.
        if (status.temperature_pcb >
            static_cast<double>(std::min(HW_LIMIT_PCB_TEMP, settings.max_pcb_temp))) {
            faults |= FAULT_OVERTEMP_PCB;
        }

#ifdef HW_V1
        // VMS FAULTs (V1 hardware only).
        if (!vm_switch::Fault::read()) {
            if (vm_switch::In::isSet()) {
                // VM-Switch is "on"
                // Disable VM-Switch diagnostics when switched on.
                // TODO: Review if this is correct behavior.
            } else if (!host::Shutdown::read()) {
                faults |= FAULT_OPEN_LOAD;
            }
        }
#endif

        if (faults) {
            status.fault_code = faults;
            fault_code_ = faults;
            last_fault_millis_ = MILLIS;

            // Update LED indicators based on fault priority.
            if (status_led_) {
                if (faults & FAULT_UNINITIALIZED) {
                    status_led_->Blink();  // 1Hz blink
                } else {
                    status_led_->Off();
                }
            }

            if (error_led_) {
                if (faults & FAULT_WRONG_HW_VERSION) {
                    error_led_->On();  // Steady on (clear indication)
                } else if (faults & FAULT_OPEN_LOAD) {
                    error_led_->Blink(125U, 125U, 0);  // Quick blink (4Hz)
                } else if (faults & (FAULT_OVERTEMP_PCB | FAULT_OVERCURRENT)) {
                    error_led_->Blink(250U, 250U, 0);  // Fast blink (2Hz)
                } else if (faults & FAULT_WATCHDOG) {
                    error_led_->Blink(0, false);  // 1Hz blink
                }
            }
        } else if (faults == 0) {
            // No faults - update status LED.
            if (status_led_) {
                if (is_shutdown) {
                    status_led_->Blink(100U, 1800U, 1);  // Short pulse
                } else {
                    status_led_->On();
                }
            }

            if (status.fault_code != 0) {
                // Reset faults only if MIN_FAULT_TIME_MILLIS passed,
                // or if it was a dedicated watchdog fault.
                if (MILLIS - last_fault_millis_ > MIN_FAULT_TIME_MILLIS ||
                    status.fault_code == FAULT_WATCHDOG) {
                    if (error_led_) {
                        error_led_->Off();
                    }
                    status.fault_code = 0;
                    fault_code_ = 0;
                }
            }
        }
    }

    uint32_t GetFaultCode() {
        return fault_code_;
    }

    uint32_t GetLastFaultMillis() {
        return last_fault_millis_;
    }

}  // namespace fault_management
