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

#pragma once

#include <cstdint>
#include "xesc_yfr4_datatypes.h"
#include "led_controller.hpp"

namespace fault_management {

    // Initialize fault management.
    void Init();

    // Update fault detection and LED status.
    // Should be called regularly (e.g., in status update cycle).
    void UpdateFaults(XescYFR4StatusPacket& status,
        const XescYFR4SettingsPacket& settings,
        bool settings_valid,
        uint32_t last_watchdog_millis,
        bool is_shutdown);

    // Get current fault code.
    uint32_t GetFaultCode();

    // Get time of last fault.
    uint32_t GetLastFaultMillis();

    // Set LED controllers for status indication.
    void SetLedControllers(LedController* status_led, LedController* error_led);
}  // namespace fault_management
