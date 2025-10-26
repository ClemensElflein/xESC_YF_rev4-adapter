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

namespace motor_control {

// Initialize SA timer
void Init();

void SetDutySetpoint(float duty);  // Set motor duty cycle setpoint (-1.0 to 1.0)
float GetDuty();                   // Get current motor duty cycle

// Update motor state based on faults, shutdown signal and current tacho.
// Should be called regularly (e.g., in status update cycle).
void UpdateMotorState(bool has_faults, bool is_shutdown, uint32_t last_tacho);

uint32_t GetSaTacho();  // Get SA (Hall sensor) tacho value
uint32_t GetSaTicks();  // Get SA timer ticks between signals
uint16_t GetRpm();      // Get RPM

// Check if motor is stopped and update internal counters.
// Returns true if motor has been stopped for enough cycles.
bool UpdateMotorStoppedState(uint32_t current_tacho);
bool hasMotorSafelyStopped();
}  // namespace motor_control
