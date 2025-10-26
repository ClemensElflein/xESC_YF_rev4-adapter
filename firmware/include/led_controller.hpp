// Created by Jörg Ebeling on 2025-10-16.
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

/**
 * @brief A simple (and light) LED-Controller for blink sequences
 *
 */

#pragma once

#include <modm/platform.hpp>
#include <cstdint>
#include "gpio_interface.hpp"

class LedController {
public:
    enum class Mode : uint8_t {
        Off = 0,
        On = 1,
        Blink = 2
    };

    enum class State : uint8_t {
        Undefined = 0,
        Idle = 1,
        On = 2,
        Off = 3,
        NextBlink = 4,
        PostPause = 5
    };

    struct Properties {
        Mode mode = Mode::Blink;
        uint32_t on_time_ms = 500;
        uint32_t off_time_ms = 500;
        uint8_t blink_cycle_limit = 0;
        uint32_t post_pause_ms = 500;
        bool complete = false;

        bool operator==(const Properties& other) const {
            return mode == other.mode &&
                on_time_ms == other.on_time_ms &&
                off_time_ms == other.off_time_ms &&
                blink_cycle_limit == other.blink_cycle_limit &&
                post_pause_ms == other.post_pause_ms &&
                complete == other.complete;  // Complete before next sequence
        }
    };

    // Constructor with GPIO interface - simple and clean.
    explicit LedController(IGpio* gpio) : gpio_(gpio) {}

    // Delete copy operations.
    LedController(const LedController&) = delete;
    LedController& operator=(const LedController&) = delete;

    // Simple API.
    void On() {
        QueueSequence({ .mode = Mode::On, .post_pause_ms = 0 });
    }

    void Off() {
        QueueSequence({ .mode = Mode::Off, .post_pause_ms = 0 });
    }

    // Blink with custom properties.
    void Blink(const Properties& properties) {
        Properties props = properties;
        props.mode = Mode::Blink;
        QueueSequence(props);
    }

    // Blink (1Hz) with default properties.
    void Blink(unsigned int cycles = 0, bool complete = false) {
        Properties props;
        props.mode = Mode::Blink;
        props.blink_cycle_limit = cycles;
        props.complete = complete;
        QueueSequence(props);
    }

    // Quick blink (<2Hz) with default properties.
    void QuickBlink(unsigned int cycles = 0, bool complete = false) {
        Properties props;
        props.mode = Mode::Blink;
        props.on_time_ms = 200;
        props.off_time_ms = 200;
        props.blink_cycle_limit = cycles;
        props.complete = complete;
        QueueSequence(props);
    }

    // Convenience method for simple blinking.
    void Blink(uint32_t on_time_ms, uint32_t off_time_ms, uint8_t cycles = 0) {
        QueueSequence({ .mode = Mode::Blink,
                       .on_time_ms = on_time_ms,
                       .off_time_ms = off_time_ms,
                       .blink_cycle_limit = cycles,
                       .post_pause_ms = 500 });
    }

    bool Update() {
        if (active_state_.state == State::Undefined &&
            queued_state_.state == State::Undefined) {
            return false;
        }

        const uint32_t now = modm::Clock::now().time_since_epoch().count();

        // Promote queued to active if ready.
        if (queued_state_.state == State::Idle &&
            (active_state_.state == State::Undefined ||
                (!active_state_.properties.complete &&
                    !(queued_state_.properties == active_state_.properties)))) {
            active_state_ = queued_state_;
            queued_state_.state = State::Undefined;
            active_state_.next_state_time_ms = now;
        }

        if (now < active_state_.next_state_time_ms) {
            return true;
        }

        ProcessState(now);
        return true;
    }

    bool IsActive() const {
        return active_state_.state != State::Undefined ||
            queued_state_.state != State::Undefined;
    }

    void Stop() {
        active_state_.state = State::Undefined;
        queued_state_.state = State::Undefined;
        if (gpio_) gpio_->Reset();
    }

    Properties GetActiveProperties() const {
        return active_state_.properties;
    }

    State GetCurrentState() const {
        return active_state_.state;
    }

    // Change GPIO at runtime.
    void SetGpio(IGpio* new_gpio) {
        gpio_ = new_gpio;
    }

private:
    struct InternalState {
        Properties properties;
        State state = State::Undefined;
        uint32_t next_state_time_ms = 0;
        uint8_t completed_cycles = 0;
    };

    IGpio* gpio_;
    InternalState active_state_;
    InternalState queued_state_;

    void QueueSequence(const Properties& properties) {
        queued_state_.properties = properties;
        queued_state_.state = State::Idle;
        queued_state_.completed_cycles = 0;
        Update();
    }

    void ProcessState(uint32_t current_time_ms) {
        if (!gpio_) return;

        switch (active_state_.properties.mode) {
        case Mode::On:
            gpio_->Set();
            active_state_.state = State::Undefined;
            break;

        case Mode::Off:
            gpio_->Reset();
            active_state_.state = State::Undefined;
            break;

        case Mode::Blink:
            ProcessBlinkState(current_time_ms);
            break;
        }
    }

    void ProcessBlinkState(uint32_t current_time_ms) {
        if (!gpio_) return;

        switch (active_state_.state) {
        case State::Idle:
            active_state_.completed_cycles = 0;
            [[fallthrough]];

        case State::NextBlink:
            gpio_->Set();
            active_state_.next_state_time_ms =
                current_time_ms + active_state_.properties.on_time_ms;
            active_state_.state = State::On;
            break;

        case State::On:
            gpio_->Reset();
            active_state_.next_state_time_ms =
                current_time_ms + active_state_.properties.off_time_ms;
            active_state_.state = State::Off;
            break;

        case State::Off:
            active_state_.completed_cycles++;

            if (active_state_.properties.blink_cycle_limit == 0 ||
                active_state_.completed_cycles <
                active_state_.properties.blink_cycle_limit) {
                active_state_.state = State::NextBlink;
                Update();
                break;
            }

            active_state_.next_state_time_ms =
                current_time_ms + active_state_.properties.post_pause_ms;
            active_state_.state = State::PostPause;
            break;

        case State::PostPause:
            active_state_.state = State::Undefined;
            break;

        default:
            break;
        }
    }
};
