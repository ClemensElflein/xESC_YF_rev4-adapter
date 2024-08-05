// Created by Jörg Ebeling on 2024-07-18.
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
 * @brief A simple (and light) LED-Sequencer for blink sequences
 *
 */

#pragma once

#include <modm/platform.hpp>

#define LEDSEQ_MODE_OFF 0
#define LEDSEQ_MODE_ON 1
#define LEDSEQ_MODE_BLINK 2

#define LEDSEQ_STATE_UNDEF 0
#define LEDSEQ_STATE_IDLE 1
#define LEDSEQ_STATE_ON 2
#define LEDSEQ_STATE_OFF 3
#define LEDSEQ_STATE_NEXT_BLINK 4
#define LEDSEQ_STATE_POST_PAUSE 5

template <class GpioStatic>
class LedSeq {
    //  LED blink properties with mostly used blink defaults
    struct LedProps {
        uint8_t mode = LEDSEQ_MODE_BLINK;  // Mode of operation
        uint32_t on = 200;                 // On time in ms (only relevant for blink mode)
        uint32_t off = 200;                // Off time in ms (only relevant for blink mode)
        uint8_t limit_blink_cycles = 0;    // Limit blink cycles (only relevant for blink mode)
        uint32_t post_pause = 500;         // Pause after blink sequence (only relevant for blink mode)
        bool fulfill = false;              // Fulfill the complete blink sequence before starting the next one

        bool operator==(const LedProps& rhs) const {
            return std::tie(mode, on, off, limit_blink_cycles, post_pause, fulfill) == std::tie(rhs.mode, rhs.on, rhs.off, rhs.limit_blink_cycles, rhs.post_pause, rhs.fulfill);
        }
    };

   private:
    struct LedState {
        LedProps props;
        uint8_t state = LEDSEQ_STATE_UNDEF;  // Current state in sequence (idle, on, off, post_pause, ...)
        uint32_t next_state_cycle_millis;    // Millis when next state-cycle shall happen
        uint8_t blink_cycles;                // Counter for completed blink cycles
    };

    GpioStatic _gpio;
    LedState _active_state, _next_state;

    void _next(LedProps props) {
        _next_state.props = props;
        _next_state.state = LEDSEQ_STATE_IDLE;
        loop();
    }

   public:
    /**
     * @brief Shorthand for LED-On (once active sequence is done (or !fulfill))
     */
    void on(void) {
        _next({.mode = LEDSEQ_MODE_ON, .post_pause = 0});
    }

    /**
     * @brief Shorthand for LED-Off (once active sequence is done (or !fulfill))
     */
    void off(void) {
        _next({.mode = LEDSEQ_MODE_OFF, .post_pause = 0});
    }

    /**
     * @brief LED blink sequence with the given blink properties.
     * Get "queued" as _next sequence, probably overwriting an already existing (but yet not started) _next sequence.
     *
     * @param props
     */
    void blink(LedProps props) {
        props.mode = LEDSEQ_MODE_BLINK;
        _next(props);
    }

    /**
     * @brief Generic loop() function for sequence processing.
     * Call as often as possible and as less as required (for a visual LED feedback).
     *
     * @return true if sequence got processed
     * @return false if all sequences got processed
     */
    bool loop(void) {
        if (_active_state.state == LEDSEQ_STATE_UNDEF && _next_state.state == LEDSEQ_STATE_UNDEF)  // Nothing to do
            return false;

        uint32_t now = modm::Clock::now().time_since_epoch().count();

        // Make _next _active when
        if (_next_state.state == LEDSEQ_STATE_IDLE &&                                           // _next job exists
            (_active_state.state == LEDSEQ_STATE_UNDEF ||                                       // _active finished
             (!_active_state.props.fulfill && !(_next_state.props == _active_state.props)))) {  // _active is not fulfillable and _next differs to _active
            _active_state = _next_state;
            _next_state.state = LEDSEQ_STATE_UNDEF;
            _active_state.next_state_cycle_millis = now;  // Just activated = next cycle = now
        }

        if (now < _active_state.next_state_cycle_millis)
            return true;

        switch (_active_state.props.mode) {
            case LEDSEQ_MODE_ON:
                _gpio.set();  // On
                // digitalWrite(_pin, HIGH);  // On
                _active_state.state = LEDSEQ_STATE_UNDEF;
                break;

            case LEDSEQ_MODE_OFF:
                _gpio.reset();  // Off
                // digitalWrite(_pin, LOW);  // Off
                _active_state.state = LEDSEQ_STATE_UNDEF;
                break;

            case LEDSEQ_MODE_BLINK:
                switch (_active_state.state) {
                    case LEDSEQ_STATE_IDLE:
                        _active_state.blink_cycles = 0;
                        /* FALLTHRU */
                    case LEDSEQ_STATE_NEXT_BLINK:
                        _gpio.set();  // On
                        // digitalWrite(_pin, HIGH);  // On
                        _active_state.next_state_cycle_millis = now + _active_state.props.on;
                        _active_state.state = LEDSEQ_STATE_ON;
                        break;
                    case LEDSEQ_STATE_ON:
                        _gpio.reset();  // Off
                        // digitalWrite(_pin, LOW);  // Off
                        _active_state.next_state_cycle_millis = now + _active_state.props.off;
                        _active_state.state = LEDSEQ_STATE_OFF;
                        break;
                    case LEDSEQ_STATE_OFF:  // One complete on/off cycle fulfilled
                        _active_state.blink_cycles++;
                        if (!_active_state.props.limit_blink_cycles || _active_state.blink_cycles < _active_state.props.limit_blink_cycles) {
                            _active_state.state = LEDSEQ_STATE_NEXT_BLINK;
                            loop();
                            break;
                        }
                        _active_state.next_state_cycle_millis = now + _active_state.props.post_pause;
                        _active_state.state = LEDSEQ_STATE_POST_PAUSE;
                        break;
                    case LEDSEQ_STATE_POST_PAUSE:  // Full sequence done
                        _active_state.state = LEDSEQ_STATE_UNDEF;
                        break;
                }
        }
        return true;
    }
};
