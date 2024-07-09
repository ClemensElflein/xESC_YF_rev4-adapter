// Created by Jörg Ebeling on 2024-07-07.
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

#include "LED.h"

LED::LED(uint32_t pin) {
    _pin = pin;

    pinMode(_pin, OUTPUT);
}

/**
 * @brief LED blink sequence
 * Always get stored as _next sequence, probably overwriting an already existing (but not started) _next sequence.
 *
 * @param props
 */
void LED::blink(LedProps props) {
    _next_state.props = props;
    _next_state.props.mode = LED_MODE_BLINK;
    _next_state.state = LED_STATE_IDLE;
    loop();
}

void LED::on() {
    _next_state.props = {.mode = LED_MODE_ON, .post_pause = 0};
    _next_state.state = LED_STATE_IDLE;
    loop();
}

void LED::off() {
    _next_state.props = {.mode = LED_MODE_OFF, .post_pause = 0};
    _next_state.state = LED_STATE_IDLE;
    loop();
}

void LED::loop(void) {
    if (_active_state.state == LED_STATE_UNDEF && _next_state.state == LED_STATE_UNDEF)  // Nothing to do
        return;

    uint32_t now = millis();

    // Make _next _active when
    if (_next_state.state == LED_STATE_IDLE &&                                              // _next job exists
        (_active_state.state == LED_STATE_UNDEF ||                                          // _active finished
         (!_active_state.props.fulfill && !(_next_state.props == _active_state.props)))) {  // _active is not fulfillable and _next differs to _active
        _active_state = _next_state;
        _next_state.state = LED_STATE_UNDEF;
        _active_state.next_state_cycle_millis = now;  // Just activated = next cycle = now
    }

    if (now < _active_state.next_state_cycle_millis)
        return;

    switch (_active_state.props.mode) {
        case LED_MODE_ON:
            digitalWrite(_pin, HIGH);  // On
            _active_state.state = LED_STATE_UNDEF;
            break;

        case LED_MODE_OFF:
            digitalWrite(_pin, LOW);  // Off
            _active_state.state = LED_STATE_UNDEF;
            break;

        case LED_MODE_BLINK:
            switch (_active_state.state) {
                case LED_STATE_IDLE:
                    _active_state.blink_cycles = 0;
                case LED_STATE_NEXT_BLINK:
                    digitalWrite(_pin, HIGH);  // On
                    _active_state.next_state_cycle_millis = now + _active_state.props.on;
                    _active_state.state = LED_STATE_ON;
                    break;
                case LED_STATE_ON:
                    digitalWrite(_pin, LOW);  // Off
                    _active_state.next_state_cycle_millis = now + _active_state.props.off;
                    _active_state.state = LED_STATE_OFF;
                    break;
                case LED_STATE_OFF:  // One complete on/off cycle fulfilled
                    _active_state.blink_cycles++;
                    if (!_active_state.props.limit_blink_cycles || _active_state.blink_cycles < _active_state.props.limit_blink_cycles) {
                        _active_state.state = LED_STATE_NEXT_BLINK;
                        loop();
                        break;
                    }
                    _active_state.next_state_cycle_millis = now + _active_state.props.post_pause;
                    _active_state.state = LED_STATE_POST_PAUSE;
                    break;
                case LED_STATE_POST_PAUSE:  // Full sequence done
                    _active_state.state = LED_STATE_UNDEF;
                    break;
            }
    }
}
