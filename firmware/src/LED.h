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

#ifndef _LED_H_
#define _LED_H_

#include <Arduino.h>

#define LED_MODE_OFF 0
#define LED_MODE_ON 1
#define LED_MODE_BLINK 2

#define LED_STATE_UNDEF 0
#define LED_STATE_IDLE 1
#define LED_STATE_ON 2
#define LED_STATE_OFF 3
#define LED_STATE_NEXT_BLINK 4
#define LED_STATE_POST_PAUSE 5

class LED {
    // LED blink properties with mostly used blink defaults
    struct LedProps {
        uint8_t mode = LED_MODE_BLINK;   // Mode of operation
        uint32_t on = 200;               // On time in ms (only relevant for blink mode)
        uint32_t off = 200;              // Off time in ms (only relevant for blink mode)
        uint8_t limit_blink_cycles = 0;  // Limit blink cycles (only relevant for blink mode)
        uint32_t post_pause = 500;       // Pause after blink sequence (only relevant for blink mode)
        bool fulfill = false;            // Fulfill the complete blink sequence before starting the next one

        bool operator==(const LedProps& rhs) const {
            return std::tie(mode, on, off, limit_blink_cycles, post_pause, fulfill) == std::tie(rhs.mode, rhs.on, rhs.off, rhs.limit_blink_cycles, rhs.post_pause, rhs.fulfill);
        }
    };

   private:
    struct LedState {
        LedProps props;
        uint8_t state = LED_STATE_UNDEF;   // Current state in sequence (idle, on, off, post_pause, ...)
        uint32_t next_state_cycle_millis;  // Millis when next state-cycle shall happen
        uint8_t blink_cycles;              // Counter for completed blink cycles
    };

    uint32_t _pin;
    LedState _active_state, _next_state;

   public:
    LED(uint32_t pin);

    void on(void);
    void off(void);
    void blink(LedProps props);

    void loop(void);
};

#endif
