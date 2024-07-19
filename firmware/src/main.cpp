// Created by Apehaenger on 2024-06-14.
//
// This file is part of the openmower project.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <modm/debug/logger.hpp>
#include <modm/platform.hpp>
#include <modm/ui/animation.hpp>
#include <modm/ui/led.hpp>

#include "LedSeq.hpp"
#include "board.hpp"

using namespace Board;
using namespace std::chrono_literals;

// UART print debugging via proto_uart
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG
// Create an IODeviceWrapper around the Uart Peripheral we want to use
modm::IODeviceWrapper<proto_uart::Uart, modm::IOBuffer::BlockIfFull> loggerDevice;
// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

// modm::ui::Led red([](uint8_t brightness) {
// Timer3::setCompareValue<Board::LedRd::Ch2>(modm::ui::table22_16_256[brightness]);
//});

// apply some animations to the leds
// modm::ui::Pulse<uint8_t> pulse(red);

// animate the period of the red pulse (Aniception?)
// static uint16_t period = 500;
// modm::ui::Animation<uint16_t> periodAnimator(period, [](uint16_t period)
//{
// pulse.setPeriod(period);
//});
// wrap it in a pulse
// modm::ui::Pulse<uint16_t> pulsePeriod(periodAnimator);

LedSeq<LedGreen> ledseq_green;
LedSeq<LedRed> ledseq_red;

MODM_ISR(TIM14) {
    Timer14::acknowledgeInterruptFlags(Timer14::InterruptFlag::Update);
    //ta.inc();
    MODM_LOG_DEBUG << "Toggle green LED " << modm::endl;
    //LedGreen::toggle();
    //ledseq_green.test();
}

MODM_ISR(TIM16) {
    Timer16::acknowledgeInterruptFlags(Timer16::InterruptFlag::Update);
    MODM_LOG_DEBUG << "Toggle red LED " << modm::endl;
    // LedRd::toggle();
    //ledseq_red.test();
    //tb.inc();
}

int main() {
    Board::initialize();

    // Initialize prototyping-UART for MODM_LOG_*
    proto_uart::Uart::connect<proto_uart::Tx::Tx, proto_uart::Rx::Rx>();
    proto_uart::Uart::initialize<SystemClock, 460800_Bd>();

    MODM_LOG_INFO << modm::endl
                  << modm::endl
                  << "Board initialized" << modm::endl;

    // connect the Timer Channels to the LEDs
    // Timer3::connect<Board::LedRd::Ch2>();

    // set up the timer for 16bit PWM
    Timer3::enable();
    Timer3::setMode(Timer3::Mode::UpCounter);

    // 42 MHz / 1 / 2^16 ~ 640 Hz refresh rate
    Timer3::setPrescaler(1);
    Timer3::setOverflow(65535);
    // configure the output channels
    // Timer3::configureOutputChannel<Board::LedRd::Ch2>(Timer3::OutputCompareMode::Pwm, 0);
    Timer3::applyAndReset();
    // start the timer
    // Timer3::start();

    Timer14::enable();
    Timer14::setMode(Timer14::Mode::UpCounter);
    Timer14::setPeriod<Board::SystemClock>(2000ms);
    Timer14::applyAndReset();
    Timer14::start();
    Timer14::enableInterrupt(Timer14::Interrupt::Update);
    Timer14::enableInterruptVector(true, 5);

    // set the animation mode for autoreverse the keyframes
    // keyFrames.setMode(modm::ui::KeyFrameAnimationMode::Autoreverse);
    // set the indicator period change to 15s
    // pulsePeriod.setPeriod(10000);
    // pulse between 0.5s and 5s.
    // pulsePeriod.setRange(500, 5000);
    // indicator.setRange(0, 100);

    // start all animations indefinitely
    // pulse.start();
    // indicator.start();
    // strobe.start();
    // keyFrames.start();
    // pulsePeriod.start();

    Timer16::enable();
    Timer16::setMode(Timer16::Mode::UpCounter);
    Timer16::setPeriod<Board::SystemClock>(250ms);
    Timer16::applyAndReset();
    Timer16::start();
    Timer16::enableInterrupt(Timer16::Interrupt::Update);
    Timer16::enableInterruptVector(true, 5);

    // LED blink code "Boot up successful"
    ledseq_green.blink({.limit_blink_cycles = 3, .fulfill = true});  // Default = 200ms ON, 200ms OFF
    ledseq_red.blink({.limit_blink_cycles = 3, .fulfill = true});    // Default = 200ms ON, 200ms OFF

    while (true) {
        ledseq_green.loop();
        ledseq_red.loop();
        // update all standard animations
        // pulse.update();
        // indicator.update();
        // strobe.update();

        // update the custom animations
        // keyFrames.update();
        // pulsePeriod.update();
    }
}
