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

#include "board.hpp"
//#include "LED.h"

using namespace Board;
using namespace std::chrono_literals;

// --- UART Debugging ---
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG
// Create an IODeviceWrapper around the Uart Peripheral we want to use
modm::IODeviceWrapper<proto_uart::Uart, modm::IOBuffer::BlockIfFull> loggerDevice;
// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

//LED led_green(PIN_LED_GREEN);
//LED led_red(PIN_LED_RED);

MODM_ISR(TIM14) {
    Timer14::acknowledgeInterruptFlags(Timer14::InterruptFlag::Update);
    MODM_LOG_DEBUG << "Toggle green LED" << modm::endl;
    LedGn::toggle();
}

MODM_ISR(TIM16) {
    Timer16::acknowledgeInterruptFlags(Timer16::InterruptFlag::Update);
    MODM_LOG_DEBUG << "Toggle red LED" << modm::endl;
    LedRd::toggle();
}

int main() {
    Board::initialize();

    // Initialize prototyping-UART for MODM_LOG_*
    proto_uart::Uart::connect<proto_uart::Tx::Tx, proto_uart::Rx::Rx>();
    proto_uart::Uart::initialize<SystemClock, 460800_Bd>();

    MODM_LOG_INFO << "Board & Logger initialized" << modm::endl;

    Timer14::enable();
    Timer14::setMode(Timer14::Mode::UpCounter);
    Timer14::setPeriod<Board::SystemClock>(2000ms);
    Timer14::applyAndReset();
    Timer14::start();
    Timer14::enableInterrupt(Timer14::Interrupt::Update);
    Timer14::enableInterruptVector(true, 5);

    Timer16::enable();
    Timer16::setMode(Timer16::Mode::UpCounter);
    Timer16::setPeriod<Board::SystemClock>(250ms);
    Timer16::applyAndReset();
    Timer16::start();
    Timer16::enableInterrupt(Timer16::Interrupt::Update);
    Timer16::enableInterruptVector(true, 5);

    while (true) {
    }
}
