#include <modm/platform.hpp>
#include "board.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;

int main() {
    Board::initialize();
    Board::LedGn::set();
    while (true) {
        Board::LedGn::toggle();
        Board::LedRd::toggle();
        modm::delay(0.5s);
    }
}
