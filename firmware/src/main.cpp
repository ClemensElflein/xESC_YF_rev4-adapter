#include <modm/platform.hpp>
using namespace modm::platform;
using namespace std::chrono_literals;

int main() {
    GpioC14::setOutput();
    //GpioA0::setOutput();
    while (true) {
        GpioC14::toggle();
        modm::delay(0.5s);
    }
}