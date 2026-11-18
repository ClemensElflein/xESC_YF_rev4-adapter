#pragma once
#include <modm/architecture.hpp>

namespace hardware {
    // Hardware version magic string (stored in the OTP area)
    static constexpr const char HARDWARE_VERSION_MAGIC[] = "ESCYFR4HWV";
    static constexpr size_t HARDWARE_VERSION_MAGIC_SIZE = sizeof(HARDWARE_VERSION_MAGIC);

    struct VersionInfo {
        uint8_t major;
        uint8_t minor;
    };
}