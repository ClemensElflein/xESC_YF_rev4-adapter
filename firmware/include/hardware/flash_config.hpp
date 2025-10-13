#pragma once
#include <cstdint>
#include "config/base_types.hpp"

namespace hardware {
    // Minimal flash storage format for hardware config in OTP area
    // DO NOT CHANGE without considering backward compatibility!
#pragma pack(push, 1)  
    struct FlashHardwareConfig {
        char magic[HARDWARE_CONFIG_MAGIC_SIZE];  // Hardware config magic string
        struct {
            uint8_t major;
            uint8_t minor;
        } version;
        uint16_t crc16;  // CRC16-CCITT-FALSE over previous bytes

        // Check if this matches a specific version
        constexpr bool isVersion(uint8_t major, uint8_t minor) const {
            return version.major == major && version.minor == minor;
        }

        // Check if magic string is valid
        bool isValidMagic() const {
            for (size_t i = 0; i < HARDWARE_CONFIG_MAGIC_SIZE - 1; ++i) {
                if (magic[i] != HARDWARE_CONFIG_MAGIC[i]) {
                    return false;
                }
            }
            return magic[HARDWARE_CONFIG_MAGIC_SIZE - 1] == '\0';
        }
    };
#pragma pack(pop)
}