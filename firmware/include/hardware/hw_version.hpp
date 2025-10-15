#pragma once
#include <cstdint>
#include "hw_types.hpp"

namespace hardware {
    // ============================================================================
    // Flash Storage Format
    // ============================================================================

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

    // ============================================================================
    // Flash OTP Access
    // ============================================================================

    // STM32C0 flash OTP area location for hardware config
    // Currently (and it's expected that this is fixed), a FlashHardwareConfig struct is 15 bytes long 
    // which is 2 of 128 OTP dwords. So, in future, if required, we could also store multiple versions 
    // every 2 dwords and use the newest (parsing from the end backwards).
    constexpr uintptr_t FLASH_HARDWARE_CONFIG_ADDRESS = 0x1FFF7000;

    // ============================================================================
    // CRC Calculation
    // ============================================================================

    // CRC16-CCITT-FALSE calculation (same as in host generator)
    inline uint16_t calculateCrc16CcittFalse(const uint8_t* data, uint16_t length) {
        uint16_t crc = 0xFFFF;

        for (uint16_t i = 0; i < length; i++) {
            crc ^= (static_cast<uint16_t>(data[i]) << 8);

            for (uint8_t bit = 0; bit < 8; bit++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }

        return crc;
    }

    // ============================================================================
    // Config Validation & Access
    // ============================================================================

    // Load hardware config from flash OTP area
    inline const FlashHardwareConfig& loadFromFlash() {
        const FlashHardwareConfig* flash_ptr =
            reinterpret_cast<const FlashHardwareConfig*>(FLASH_HARDWARE_CONFIG_ADDRESS);

        return *flash_ptr;
    }

    // Validate flash config (magic + CRC)
    inline bool isValidFlashConfig(const FlashHardwareConfig& config) {
        // Check magic string first
        if (!config.isValidMagic()) {
            return false;
        }

        // Calculate CRC over everything except the CRC field itself
        uint16_t calculated_crc = calculateCrc16CcittFalse(
            reinterpret_cast<const uint8_t*>(&config),
            sizeof(FlashHardwareConfig) - sizeof(config.crc16)
        );

        return calculated_crc == config.crc16;
    }

    // ============================================================================
    // Compile-Time Hardware Version Check
    // ============================================================================

    // Check if flashed hardware config matches the compiled binary version
    // This is used to detect when wrong firmware is flashed to wrong hardware
    inline bool checkFlashMatchesBinary() {
        const auto& flash_config = loadFromFlash();

        // If flash config is invalid, assume default behavior based on hardware version:
        // - V1 hardware: Early units may not have OTP record -> assume V1 (return true)
        // - V2 hardware: Should always have OTP record -> mismatch (return false)
        if (!isValidFlashConfig(flash_config)) {
#ifdef HW_V1
            return true;  // V1 firmware on hardware without OTP -> assume V1 hardware (backward compat)
#elif defined(HW_V2)
            return false; // V2 firmware on hardware without OTP -> likely wrong hardware
#else
#error "Either HW_V1 or HW_V2 must be defined"
#endif
        }

        // Check if flash version matches compile-time hardware version
#ifdef HW_V1
        return flash_config.isVersion(1, 0);
#elif defined(HW_V2)
        return flash_config.isVersion(2, 0);
#else
#error "Either HW_V1 or HW_V2 must be defined"
#endif
    }

    // ============================================================================
    // Debug Output
    // ============================================================================

    // Print hardware version information for debugging
    // Call this during initialization when PROTO_DEBUG is enabled
    template<typename IOStream>
    inline void printVersionInfo(IOStream& stream) {
        const auto& flash_config = loadFromFlash();

#ifdef HW_V1
        stream << "Binary: HW v1.0";
#elif defined(HW_V2)
        stream << "Binary: HW v2.0";
#else
        stream << "Binary: HW version unknown";
#endif

        if (isValidFlashConfig(flash_config)) {
            stream << ", OTP Flash: v"
                << flash_config.version.major << "."
                << flash_config.version.minor;

            if (!checkFlashMatchesBinary()) {
                stream << " [MISMATCH - WRONG HARDWARE!]";
            }
        } else {
            stream << ", OTP Flash: invalid/empty";
#ifdef HW_V1
            stream << " [OK - V1 backward compat]";
#else
            stream << " [ERROR - V2 requires OTP]";
#endif
        }
    }
}
