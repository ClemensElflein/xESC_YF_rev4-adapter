#pragma once
#include "flash_config.hpp"
#include "config/base_types.hpp"
#include "config/v1_0.hpp"
#include "config/v2_0.hpp"

namespace hardware {
    // STM32C0 flash OTP area location for hardware config
    // Currently (and it's expected that this is fixed), a FlashHardwareConfig struct is 15 bytes long which is 2 of 128 OTP dwords.
    // So, in future, if required, we could also store multiple versions every 2 dwords and use the newest (parsing from the end backwards).
    constexpr uintptr_t FLASH_HARDWARE_CONFIG_ADDRESS = 0x1FFF7000;

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

    // Load hardware config from flash OTP area
    inline const FlashHardwareConfig& loadFromFlash() {
        // Access flash OTP data directly
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
    // Hardware config dispatcher - calls the appropriate templated function based on version
    template<typename Function>
    auto dispatchHardwareConfig(const FlashHardwareConfig& flashConfig, Function&& func) {
        if (flashConfig.isVersion(1, 0)) {
            return func(hardware::versions::v1_0);
        } else if (flashConfig.isVersion(2, 0)) {
            return func(hardware::versions::v2_0);
        } else {
            // Assume hardware v1.0 if there's no (valid) hardware config in flash
            return func(hardware::versions::v1_0);
        }
    }
}