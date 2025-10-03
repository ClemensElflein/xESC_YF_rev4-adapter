// Hardware Version Detection Implementation for xESC_YF_rev4
//
// This file implements runtime hardware version detection by reading
// a hardware info structure from flash memory.
// 
// Author : Apehaenger
// Created: 2025-09-28

#include "hardware_version.h"

#include <modm/platform.hpp>

namespace hw_version {
    // Global detected hardware major version
    uint8_t g_hw_major_version = 0;  // 0 = uninitialized, defaults to v1.x behavior
}

namespace {
    // Calculate CRC16-CCITT-FALSE.
    //
    // Args:
    //   data: Pointer to data
    //   length: Data length in bytes
    // Returns:
    //   Calculated CRC16
    uint16_t CalculateCrc16CcittFalse(const uint8_t* data, uint16_t length) {
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

    // Verify hardware info structure CRC.
    //
    // Args:
    //   info: Reference to hardware info structure
    // Returns:
    //   true if CRC is valid, false otherwise
    bool VerifyHwInfoCrc(const hw_version::HwInfo& info) {
        // Calculate CRC over first 14 bytes (everything except the CRC field itself)
        uint16_t calculated_crc = CalculateCrc16CcittFalse(
            reinterpret_cast<const uint8_t*>(&info),
            sizeof(hw_version::HwInfo) - sizeof(info.crc16)
        );

        return calculated_crc == info.crc16;
    }

}  // namespace

namespace hw_version {

    void Init() {
        // Read hardware info structure from flash
        const auto* hw_info = reinterpret_cast<const HwInfo*>(HW_INFO_FLASH_ADDR);

        // Check if we have valid hardware version data
        if (hw_info->magic == HW_VERSION_MAGIC && VerifyHwInfoCrc(*hw_info)) {
            // Valid hardware info found - use the major version
            g_hw_major_version = hw_info->major;

            // Debug output (if enabled)
#ifdef PROTO_DEBUG
            MODM_LOG_INFO << "Hardware version detected: " << hw_info->major << "."
                << hw_info->minor << modm::endl;
#endif
        } else {
            // No valid hardware info found - default to HW_V1
            g_hw_major_version = 1;

#ifdef PROTO_DEBUG
            MODM_LOG_INFO << "No valid hardware version data found, defaulting to HW_V1"
                << modm::endl;
#endif
        }
    }
}  // namespace hw_version