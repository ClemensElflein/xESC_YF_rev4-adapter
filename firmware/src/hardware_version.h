// Hardware Version Detection for xESC_YF_rev4
//
// This file provides runtime hardware version detection by reading
// a hardware info structure from flash memory.
//
// Author : Apehaenger
// Created: 2025-09-28

#ifndef HARDWARE_VERSION_H_
#define HARDWARE_VERSION_H_

#include <cstdint>

namespace hw_version {

    // Hardware info structure location (first 16 bytes of reserved page 15)
    constexpr uint32_t HW_INFO_FLASH_ADDR = 0x08007800U;

    // Hardware version magic constant 
    // Flash contains bytes: 0x48 0x57 0x56 0x52 ("HWVR" ASCII string)
    // When read as uint32_t little-endian: 0x52565748
    constexpr uint32_t HW_VERSION_MAGIC = 0x52565748U;

    // Hardware info structure (16 bytes total, matches TCL branding script format)
    struct __attribute__((packed)) HwInfo {
        uint32_t magic;         // Magic: bytes 0x48,0x57,0x56,0x52 ("HWVR"), reads as 0x52565748 when little-endian
        uint8_t major;          // Major version number
        uint8_t minor;          // Minor version number  
        uint8_t led_green_port; // GPIO port for green LED (0=A, 1=B, 2=C, etc.)
        uint8_t led_green_pin;  // GPIO pin number for green LED
        uint8_t led_red_port;   // GPIO port for red LED (0=A, 1=B, 2=C, etc.)
        uint8_t led_red_pin;    // GPIO pin number for red LED
        uint8_t reserved;       // Reserved for future use
        uint8_t reserved2;      // Additional reserved byte
        uint16_t reserved3;     // Additional reserved space (2 bytes)
        uint16_t crc16;         // CRC16-CCITT-FALSE over first 14 bytes
    };

    // Global detected hardware major version (1=v1.x, 2=v2.x, 0=unknown/default to v1.x)
    extern uint8_t g_hw_major_version;

    // Initialize hardware version detection.
    //
    // Reads the hardware info structure from flash and determines
    // the hardware major version. If no valid structure is found,
    // defaults to version 1.
    //
    // This function should be called early in main() before any
    // hardware-specific initialization.
    void Init();

    // Check if running on Hardware Version 1.
    //
    // Returns true if v1.x, false otherwise.
    inline bool IsV1() {
        return g_hw_major_version == 1 || g_hw_major_version == 0;  // Default to v1.x
    }

    // Check if running on Hardware Version 2.
    //
    // Returns true if v2.x, false otherwise.
    inline bool IsV2() {
        return g_hw_major_version == 2;
    }

}  // namespace hw_version

#endif  // HARDWARE_VERSION_H_