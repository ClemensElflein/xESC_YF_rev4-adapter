// Hardware CRC-16-CCITT-FALSE wrapper for STM32C011
// Based on STM32C0 Reference Manual Chapter 13
//
// Uses CMSIS CRC_TypeDef directly instead of custom struct.
// CMSIS defines: CRC, CRC_BASE, CRC_TypeDef, CRC_CR_*, CRC_POL_*, etc.
#pragma once

#include <modm/platform.hpp>
#include <cstdint>
#include <cstddef>

namespace hw_crc {

    // Use CMSIS-defined CRC peripheral directly
    // CRC macro expands to: ((CRC_TypeDef *) CRC_BASE)
    // Defined in stm32c011xx.h line 603
    static CRC_TypeDef* const CRC_PERIPH = CRC;

    // Control register bits (use CMSIS defines where available)
    // Note: Some newer CMSIS defines like CRC_CR_POLYSIZE_16 may not exist in older headers
    static constexpr uint32_t CR_RESET = CRC_CR_RESET;
    static constexpr uint32_t CR_POLYSIZE_16 = (1UL << 3);  // CRC_CR_POLYSIZE_0

    /**
     * @brief Initialize CRC peripheral for CRC-16-CCITT-FALSE
     *
     * Configuration:
     * - Polynomial: 0x1021 (CRC-16-CCITT)
     * - Initial value: 0xFFFF
     * - Polynomial size: 16-bit
     * - No input/output bit reversal
     */
    inline void Init() {
        // Enable CRC peripheral clock
        modm::platform::Rcc::enable<modm::platform::Peripheral::Crc>();

        // Configure for CRC-16-CCITT-FALSE
        CRC_PERIPH->POL = 0x1021;           // CRC-16-CCITT polynomial
        CRC_PERIPH->INIT = 0xFFFF;          // Initial value
        CRC_PERIPH->CR = CR_POLYSIZE_16;    // 16-bit polynomial, no reversal
        CRC_PERIPH->CR |= CR_RESET;         // Reset CRC calculator
    }

    /**
     * @brief Calculate CRC-16-CCITT-FALSE using hardware
     *
     * @param data Pointer to data buffer
     * @param length Number of bytes
     * @return uint16_t Calculated CRC value
     */
    inline uint16_t Calculate(const uint8_t* data, size_t length) {
        // Reset CRC unit
        CRC_PERIPH->CR |= CR_RESET;

        // Feed data byte by byte
        for (size_t i = 0; i < length; i++) {
            *reinterpret_cast<volatile uint8_t*>(&CRC_PERIPH->DR) = data[i];
        }

        // Return 16-bit result
        return static_cast<uint16_t>(CRC_PERIPH->DR & 0xFFFF);
    }

} // namespace hw_crc
