#pragma once
#include <cstdint>
#include "hw_types.hpp"

namespace hardware {
    // ============================================================================
    // OTP Record Format
    // ============================================================================

    // Minimal OTP record format for the hardware version
    // DO NOT CHANGE without considering backward compatibility!
#pragma pack(push, 1)  
    struct OtpHardwareVersion {
        char magic[HARDWARE_VERSION_MAGIC_SIZE];  // Hardware version magic string
        VersionInfo version;
        uint16_t crc16;  // CRC16-CCITT-FALSE over previous bytes

        // Check if this matches a specific version
        constexpr bool isVersion(uint8_t major, uint8_t minor) const {
            return version.major == major && version.minor == minor;
        }

        // Check if magic string is valid
        bool isValidMagic() const {
            for (size_t i = 0; i < HARDWARE_VERSION_MAGIC_SIZE - 1; ++i) {
                if (magic[i] != HARDWARE_VERSION_MAGIC[i]) {
                    return false;
                }
            }
            return magic[HARDWARE_VERSION_MAGIC_SIZE - 1] == '\0';
        }
    };
#pragma pack(pop)

    static_assert(sizeof(OtpHardwareVersion) == 15, "OtpHardwareVersion layout changed!");

    // ============================================================================
    // OTP Access
    // ============================================================================

    // STM32C0 OTP area address of the hardware-version record.
    // The record is 15 bytes long, which is 2 of the 128 OTP double words (64 bit each).
    constexpr uintptr_t OTP_HARDWARE_VERSION_ADDRESS = 0x1FFF7000;

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
    // Record Validation & Access
    // ============================================================================

    // Read the hardware-version record from the OTP area
    inline const OtpHardwareVersion& loadHardwareVersion() {
        const OtpHardwareVersion* otp_ptr =
            reinterpret_cast<const OtpHardwareVersion*>(OTP_HARDWARE_VERSION_ADDRESS);

        return *otp_ptr;
    }

    // Validate the hardware-version record (magic + CRC)
    inline bool isValidHardwareVersion(const OtpHardwareVersion& record) {
        // Check magic string first
        if (!record.isValidMagic()) {
            return false;
        }

        // Calculate CRC over everything except the CRC field itself
        uint16_t calculated_crc = calculateCrc16CcittFalse(
            reinterpret_cast<const uint8_t*>(&record),
            sizeof(OtpHardwareVersion) - sizeof(record.crc16)
        );

        return calculated_crc == record.crc16;
    }

    // ============================================================================
    // Hardware profiles (analog settings per hardware revision)
    // ============================================================================

    // The analog configuration is compiled into the firmware, not stored in OTP.
    // At boot the active profile is selected from the OTP hardware version:
    //   - all versions < 2.1 (incl. no/legacy OTP): R17 = 1k,   1.25 A limit
    //   - HW v2.1:                                  R17 = 510R, 2.0 A limit
    //
    // Current-limit notes for HW v2.1:
    //   - 3.0 A: nominal TPS1H100 hardware current limit (R14 = 820R,
    //            +/-14% => 2.59..3.43 A). This is the hardware safety net,
    //            not a software control target.
    //   - 2.5 A: maximum sensible software limit: below the guaranteed
    //            minimum TPS limit (2.59 A). The ADC range (0..3.1 V with
    //            R17 = 510R => 1.02 V/A) covers up to ~3.0 A.
    //   - 2.0 A: selected profile limit: headroom over the motor I_max
    //            (1.5 A) at full 7S battery voltage, but below the TPS limit.
    struct HardwareProfile {
        const char* name;
        uint16_t r_cs_ohm;
        float current_limit;
    };

    inline constexpr HardwareProfile HW_PROFILE_DEFAULT = {"HW v1.x/v2.0", 1000, 1.25f};
    inline constexpr HardwareProfile HW_PROFILE_V2_1 = {"HW v2.1", 510, 2.0f};

    // Active profile pointer, set once at boot by initActiveProfile().
    inline const HardwareProfile* active_profile = &HW_PROFILE_DEFAULT;

    inline const HardwareProfile& getActiveProfile() {
        return *active_profile;
    }

    inline void initActiveProfile() {
        const auto& hw_version = loadHardwareVersion();
        if (isValidHardwareVersion(hw_version) && hw_version.version.major == 2 && hw_version.version.minor >= 1) {
            active_profile = &HW_PROFILE_V2_1;
        } else {
            active_profile = &HW_PROFILE_DEFAULT;
        }
    }

    // Current-sense resistor (R17/R_CS) of the active hardware profile
    inline uint16_t getCurrentSenseResistor() {
        return active_profile->r_cs_ohm;
    }

    // Firmware overcurrent limit of the active hardware profile
    inline float getHardwareCurrentLimit() {
        return active_profile->current_limit;
    }

    // ============================================================================
    // Compile-Time Hardware Version Check
    // ============================================================================

    // Check if the OTP hardware version matches the compiled binary version
    // This is used to detect when wrong firmware is flashed to wrong hardware
    inline bool checkHardwareVersionMatchesBinary() {
        const auto& otp_version = loadHardwareVersion();

        // If the OTP record is invalid, assume default behavior based on hardware version:
        // - V1 hardware: Early units may not have OTP record -> assume V1 (return true)
        // - V2 hardware: Should always have OTP record -> mismatch (return false)
        if (!isValidHardwareVersion(otp_version)) {
#ifdef HW_V1
            return true;  // V1 firmware on hardware without OTP -> assume V1 hardware (backward compat)
#elif defined(HW_V2)
            return false; // V2 firmware on hardware without OTP -> likely wrong hardware
#elif defined(HW_BRAND)
            return true;
#else
#error "Either HW_V1 or HW_V2 must be defined"
#endif
        }

        // Check if the OTP version matches the compile-time hardware version
#ifdef HW_V1
        return otp_version.isVersion(1, 0);
#elif defined(HW_V2)
        // A single HW_V2 binary supports both v2.0 (R17 = 1k) and v2.1
        // (R17 = 510R). The analog profile is selected from the OTP version.
        return otp_version.isVersion(2, 0) || otp_version.isVersion(2, 1);
#elif defined(HW_BRAND)
        return true;
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
        const auto& otp_version = loadHardwareVersion();

#ifdef HW_V1
        stream << "Binary: HW v1.0";
#elif defined(HW_V2)
        stream << "Binary: HW v2.0/v2.1";
#else
        stream << "Binary: HW version unknown";
#endif

        if (isValidHardwareVersion(otp_version)) {
            stream << ", OTP: v"
                << otp_version.version.major << "."
                << otp_version.version.minor;

            // Analog settings of the active hardware profile
            stream << ", profile=" << getActiveProfile().name
                   << ", R_CS=" << getActiveProfile().r_cs_ohm << "R"
                   << ", I_limit=" << getActiveProfile().current_limit << "A";

            if (!checkHardwareVersionMatchesBinary()) {
                stream << " [MISMATCH - WRONG HARDWARE!]";
            }
        } else {
            stream << ", OTP: invalid/empty";
#ifdef HW_V1
            stream << " [OK - V1 backward compat]";
#else
            stream << " [ERROR - V2 requires OTP]";
#endif
        }
    }
}
