// Hardware version generator - generates minimal OTP record binaries

#include <iostream>
#include <fstream>
#include <cstdint>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>

// Include the OTP record definition
#include "../include/hardware/hw_version.hpp"

// Hardware version entries
struct HardwareVersion {
    uint8_t major;
    uint8_t minor;
    const char* name;
};

// Available hardware versions
const HardwareVersion hw_versions[] = {
    {1, 0, "v1.0"},
    {2, 0, "v2.0"},
    {2, 1, "v2.1"}
};

bool GenerateVersionBinary(const HardwareVersion& hw_version, const std::string& output_file) {
    // Create the OTP hardware-version record
    hardware::OtpHardwareVersion otp_version = {};

    // Set magic string using the shared constant
    strcpy(otp_version.magic, hardware::HARDWARE_VERSION_MAGIC);

    // Set version
    otp_version.version.major = hw_version.major;
    otp_version.version.minor = hw_version.minor;

    // Calculate CRC over everything except the CRC field itself
    uint16_t crc = hardware::calculateCrc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&otp_version),
        sizeof(hardware::OtpHardwareVersion) - sizeof(otp_version.crc16)
    );
    otp_version.crc16 = crc;

    // Write binary file
    std::ofstream file(output_file, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot create " << output_file << std::endl;
        return false;
    }

    file.write(reinterpret_cast<const char*>(&otp_version), sizeof(otp_version));
    file.close();

    std::cout << "Generated " << output_file << ": " << hw_version.name
        << " (v" << static_cast<int>(hw_version.major) << "." << static_cast<int>(hw_version.minor) << ")"
        << ", Size=" << sizeof(otp_version) << " bytes"
        << ", CRC=0x" << std::hex << crc << std::dec << std::endl;

    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Hardware Version Generator ===" << std::endl;
    std::cout << "Generating hardware version binaries..." << std::endl;

    // Use output directory from command line argument if provided
    std::string output_dir = ".";
    if (argc > 1) {
        output_dir = argv[1];
        // Create output directory if it doesn't exist
        mkdir(output_dir.c_str(), 0755);
    }

    bool success = true;

    // Generate all available hardware version binaries
    for (const auto& hw_version : hw_versions) {
        std::string filename = output_dir + "/hw_config_v" +
            std::to_string(hw_version.major) + "." +
            std::to_string(hw_version.minor) + ".bin";
        success &= GenerateVersionBinary(hw_version, filename);
    }

    if (success) {
        std::cout << "All hardware version binaries generated successfully." << std::endl;
        std::cout << "These can now be written to the OTP area at 0x1FFF7000." << std::endl;
        return 0;
    } else {
        std::cerr << "Failed to generate some hardware version binaries!" << std::endl;
        return 1;
    }
}
