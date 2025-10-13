// Hardware config generator - generates minimal flash config binaries

#include <iostream>
#include <fstream>
#include <cstdint>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>

// Include the simple flash config (no GPIO dependencies)
#include "../include/hardware/flash_config.hpp"

// Hardware version configurations
struct HardwareVersionConfig {
    uint8_t major;
    uint8_t minor;
    const char* name;
};

// Available hardware versions
const HardwareVersionConfig hw_configs[] = {
    {1, 0, "v1.0"},
    {2, 0, "v2.0"}
};

// CRC16-CCITT-FALSE calculation (same as in firmware)
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

bool GenerateConfigBinary(const HardwareVersionConfig& hw_config, const std::string& output_file) {
    // Create the minimal flash config structure
    hardware::FlashHardwareConfig flash_config;

    // Set magic string using the shared constant
    strcpy(flash_config.magic, hardware::HARDWARE_CONFIG_MAGIC);

    // Set version
    flash_config.version.major = hw_config.major;
    flash_config.version.minor = hw_config.minor;

    // Calculate CRC over everything except the CRC field itself
    uint16_t crc = CalculateCrc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&flash_config),
        sizeof(hardware::FlashHardwareConfig) - sizeof(flash_config.crc16)
    );
    flash_config.crc16 = crc;

    // Write binary file
    std::ofstream file(output_file, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot create " << output_file << std::endl;
        return false;
    }

    file.write(reinterpret_cast<const char*>(&flash_config), sizeof(flash_config));
    file.close();

    std::cout << "Generated " << output_file << ": " << hw_config.name
        << " (v" << static_cast<int>(hw_config.major) << "." << static_cast<int>(hw_config.minor) << ")"
        << ", Size=" << sizeof(flash_config) << " bytes"
        << ", CRC=0x" << std::hex << crc << std::dec << std::endl;

    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Hardware Config Generator ===" << std::endl;
    std::cout << "Generating hardware config binaries from constexpr configurations..." << std::endl;

    // Use output directory from command line argument if provided
    std::string output_dir = ".";
    if (argc > 1) {
        output_dir = argv[1];
        // Create output directory if it doesn't exist
        mkdir(output_dir.c_str(), 0755);
    }

    bool success = true;

    // Generate all available hardware configurations
    for (const auto& hw_config : hw_configs) {
        std::string filename = output_dir + "/hw_config_v" +
            std::to_string(hw_config.major) + "." +
            std::to_string(hw_config.minor) + ".bin";
        success &= GenerateConfigBinary(hw_config, filename);
    }

    if (success) {
        std::cout << "All hardware config binaries generated successfully!" << std::endl;
        std::cout << "These can now be flashed with WRP protection using the TCL branding scripts." << std::endl;
        return 0;
    } else {
        std::cerr << "Failed to generate some hardware config binaries!" << std::endl;
        return 1;
    }
}