#pragma once

#include "COBS.h"
#ifdef CRC  // FIXME remove modm/STM32 CRC or use it
#undef CRC
#endif
#include "CRC.h" 
#include "config.h"
#include "xesc_yfr4_datatypes.h"
#include "LedSeq.hpp"
#include "../src/board.hpp"
#include "debug.h"
#include <cstring>

/**
 * @brief Hardware-agnostic Host Communication class
 *
 * Handles UART communication with COBS encoding/decoding, CRC validation,
 * and packet processing. Uses template-based hardware configuration for
 * hardware-specific pins and LED status indication.
 *
 * @tparam HardwareController The hardware controller type (contains config and LED access)
 */
template<typename HardwareController>
class HostComm {
public:
    static constexpr size_t BUFFER_SIZE = 100;

    /**
     * @brief Constructor - takes reference to hardware controller for LED and config access
     */
    explicit HostComm(HardwareController& controller) : hw_controller(controller) {
        using namespace modm::literals;

        // Initialize UART hardware - access config through hardware controller
        using HardwareConfig = typename HardwareController::HardwareConfig;
        using HostConfigType = typename HardwareConfig::HostConfig;
        HostConfigType::Uart::template connect<typename HostConfigType::Tx::Tx, typename HostConfigType::Rx::Rx>();
        HostConfigType::Uart::template initialize<Board::SystemClock, 115200_Bd>();
        HostConfigType::Shutdown::setInput(modm::platform::Gpio::InputType::Floating);

        // Remap UART pins
        modm::platform::GpioA9::remap();  // Remap A9 -> A11
        modm::platform::GpioA10::remap(); // Remap A10 -> A12
    }

    /**
     * @brief Send a message via UART with COBS encoding and CRC
     */
    void sendMessage(void* message, size_t size) {
        // Packages have to be at least 1 byte of type + 1 byte of data + 2 bytes of CRC
        if (size < 4) {
            signalCommError();
            return;
        }
        uint8_t* data_pointer = (uint8_t*)message;

        // Calc CRC
        uint16_t crc = CRC::Calculate(data_pointer, size - 2, CRC::CRC_16_CCITTFALSE());
        data_pointer[size - 1] = (crc >> 8) & 0xFF;
        data_pointer[size - 2] = crc & 0xFF;

        // Encode message
        size_t encoded_size = cobs.encode((uint8_t*)message, size, buffer_tx);
        buffer_tx[encoded_size] = 0;
        encoded_size++;

        // Write bytes to UART
        using HardwareConfig = typename HardwareController::HardwareConfig;
        using HostConfigType = typename HardwareConfig::HostConfig;
        for (size_t i = 0; i < encoded_size;) {
            if (HostConfigType::Uart::write(buffer_tx[i])) {
                i++;
            }
        }
    }

    /**
     * @brief Handle incoming UART data and process complete packets
     */
    void handleRxData() {
        using HardwareConfig = typename HardwareController::HardwareConfig;
        using HostConfigType = typename HardwareConfig::HostConfig;
        uint8_t data;
        while (HostConfigType::Uart::read(data)) {
            buffer_rx[buffer_rx_idx++] = data;
            if (buffer_rx_idx >= BUFFER_SIZE) { // Buffer is full, but no COBS end marker. Reset
                signalCommError();
                buffer_rx_idx = 0;
                return;
            }

            if (data == 0) { // COBS end marker
                processReceivedPacket();
                buffer_rx_idx = 0;
                return;
            }

            // Bootloader trigger string?
            if (buffer_rx_idx == sizeof BOOTLOADER_TRIGGER_STR &&
                strcmp((const char*)buffer_rx, BOOTLOADER_TRIGGER_STR) == 0) {
                // Use hardware controller's jumpSystemBootloader method
                hw_controller.jumpSystemBootloader();
            }
        }
    }

    /**
     * @brief Check if shutdown signal is active
     */
    bool isShutdownActive() const {
        using HardwareConfig = typename HardwareController::HardwareConfig;
        using HostConfigType = typename HardwareConfig::HostConfig;
        return HostConfigType::Shutdown::read();
    }

private:
    HardwareController& hw_controller;

    // Helper methods to access LED sequencers through hardware controller  
    auto& getGreenLed() { return hw_controller.getGreenLed(); }
    auto& getRedLed() { return hw_controller.getRedLed(); }

    // Communication buffers
    uint8_t buffer_rx[BUFFER_SIZE];
    unsigned int buffer_rx_idx = 0;
    uint8_t buffer_tx[BUFFER_SIZE];
    COBS cobs;

    /**
     * @brief Signal communication error via red LED
     */
    void signalCommError() {
        getRedLed().blink({ .on = 20, .off = 30, .limit_blink_cycles = 1, .post_pause = 0, .fulfill = true });
    }

    /**
     * @brief Process a complete COBS encoded packet
     */
    void processReceivedPacket() {
        static uint8_t pkt_buffer[BUFFER_SIZE]; // COBS decoded packet buffer

#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "RX[" << buffer_rx_idx << " bytes] COBS:";
        for (unsigned int i = 0; i < buffer_rx_idx; ++i) MODM_LOG_DEBUG << " " << HEX_BYTE(buffer_rx[i]);
        MODM_LOG_DEBUG << modm::endl;
#endif

        size_t pkt_size = cobs.decode(buffer_rx, buffer_rx_idx - 1, (uint8_t*)pkt_buffer);

#ifdef PROTO_DEBUG_HOST_RX
        MODM_LOG_DEBUG << "    [" << pkt_size << " bytes] Decoded:";
        for (size_t i = 0; i < pkt_size; ++i) MODM_LOG_DEBUG << " " << HEX_BYTE(pkt_buffer[i]);
        MODM_LOG_DEBUG << modm::endl << modm::flush;
#endif

        // Calculate the CRC only if we have at least three bytes (two CRC, one data)
        if (pkt_size < 3) {
#ifdef PROTO_DEBUG_HOST_RX
            MODM_LOG_DEBUG << "    ERROR: Packet too short" << modm::endl << modm::flush;
#endif
            signalCommError();
            return;
        }

        // Check CRC
        uint16_t crc = CRC::Calculate(pkt_buffer, pkt_size - 2, CRC::CRC_16_CCITTFALSE());
        if (pkt_buffer[pkt_size - 1] != ((crc >> 8) & 0xFF) ||
            pkt_buffer[pkt_size - 2] != (crc & 0xFF)) {
#ifdef PROTO_DEBUG_HOST_RX
            MODM_LOG_DEBUG << "    ERROR: CRC mismatch (expected: "
                << HEX_BYTE(crc >> 8) << HEX_BYTE(crc & 0xFF)
                << ", got: " << HEX_BYTE(pkt_buffer[pkt_size - 2]) << HEX_BYTE(pkt_buffer[pkt_size - 1])
                << ")" << modm::endl << modm::flush;
#endif
            signalCommError();
            return;
        }

        // Process packet based on type
        switch (pkt_buffer[0]) {
        case XESCYFR4_MSG_TYPE_CONTROL:
#ifdef PROTO_DEBUG_HOST_RX
            MODM_LOG_DEBUG << "    Type: CONTROL" << modm::endl << modm::flush;
#endif
            handleControlPacket(pkt_buffer, pkt_size);
            break;
        case XESCYFR4_MSG_TYPE_SETTINGS:
#ifdef PROTO_DEBUG_HOST_RX
            MODM_LOG_DEBUG << "    Type: SETTINGS" << modm::endl << modm::flush;
#endif
            handleSettingsPacket(pkt_buffer, pkt_size);
            break;
        default:
#ifdef PROTO_DEBUG_HOST_RX
            MODM_LOG_DEBUG << "    ERROR: Unknown packet type 0x" << HEX_BYTE(pkt_buffer[0])
                << modm::endl << modm::flush;
#endif
            // Wrong/unknown packet type
            signalCommError();
            break;
        }
    }

    /**
     * @brief Handle control packet (to be implemented or connected to callbacks)
     */
    void handleControlPacket(uint8_t* pkt_buffer, size_t pkt_size) {
        (void)pkt_buffer; // TODO: Will be used when implementing control packet handling
        if (pkt_size != sizeof(struct XescYFR4ControlPacket)) {
            signalCommError();
            return;
        }
        // TODO: Add callback mechanism or integrate with main application
        // For now this is a placeholder for the migration
    }

    /**
     * @brief Handle settings packet (to be implemented or connected to callbacks)
     */
    void handleSettingsPacket(uint8_t* pkt_buffer, size_t pkt_size) {
        (void)pkt_buffer; // TODO: Will be used when implementing settings packet handling
        if (pkt_size != sizeof(struct XescYFR4SettingsPacket)) {
            signalCommError();
            return;
        }
        // TODO: Add callback mechanism or integrate with main application
        // For now this is a placeholder for the migration
    }
};