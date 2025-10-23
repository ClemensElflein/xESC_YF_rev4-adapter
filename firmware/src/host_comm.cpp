// Created by Jörg Ebeling on 2025-10-18.
// Copyright (c) 2025 Jörg Ebeling for OpenMower/Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons
// Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't
// try to sell the design or products based on it without getting my consent
// first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "host_comm.hpp"

#include <cstring>
#include <modm/platform.hpp>

#include "COBS.h"
#include "board.hpp"
#include "config.h"
#include "debug.h"
#include "hw_crc.hpp"
#include "jump_system_bootloader.hpp"
#include "led_controller.hpp"
#include "motor_control.hpp"

using namespace Board;

#define MILLIS modm::Clock::now().time_since_epoch().count()

extern LedController error_led;

namespace host_comm {

// Internal state.
namespace {
constexpr size_t kBufferSize = 100;
uint8_t tx_buffer_[kBufferSize];
uint8_t rx_buffer_[kBufferSize];
unsigned int rx_buffer_idx_;
COBS cobs_;

uint32_t last_watchdog_millis_;
XescYFR4SettingsPacket settings_ = {};
bool settings_valid_;
}  // namespace

void Init() {
  last_watchdog_millis_ = 0;
  settings_valid_ = false;
  rx_buffer_idx_ = 0;

  // Initialize hardware CRC peripheral
  hw_crc::Init();
}

// Signal comm errors.
void SignalCommError() {
  error_led.Blink({.on_time_ms = 20, .off_time_ms = 30, .blink_cycle_limit = 1, .post_pause_ms = 0, .complete = true});
}

void SendMessage(void* message, size_t size) {
  uint8_t* data_pointer = static_cast<uint8_t*>(message);

#ifdef PROTO_DEBUG_HOST_TX
  MODM_LOG_DEBUG << "TX[" << size << " bytes] Raw:";
  for (size_t i = 0; i < size - 2; ++i) {
    MODM_LOG_DEBUG << " " << HEX_BYTE(data_pointer[i]);
  }
  MODM_LOG_DEBUG << modm::endl;
#endif

  // Calculate CRC using hardware peripheral (7x faster than software)
  uint16_t crc = hw_crc::Calculate(data_pointer, size - 2);
  data_pointer[size - 1] = (crc >> 8) & 0xFF;
  data_pointer[size - 2] = crc & 0xFF;

#ifdef PROTO_DEBUG_HOST_TX
  MODM_LOG_DEBUG << "    [" << size << " bytes] With CRC:";
  for (size_t i = 0; i < size; ++i) {
    MODM_LOG_DEBUG << " " << HEX_BYTE(data_pointer[i]);
  }
  MODM_LOG_DEBUG << " (CRC: " << HEX_BYTE(crc >> 8) << HEX_BYTE(crc & 0xFF) << ")" << modm::endl;
#endif

  // Encode message.
  size_t encoded_size = cobs_.encode(static_cast<uint8_t*>(message), size, tx_buffer_);
  tx_buffer_[encoded_size] = 0;
  encoded_size++;

#ifdef PROTO_DEBUG_HOST_TX
  MODM_LOG_DEBUG << "    [" << encoded_size << " bytes] COBS:";
  for (size_t i = 0; i < encoded_size; ++i) {
    MODM_LOG_DEBUG << " " << HEX_BYTE(tx_buffer_[i]);
  }
  MODM_LOG_DEBUG << modm::endl << modm::flush;
#endif

  // Write bytes to UART.
  for (size_t i = 0; i < encoded_size;) {
    if (host::Uart::write(tx_buffer_[i])) {
      i++;
    }
  }
}

// Decode and process a complete COBS-encoded packet.
static void DecodeAndProcessPacket(uint8_t* buffer, size_t length) {
  static uint8_t pkt_buffer[kBufferSize];

#ifdef PROTO_DEBUG_HOST_RX
  MODM_LOG_DEBUG << "RX[" << length << " bytes] COBS:";
  for (size_t i = 0; i < length; ++i) {
    MODM_LOG_DEBUG << " " << HEX_BYTE(buffer[i]);
  }
  MODM_LOG_DEBUG << modm::endl;
#endif

  size_t pkt_size = cobs_.decode(buffer, length - 1, pkt_buffer);

#ifdef PROTO_DEBUG_HOST_RX
  MODM_LOG_DEBUG << "    [" << pkt_size << " bytes] Decoded:";
  for (size_t i = 0; i < pkt_size; ++i) {
    MODM_LOG_DEBUG << " " << HEX_BYTE(pkt_buffer[i]);
  }
  MODM_LOG_DEBUG << modm::endl << modm::flush;
#endif

  // Check minimum packet size (1 type + 1 data + 2 CRC).
  if (pkt_size < 3) {
#ifdef PROTO_DEBUG
    MODM_LOG_WARNING << "WARN: RX Packet too short" << modm::endl << modm::flush;
#endif
    SignalCommError();
    return;
  }

  // Verify CRC using hardware peripheral
  uint16_t crc = hw_crc::Calculate(pkt_buffer, pkt_size - 2);

  if (pkt_buffer[pkt_size - 1] != ((crc >> 8) & 0xFF) || pkt_buffer[pkt_size - 2] != (crc & 0xFF)) {
#ifdef PROTO_DEBUG
    MODM_LOG_WARNING << "WARN: RX CRC mismatch (expected: " << HEX_BYTE(crc >> 8) << HEX_BYTE(crc & 0xFF)
                     << ", got: " << HEX_BYTE(pkt_buffer[pkt_size - 2]) << HEX_BYTE(pkt_buffer[pkt_size - 1]) << ")"
                     << modm::endl
                     << modm::flush;
#endif
    SignalCommError();
    return;
  }

  // Process packet based on type.
  switch (pkt_buffer[0]) {
    case XESCYFR4_MSG_TYPE_CONTROL: {
      if (pkt_size != sizeof(XescYFR4ControlPacket)) {
        SignalCommError();
        return;
      }
      // Got control packet.
      UpdateWatchdog();
      XescYFR4ControlPacket* packet = reinterpret_cast<XescYFR4ControlPacket*>(pkt_buffer);
#ifdef PROTO_DEBUG_COMMS
      MODM_LOG_INFO << "RX CONTROL: duty_cycle: " << packet->duty_cycle << modm::endl << modm::flush;
#endif
      motor_control::SetDutySetpoint(packet->duty_cycle);
      break;
    }
    case XESCYFR4_MSG_TYPE_SETTINGS: {
      if (pkt_size != sizeof(XescYFR4SettingsPacket)) {
        settings_valid_ = false;
        SignalCommError();
        return;
      }
#ifdef PROTO_DEBUG_COMMS
      MODM_LOG_INFO << "RX SETTINGS" << modm::endl << modm::flush;
#endif
      XescYFR4SettingsPacket* packet = reinterpret_cast<XescYFR4SettingsPacket*>(pkt_buffer);
      settings_ = *packet;
      settings_valid_ = true;
      break;
    }
    default:
      // Wrong/unknown packet type.
#ifdef PROTO_DEBUG
      MODM_LOG_WARNING << "WARN: RX Unknown packet type 0x" << HEX_BYTE(pkt_buffer[0]) << modm::endl << modm::flush;
#endif
      SignalCommError();
      break;
  }
}

void ProcessUartData() {
  uint8_t data;
  while (host::Uart::read(data)) {
    rx_buffer_[rx_buffer_idx_++] = data;

    if (rx_buffer_idx_ >= kBufferSize) {
      // Buffer overflow - reset and signal error.
      SignalCommError();
      rx_buffer_idx_ = 0;
      return;
    }

    if (data == 0) {
      // COBS end marker - process packet.
      DecodeAndProcessPacket(rx_buffer_, rx_buffer_idx_);
      rx_buffer_idx_ = 0;
      return;
    }

    // Check for bootloader trigger string.
    if (rx_buffer_idx_ == sizeof(BOOTLOADER_TRIGGER_STR) &&
        strcmp(reinterpret_cast<const char*>(rx_buffer_), BOOTLOADER_TRIGGER_STR) == 0) {
      jump_system_bootloader();
    }
  }
}

bool HasWatchdogTimedOut() {
  return (MILLIS - last_watchdog_millis_) > WATCHDOG_TIMEOUT_MILLIS;
}

void UpdateWatchdog() {
  last_watchdog_millis_ = MILLIS;
}

bool AreSettingsValid() {
  return settings_valid_;
}

const XescYFR4SettingsPacket& GetSettings() {
  return settings_;
}

}  // namespace host_comm
