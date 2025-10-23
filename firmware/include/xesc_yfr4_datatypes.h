#ifndef SRC_XESC_YFR4_DATATYPES_H
#define SRC_XESC_YFR4_DATATYPES_H

#include <cstdint>

#define XESCYFR4_MSG_TYPE_STATUS 1
#define XESCYFR4_MSG_TYPE_CONTROL 2
#define XESCYFR4_MSG_TYPE_SETTINGS 3

#define FAULT_UNINITIALIZED (1u << 0)      // 0x0001
#define FAULT_WATCHDOG (1u << 1)           // 0x0002
#define FAULT_UNDERVOLTAGE (1u << 2)       // 0x0004
#define FAULT_OVERVOLTAGE (1u << 3)        // 0x0008
#define FAULT_OVERCURRENT (1u << 4)        // 0x0010
#define FAULT_OVERTEMP_MOTOR (1u << 5)     // 0x0020
#define FAULT_OVERTEMP_PCB (1u << 6)       // 0x0040
#define FAULT_INVALID_HALL (1u << 7)       // 0x0080
#define FAULT_INTERNAL_ERROR (1u << 8)     // 0x0100
#define FAULT_OPEN_LOAD (1u << 9)          // 0x0200
#define FAULT_WRONG_HW_VERSION (1u << 10)  // 0x0400

#pragma pack(push, 1)
struct XescYFR4StatusPacket {
  uint8_t message_type;
  uint32_t seq;
  uint8_t fw_version_major;
  uint8_t fw_version_minor;
  double temperature_pcb;   // temperature of printed circuit board (degrees Celsius)
  double current_input;     // input current (ampere)
  double duty_cycle;        // duty cycle (0 to 1)
  bool direction;           // direction CW/CCW
  uint32_t tacho;           // wheel ticks
  uint32_t tacho_absolute;  // wheel ticks absolute
  uint16_t rpm;             // revolutions per minute (of the axis/shaft)
  int32_t fault_code;
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct XescYFR4ControlPacket {
  uint8_t message_type;
  double duty_cycle;  // duty cycle (0 to 1)
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct XescYFR4SettingsPacket {
  uint8_t message_type;
  float motor_current_limit;
  float min_pcb_temp;
  float max_pcb_temp;
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#endif  // XESC_YFR4_DATATYPES_H
