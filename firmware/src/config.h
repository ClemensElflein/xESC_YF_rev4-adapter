#pragma once

// #define PROTO_DEBUG  // Enable for debug logging on proto UART2
// #define PROTO_DEBUG_BAUD 921600_Bd
// #define PROTO_DEBUG_MOTOR  // Enable motor specific debugging messages
// #define PROTO_DEBUG_COMMS  // Enable Comms msgs of received and sent messages
// #define PROTO_DEBUG_HOST_RX  // Enable host RX debugging messages (COBS, Packet, ...)
// #define PROTO_DEBUG_HOST_TX  // Enable host TX debugging messages (COBS, Packet, ...)
// #define PROTO_DEBUG_ADC  // Enable ADC specific debugging messages

#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG  // ERROR, WARNING, INFO, DEUG

#define STATUS_CYCLE 20ms
// Number of STATUS_CYCLES before motor get declared as stopped (if tacho doesn't change)
#define NUM_STATUS_CYCLES_MOTOR_STOPPED 4
#define WATCHDOG_TIMEOUT_MILLIS 500
#define BOOTLOADER_TRIGGER_STR "Reboot into Bootloader"
#define BOOTLOADER_ADDR 0x1FFF0000UL

// Time to keep motor controller in fault once a fault occurs
#define MIN_FAULT_TIME_MILLIS 2000

// SA (Hall) Capture-Compare-Timer, RPM-Calc
// Rated RPM = 3500 => Targeted max. RPM = 5000
// 5000 RPM = 83.3333 RPS * 4 (Halls) * 2 (Both edges) = 666.667 Cycles/s = ~1.5ms per Hall edge
// 24MHz / 15 (Prescaler) = 0.625 uS per tick => 1.5ms / 0.625us = 2400 ticks
#define SA_TIMER_PRESCALER 15

// ADC
#define ADC_RESOLUTION Resolution::Bits12
#define ADC_NUM_CODES 4096U

// Hardware limits before going into fault
#define HW_LIMIT_PCB_TEMP 80.0f
#define HW_LIMIT_CURRENT 1.25f
