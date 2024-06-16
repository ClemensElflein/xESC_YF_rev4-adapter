#pragma once

#ifdef HW_RMECOWV100 // Dev ButtonBoard while waiting for PCB

// LEDs
#define PIN_LED_GREEN PB11
#define PIN_LED_RED PB12

// Serial communication with host (WROOM2)
#define HOST_RX PA10
#define HOST_TX PA9

// Serial print(f) debugging (LowLevel Serial Pins of ButtonBoard)
#define DEBUG_RX PA3
#define DEBUG_TX PA2

#else

// LEDs
#define PIN_LED_GREEN PC15
#define PIN_LED_RED PC14

// Serial communication with host
#define HOST_RX PA12
#define HOST_TX PA11

// Serial print(f) debugging (LowLevel Serial Pins of ButtonBoard)
#define DEBUG_RX PA5
#define DEBUG_TX PA4

#endif

// Timer
#define TIMER_STATUS TIM14

// old

#define PIN_CURRENT_SENSE 29
#define PIN_TEMP_MOTOR 28
#define PIN_TEMP_PCB 26
#define PIN_VIN 27
