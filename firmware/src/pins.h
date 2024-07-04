#pragma once

// GPIOs
#define PIN_LED_GREEN PC15
#define PIN_LED_RED PC14

#define PIN_VMS_IN PA0  // VM-Switch IN

#define PIN_MTR_SA PB7   // Motor's SA (Hall) pin
#define PIN_MTR_BRK PA3  // Motor's BRK pin
#define PIN_MTR_RS PA6   // Motor's RS pin

// Serial communication with host
#define HOST_RX PA10_R
#define HOST_TX PA9_R

// Serial print(f) debugging on proto RX/TX PCB pins
#define DEBUG_RX PA5
#define DEBUG_TX PA4

// old

#define PIN_CURRENT_SENSE 29
#define PIN_TEMP_MOTOR 28
#define PIN_TEMP_PCB 26
#define PIN_VIN 27
