#pragma once

#define WATCHDOG_TIMEOUT_MILLIS 500

// Time to keep motor controller in fault once a fault occurs
#define MIN_FAULT_TIME_MILLIS 2000

#define NUM_SA_CYCLES_PER_TURN 4
#define RPM_CALC_CYCLE_MILLIS 200  // How often to calc RPMs. Need to be high enough to catch enough SA cycles

// Timer
#define TIMER_STATUS TIM14

// Analog read resolution
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

// ADC - ATEMP
#define CALX_TEMP 30
#define VTEMP 760
#define AVG_SLOPE 2530

// old

// Hardware limits before going into fault
#define HW_LIMIT_PCB_TEMP 80.0
#define HW_LIMIT_MOTOR_TEMP 80.0

// Max amps before shutting down
#define HW_LIMIT_CURRENT 1.5

// P and I values for the current control
#define CURRENT_P 50.0f
#define CURRENT_I 150.1f

// Hardware limit for undervoltage and overvoltage.
#define HW_LIMIT_VLOW 12.0
#define HW_LIMIT_VHIGH 40.0

// Values for the voltage sensing resistor divider
#define VIN_R1 22000.0f
#define VIN_R2 1500.0f

// Value for the shunt resistor
#define R_SHUNT 0.033f

// Gain for the current sense amplifier
#define CURRENT_SENSE_GAIN 200.0f

// Max duty cycle to allow. This can't be 1, because the gate driver doesn't allow that for some reason.
#define MAX_DUTY_CYCLE 0.95f
