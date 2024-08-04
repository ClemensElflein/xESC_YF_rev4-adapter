#pragma once

// Enable for debug logging on proto UART2
#define PROTO_DEBUG
#define PROTO_DEBUG_BAUD 460800_Bd
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG  // ERROR, WARNING, INFO, DEUG

#define STATUS_CYCLE 20ms
#define NUM_STATUS_CYCLES_MOTOR_STOPPED 4    // Number of STATUS_CYCLES before motor get declared as stopped (if tacho doesn't changed)
#define WATCHDOG_TIMEOUT_MILLIS 500

// Time to keep motor controller in fault once a fault occurs
#define MIN_FAULT_TIME_MILLIS 2000

/* SA (Hall) Capture-Compare-Timer, RPM-Calc
 * RPM = 60 / ( (1/TimClock) * TimPrescaler * (CapCompTicks * 4 (Cycles/360°) / InputPrescaler) ) =>
 * RPM = (15 * TimClock * InputPrescaler) / (TimPrescaler * CapCompTicks)
 */
#define SA_TIMER_INPUT_PRESCALER 1
#define SA_TIMER_PRESCALER 120
#define SA_TIMER_MIN_TICKS 500  // Minimum possible tick size

// ----- old ----

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
