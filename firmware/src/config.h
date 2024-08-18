#pragma once

// #define PROTO_DEBUG          // Enable for debug logging on proto UART2
#define PROTO_DEBUG_BAUD 460800_Bd
// #define PROTO_DEBUG_MOTOR    // Enable motor specific debugging messages
// #define PROTO_DEBUG_HOST_RX  // Enable host RX debugging messages
// #define PROTO_DEBUG_HOST_TX  // Enable host TX debugging messages
// #define PROTO_DEBUG_ADC      // Enable ADC specific debugging messages

#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG  // ERROR, WARNING, INFO, DEUG

#define STATUS_CYCLE 20ms
#define NUM_STATUS_CYCLES_MOTOR_STOPPED 4  // Number of STATUS_CYCLES before motor get declared as stopped (if tacho doesn't changed)
#define WATCHDOG_TIMEOUT_MILLIS 500

// Time to keep motor controller in fault once a fault occurs
#define MIN_FAULT_TIME_MILLIS 2000

/* SA (Hall) Capture-Compare-Timer, RPM-Calc
 * RPM = 60 / ( (1/TimClock) * TimPrescaler * (CapCompTicks * 4 (Cycles/360°) / InputPrescaler) ) =>
 * RPM = (15 * TimClock * InputPrescaler) / (TimPrescaler * CapCompTicks)
 */
#define SA_TIMER_INPUT_PRESCALER 1
#define SA_TIMER_PRESCALER 120
#define SA_TIMER_MIN_TICKS 50   // Minimum possible tick size

// ADC
#define ADC_RESOLUTION Resolution::Bits12
#define ADC_NUM_CODES 4096U
#define CUR_SENSE_GAIN 40.0f
#define CUR_SENSE_2_GAIN 20.0f
#define R_SHUNT 0.075f  // Shunt resistor

// ----- old ----

// Hardware limits before going into fault
#define HW_LIMIT_PCB_TEMP 80.0

// Max amps before shutting down
#define HW_LIMIT_CURRENT 1.5

// Max duty cycle to allow. This can't be 1, because the gate driver doesn't allow that for some reason.
#define MAX_DUTY_CYCLE 0.95f
