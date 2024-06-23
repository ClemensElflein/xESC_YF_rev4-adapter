// Created by Clemens Elflein on 06/28/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
// Adapted to xESC_YF_r4 by Jörg Ebeling on 2024-06-16
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
//

#include <Arduino.h>
#include <ezLED.h>

#include <numeric>

// #include "hardware/pwm.h"
#include <FastCRC.h>
#include <PacketSerial.h>

#include "config.h"
#include "debug.h"
// #include "hardware/adc.h"
// #include "hardware/dma.h"
// #include "hardware/watchdog.h"*/
#include "pins.h"
#include "xesc_2040_datatypes.h"

#define LED_ERROR_HOST_COMM led_red.blinkNumberOfTimes(20, 50, 1)

// #define WRAP 3299

// The current duty for the motor (-1.0 - 1.0)
volatile float duty = 0.0;
// The absolute value to cap the requested duty to stay within current limits
//(0.0 - 1.0) volatile float current_limit_duty = 0.0;*/
// The requested duty
float duty_setpoint = 0.0;
/*
// init with invalid hall so we have to update
volatile uint last_hall = 0xFF;
volatile uint last_commutation = 0xFF;
// init with out of range duty, so we have to update
volatile float last_duty = 1000.0f;
*/

volatile uint32_t tacho = 0;
volatile uint sa_cycles = 0;  // SA cycles since last status

volatile unsigned int rpm = 0;
volatile uint32_t next_rpm_calc_millis = 0;
volatile unsigned long last_rpm_calc_tacho = 0;

volatile Xesc2040StatusPacket status = {0};
Xesc2040SettingsPacket settings = {0};
bool settings_valid = false;

/*
bool invalid_hall = false;
bool internal_error = false;
float error_i = 0;

unsigned long last_current_control_micros = 0;
unsigned long invalid_hall_start = 0;
*/
volatile unsigned long last_watchdog_millis = 0;
volatile unsigned long last_fault_millis = 0;

// uint8_t analog_round_robin = 0;

HardwareSerial hostSerial(HOST_RX, HOST_TX);
PacketSerial packetSerial;
FastCRC16 CRC16;

ezLED led_green(PIN_LED_GREEN, CTRL_ANODE);
ezLED led_red(PIN_LED_RED, CTRL_ANODE);

HardwareTimer *status_timer;

// volatile uint32_t FrequencyMeasured, LastCapture = 0, CurrentCapture;
// uint32_t input_freq = 0;
// volatile uint32_t rolloverCompareCount = 0;

#ifdef DEBUG_SERIAL
HardwareSerial debugSerial(DEBUG_RX, DEBUG_TX);
#endif

void sendMessage(volatile void *message, size_t size) {
    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of
    if (size < 4) {
        LED_ERROR_HOST_COMM;
        return;
    }
    uint8_t *data_pointer = (uint8_t *)message;

    // calculate the CRC
    uint16_t crc = CRC16.ccitt((uint8_t *)message, size - 2);
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    packetSerial.send((uint8_t *)message, size);
}

/*float readCurrent()
{
  return (float)analogRead(PIN_CURRENT_SENSE) * (3.3f / 4096.0f) /
(CURRENT_SENSE_GAIN * R_SHUNT);
}*/

void updateFaults() {
    uint32_t faults = 0;

    // FAULT_UNINITIALIZED
    if (!settings_valid) {
        faults |= FAULT_UNINITIALIZED;
    }

    // FAULT_WATCHDOG // TODO use STMs IWDG
    if (millis() - last_watchdog_millis > WATCHDOG_TIMEOUT_MILLIS) {
        faults |= FAULT_WATCHDOG;
    }

    // FAULT_UNDERVOLTAGE
    /*if(status.voltage_input < HW_LIMIT_VLOW) {
      faults |= FAULT_UNDERVOLTAGE;
    }

    // FAULT_OVERVOLTAGE
    if(status.voltage_input > HW_LIMIT_VHIGH) {
      faults |= FAULT_OVERVOLTAGE;
    }

    // FAULT_OVERCURRENT
    if(status.current_input > HW_LIMIT_CURRENT) {
      faults |= FAULT_OVERCURRENT;
    }

    // FAULT_OVERTEMP_MOTOR
    //if(settings.has_motor_temp && status.temperature_motor >
  min(HW_LIMIT_MOTOR_TEMP, settings.max_motor_temp)) {
    //  faults |= FAULT_OVERTEMP_MOTOR;
    //}
    // FAULT_OVERTEMP_PCB
    if(status.temperature_pcb > min(HW_LIMIT_PCB_TEMP, settings.max_pcb_temp)) {
      faults |= FAULT_OVERTEMP_PCB;
    }

    // FAULT_INTERNAL_ERROR - this should NEVER be set
    if(internal_error) {
      faults |= FAULT_INTERNAL_ERROR;
    }*/

    if (faults) {
        if (led_red.getState() != LED_BLINKING)  // Boot-up blink sequence FIXME: Ugly
            led_red.blink(500, 500);
        if (led_green.getState() != LED_BLINKING)  // Boot-up blink sequence FIXME: Ugly
            led_green.turnOFF();
        status.fault_code = faults;
        last_fault_millis = millis();
    } else if (faults == 0) {
        if (led_green.getState() != LED_BLINKING)  // SA cycle blink sequence FIXME: Ugly
            led_green.turnON();
        if (status.fault_code != 0) {
            // Reset faults only if MIN_FAULT_TIME_MILLIS passed, or if it was a dedicated watchdog fault
            if (millis() - last_fault_millis > MIN_FAULT_TIME_MILLIS || status.fault_code == FAULT_WATCHDOG) {
                led_red.turnOFF();
                status.fault_code = 0;
            }
        }
    }
}

/**
 * @brief send_status
 * Get called every STATUS_UPDATE_MICROS (by hardware timer)
 */
void send_status() {
    uint sa_cycles_bak = sa_cycles;  // Buffer SA cycles as the next get counted within the next 3ms
    sa_cycles = 0;
    status.seq++;
    tacho += sa_cycles_bak;
    updateFaults();

    if (sa_cycles_bak && led_green.getState() != LED_BLINKING)  // Boot-up blink sequence FIXME: Ugly
        led_green.blinkNumberOfTimes(20, 20, 1);                // FIXME: To fast @ full speed

    /* Some SA cycle infos
     *
     * My stock motor stats:
     *   Nom RPM @ 24VDC = 3488rpm = 4.3ms/SA cycle whereas 4 cycles = 360°
     */
    uint32_t now = millis();
    if (now >= next_rpm_calc_millis) {
        uint32_t diff_millis = RPM_CALC_CYCLE_MILLIS + (now - next_rpm_calc_millis);
        next_rpm_calc_millis = now + RPM_CALC_CYCLE_MILLIS;
        unsigned int diff_tacho = tacho - last_rpm_calc_tacho;  // FIXME: Handle overflow?!
        last_rpm_calc_tacho = tacho;
        rpm = (60000.0f / diff_millis) * diff_tacho / 4;
    }

    DEBUG_PRINTF("now %u ms, SA %i, sa_cycles_bak %u, tacho %u, RPM %u\n", now, digitalRead(PIN_MTR_SA), sa_cycles_bak, tacho, rpm);

    status.duty_cycle = duty;
    status.tacho = tacho;
    status.tacho_absolute = tacho;
    status.direction = 0;

    sendMessage(&status, sizeof(status));
}

/* Handle SA raising edge (called by EXTI)
 * Don't waste CPU here as this ISR might get called every 3ms
 */
void SA_ISR() {
    sa_cycles++;
}

void commitMotorState() {
    float new_duty;
    if (status.fault_code) {
        new_duty = 0.0f;
    } else {
        new_duty = duty_setpoint;
    }
    if (new_duty == duty) {
        return;
    }

    if (new_duty == 0.0f) {               // Stop motor
        digitalWrite(PIN_MTR_RS, HIGH);   // !RS off
        digitalWrite(PIN_MTR_BRK, HIGH);  // TODO: Release BRK after timeout?
        // digitalWrite(PIN_VMS_IN, LOW); // TODO: Switch off once RPM=0?
    } else {                             // Start motor
        digitalWrite(PIN_VMS_IN, HIGH);  // TODO Wire & health check result
        digitalWrite(PIN_MTR_BRK, LOW);
        digitalWrite(PIN_MTR_RS, LOW);  // !RS on
    }
    duty = new_duty;  // FIXME: Move to BRK release or RPM measurement?
}

void onPacketReceived(const uint8_t *buffer, size_t size) {
    // Check, if the packet is valid (1 type byte + 1 data byte + 2 bytes min)
    if (size < 3) {
        LED_ERROR_HOST_COMM;
        DEBUG_PRINTLN("Packet size error");
        return;
    }

    // Check CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);
    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF)) {
        LED_ERROR_HOST_COMM;
        DEBUG_PRINTLN("Packet CRC error");
        return;
    }

    switch (buffer[0]) {
        case XESC2040_MSG_TYPE_CONTROL: {
            if (size != sizeof(struct Xesc2040ControlPacket)) {
                LED_ERROR_HOST_COMM;
                return;
            }
            // Got control packet
            last_watchdog_millis = millis();
            Xesc2040ControlPacket *packet = (Xesc2040ControlPacket *)buffer;
            duty_setpoint = packet->duty_cycle;
            commitMotorState();
        } break;
        case XESC2040_MSG_TYPE_SETTINGS: {
            if (size != sizeof(struct Xesc2040SettingsPacket)) {
                settings_valid = false;
                return;
            }
            Xesc2040SettingsPacket *packet = (Xesc2040SettingsPacket *)buffer;
            settings = *packet;
            settings_valid = true;
        } break;

        default:
            // Wrong/unknown packet ID
            LED_ERROR_HOST_COMM;
            DEBUG_PRINTLN("Packet ID error");
            break;
    }
}

void setup() {
    DEBUG_BEGIN(DEBUG_BAUD);
    DEBUG_PRINTLN("setup()");

    // VM Switch
    pinMode(PIN_VMS_IN, OUTPUT);
    // digitalWrite(PIN_VMS_IN, LOW);
    digitalWrite(PIN_VMS_IN, HIGH);  // TODO Wire & health check

    // Motor
    pinMode(PIN_MTR_BRK, OUTPUT);
    digitalWrite(PIN_MTR_BRK, LOW);
    pinMode(PIN_MTR_RS, OUTPUT);
    digitalWrite(PIN_MTR_RS, HIGH);

    // Tacho get handled with ISR on PIN_MTR_SA
    pinMode(PIN_MTR_SA, INPUT_FLOATING);
    attachInterrupt(PIN_MTR_SA, SA_ISR, RISING);

    // We've hardware timer on mass, no need to count millis() by hand
    status_timer = new HardwareTimer(TIMER_STATUS);
    status_timer->setOverflow(STATUS_UPDATE_MICROS, MICROSEC_FORMAT);
    status_timer->attachInterrupt(send_status);
    status_timer->resume();

    status.message_type = XESC2040_MSG_TYPE_STATUS;
    status.fw_version_major = 0;
    status.fw_version_minor = 10;

    // Already initialized

    // --- old

    // last_commutation = 0xFF;
    // analogReadResolution(12);

    // Comms UART
    PACKET_SERIAL.begin(115200);
    packetSerial.setStream(&PACKET_SERIAL);
    packetSerial.setPacketHandler(&onPacketReceived);

    // LED blink code "Boot up successful"
    led_green.blinkNumberOfTimes(200, 200, 3);  // 50ms ON, 200ms OFF, repeat 3 times, blink immediately
    led_red.blinkNumberOfTimes(200, 200, 3);    // 50ms ON, 200ms OFF, repeat 3 times, blink immediately
}

/*void loop1()
{
  if (duty_setpoint_ramped > current_limit_duty)
  {

    //digitalWrite(PIN_LED_GREEN, HIGH);
    duty = current_limit_duty;
  }
  else if (duty_setpoint_ramped < -current_limit_duty)
  {

    //digitalWrite(PIN_LED_GREEN, HIGH);
    duty = -current_limit_duty;
  }
  else
  {
    //digitalWrite(PIN_LED_GREEN, LOW);
    duty = duty_setpoint_ramped;
  }

  unsigned long now = micros();
  if (now - last_current_control_micros > 500)
  {
    float dt = (now - last_current_control_micros) / 1000000.0f;

    float error = settings.motor_current_limit - status.current_input;
    if (duty_setpoint_ramped == 0.0f)
    {
      error_i = 0.0f;
    }
    else if (abs(current_limit_duty) < duty_setpoint_ramped)
    {
      error_i += error * dt;
    }
    current_limit_duty = CURRENT_P * error + CURRENT_I * error_i;
    current_limit_duty = constrain(current_limit_duty, 0.0f, MAX_DUTY_CYCLE);

    last_current_control_micros = now;
  }
  updateCommutation();
}*/

/*float readVIN()
{
  return (float)analogRead(PIN_VIN) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) /
VIN_R2);
}
*/

void loop() {
    led_green.loop();
    led_red.loop();
    commitMotorState();

    // Green LED
    /*if (status.fault_code != 0 && duty == 0.0f) {
      led_green.turnON();
    } else {
      led_green
    }*/

    /*
  switch (analog_round_robin)
  {
  case 0:
    status.current_input = readCurrent() * 0.0008 + status.current_input * 0.9992;
    break;
  case 1:
    status.voltage_input = readVIN() * 0.001 + status.voltage_input * 0.999;
    break;
  case 2:
    status.temperature_motor = readMotorTemp() * 0.001 + status.temperature_motor
  * 0.999; break; case 3: status.temperature_pcb = readPcbTemp() * 0.001 +
  status.temperature_pcb * 0.999; break;

  default:
    break;
  }

  analog_round_robin = (analog_round_robin + 1) % 4;
  */

    packetSerial.update();
}
