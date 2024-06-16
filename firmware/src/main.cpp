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

#include "debug.h"
// #include "hardware/pwm.h"
#include <FastCRC.h>
#include <PacketSerial.h>

#include "config.h"
// #include "hardware/adc.h"
// #include "hardware/dma.h"
// #include "hardware/watchdog.h"*/
#include "pins.h"
#include "xesc_2040_datatypes.h"

#define LED_ERROR_HOST_COMM led_red.blink(50, 100)

/*#define WRAP 3299

// The current duty for the motor (-1.0 - 1.0)
volatile float duty = 0.0;
// The absolute value to cap the requested duty to stay within current limits
(0.0 - 1.0) volatile float current_limit_duty = 0.0;
// The requested duty
volatile float duty_setpoint = 0.0;
// The ramped duty. This will ramp to avoid current spikes
volatile float duty_setpoint_ramped = 0.0;

// init with invalid hall so we have to update
volatile uint last_hall = 0xFF;
volatile uint last_commutation = 0xFF;
// init with out of range duty, so we have to update
volatile float last_duty = 1000.0f;
*/

volatile uint32_t tacho = 0;
volatile uint32_t tacho_absolute = 0;  // wheel ticks absolute
volatile bool direction;               // direction CW/CCW

volatile Xesc2040StatusPacket status = {0};
Xesc2040SettingsPacket settings = {0};
bool settings_valid = false;

/*
bool invalid_hall = false;
bool internal_error = false;
float error_i = 0;

unsigned long last_current_control_micros = 0;
unsigned long invalid_hall_start = 0;
unsigned long last_ramp_update_millis = 0;
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

#ifdef SERIAL_DEBUG
HardwareSerial debugSerial(DEBUG_RX, DEBUG_TX);
#endif

void onPacketReceived(const uint8_t *buffer, size_t size);

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

void setCommutation(uint8_t step) {
    /*if(duty == 0.0f) {
      gpio_put_masked(0b111 << 8, 0b111 << 8);
      pwm_set_gpio_level(PIN_UH, 0);
      pwm_set_gpio_level(PIN_VH, 0);
      pwm_set_gpio_level(PIN_WH, 0);
      return;
    }*/
}

void updateCommutation() {
    /*
    // check, if halls or duty changed. if not, don't update anything
    if (last_duty == duty && last_hall == hall_sensors)
      return;


    // If fault, turn off motor
    if (status.fault_code || !settings_valid)
    {
      setCommutation(255);
    } else {
      setCommutation(commutation);
    }



    if(last_hall != hall_sensors && last_commutation != 0xFF) {
      // we got a hall tick, check direction and update tacho
      int8_t diff = (last_commutation-1) - (commutation-1);
      if(diff < -3) {
        diff += 6;
      } if(diff > 3) {
        diff -= 6;
      }
      if(diff >= -3 && diff <= 3) {
        tacho += diff;
        tacho_absolute += abs(diff);
        direction = diff < 0;
      }
      else
      {
        internal_error = true;
      }
    }
    last_commutation = commutation;
    last_hall = hall_sensors;
    last_duty = duty; */
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
        // We have faults, set them in the status
        if (led_red.getState() != LED_BLINKING) led_red.blink(500, 500);
        status.fault_code = faults;
        last_fault_millis = millis();
    } else if (faults == 0 && status.fault_code != 0) {
        // Reset faults only if MIN_FAULT_TIME_MILLIS passed, or if it was a dedicated watchdog fault
        if (millis() - last_fault_millis > MIN_FAULT_TIME_MILLIS || status.fault_code == FAULT_WATCHDOG) {
            led_red.turnOFF();
            status.fault_code = 0;
        }
    }
}

void send_status() {
    status.seq++;
    updateFaults();
    // status.duty_cycle = duty;
    status.tacho = tacho;
    status.tacho_absolute = tacho_absolute;
    status.direction = direction;
    sendMessage(&status, sizeof(status));
}

void setup() {
    DEBUG_BEGIN(9600);
    DEBUG_PRINTLN("setup()");

#ifdef HW_RMECOWV100  // Dev ButtonBoard while waiting for PCB
    // Disable ESP-WROOM-02 on Dev ButtonBoard
    pinMode(PF7, OUTPUT);
    digitalWrite(PF7, LOW);  // EN = off
    delay(500);

    // Test alt comm port for dev
    // hostSerial.begin(9600);
    // delay(500);
    // hostSerial.write("Hello World", 5);
#endif

    // We've hardware timer on mass, no need to count millis() by hand
    status_timer = new HardwareTimer(TIMER_STATUS);
    status_timer->setOverflow(STATUS_UPDATE_MICROS, MICROSEC_FORMAT);
    status_timer->attachInterrupt(send_status);
    status_timer->resume();

    status.message_type = XESC2040_MSG_TYPE_STATUS;
    status.fw_version_major = 0;
    status.fw_version_minor = 10;
    // Already initialized
    // tacho = 0;
    // tacho_absolute = 0;
    // settings_valid = false;

    // --- old

    /*

    last_commutation = 0xFF;*/

    // Setup Pin directions
    /*pinMode(PIN_UH, OUTPUT);
    pinMode(PIN_VH, OUTPUT);

    analogReadResolution(12);*/

    PACKET_SERIAL.begin(115200);
    packetSerial.setStream(&PACKET_SERIAL);
    packetSerial.setPacketHandler(&onPacketReceived);

    // LED blink code "Boot up successful"
    led_green.blinkNumberOfTimes(100, 200, 3);  // 50ms ON, 200ms OFF, repeat 3 times, blink immediately
    led_red.blinkNumberOfTimes(100, 200, 3);    // 50ms ON, 200ms OFF, repeat 3 times, blink immediately
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

void onPacketReceived(const uint8_t *buffer, size_t size) {
    // Check, if the packet is valid (1 type byte + 1 data byte + 2 bytes min)
    if (size < 3) {
        LED_ERROR_HOST_COMM;
        return;
    }

    // Check CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);
    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF)) {
        LED_ERROR_HOST_COMM;
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
            // TODO!!!
            // duty_setpoint = constrain(packet->duty_cycle, -MAX_DUTY_CYCLE, MAX_DUTY_CYCLE);
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
            break;
    }
}

void loop() {
    led_green.loop();
    led_red.loop();

    /*

  // We need to ramp the duty cycle
  if (millis() - last_ramp_update_millis > 10)
  {
    float dt = (float)(millis() - last_ramp_update_millis) / 1000.0f;
    last_ramp_update_millis = millis();
    if(status.fault_code) {
      duty_setpoint_ramped = 0.0;
    } else if (duty_setpoint != duty_setpoint_ramped)
    {
      if (duty_setpoint > duty_setpoint_ramped)
      {
        duty_setpoint_ramped = min(duty_setpoint, duty_setpoint_ramped +
  settings.acceleration * dt);
      }
      else
      {
        duty_setpoint_ramped = max(duty_setpoint, duty_setpoint_ramped -
  settings.acceleration * dt);
      }
    }
  }

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
