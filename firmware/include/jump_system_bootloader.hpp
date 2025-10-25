// Created by Apehaenger on 2024-08-20.
//
// This file is part of the openmower project.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based
// on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#pragma once

#include <modm/platform.hpp>

#include "AdcSampler.hpp"
#include "board.hpp"
#include "config.h"
#include "led_controller.hpp"

using namespace Board;
using namespace std::chrono_literals;

void jump_system_bootloader() {
  void (*SysMemBootJump)(void);

  // Disable Timer & interrupts
  Timer14::disableInterrupt(Timer14::Interrupt::Update);
  Timer14::disable();
  Timer1::disableInterrupt(Timer1::Interrupt::CaptureCompare4);
  Timer1::disable();
  AdcSampler::disable();

  __disable_irq();    // Disable all interrupts
  SysTick->CTRL = 0;  // Disable Systick timer
  // HAL_RCC_DeInit();// Set the clock to the default state
  Rcc::setHsiSysDivider(Rcc::HsiSysDivider::Div4);  // Default HsiSysDivider = boot frequency (12MHz)

  // Clear Interrupt Enable Register & Interrupt Pending Register
  for (size_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  __enable_irq();  // Re-enable all interrupts

  // Set up the jump to boot loader address + 4
  SysMemBootJump = (void (*)(void))(*((uint32_t*)((BOOTLOADER_ADDR + 4))));

  // Set the main stack pointer to the boot loader stack
  __set_MSP(*(uint32_t*)BOOTLOADER_ADDR);

  // Call the function to jump to boot loader location
  SysMemBootJump();

  // Jump is done successfully, we should never reach here!
}
