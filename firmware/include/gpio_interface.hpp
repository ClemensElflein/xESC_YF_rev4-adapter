// Created by Jörg Ebeling on 2025-10-17.
// Copyright (c) 2025 Jörg Ebeling for OpenMower/Clemens Elflein. All rights reserved.
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

#pragma once

#include <modm/platform.hpp>

using namespace modm::platform;

// GPIO Interface for runtime-configurable GPIO operations.
//
// Implements the Strategy Pattern to allow runtime switching between
// different GPIO implementations without template complexity.
class IGpio {
public:
    virtual ~IGpio() = default;

    virtual void SetOutput(const Gpio::OutputType type) = 0;
    virtual void SetInput(const Gpio::InputType type) = 0;
    virtual void Set() = 0;
    virtual void Reset() = 0;
    virtual void Toggle() = 0;
    virtual bool Read() = 0;
};

// Concrete MODM GPIO implementation.
//
// Template Parameters:
//   Pin: MODM GPIO pin type (e.g., GpioOutputA5)
template <typename Pin>
class ModmGpio : public IGpio {
public:
    void SetOutput(Gpio::OutputType type) override { Pin::setOutput(type); }
    void SetInput(Gpio::InputType type) override { Pin::setInput(type); }
    void Set() override { Pin::set(); }
    void Reset() override { Pin::reset(); }
    void Toggle() override { Pin::toggle(); }
    bool Read() override { return Pin::read(); }
};
