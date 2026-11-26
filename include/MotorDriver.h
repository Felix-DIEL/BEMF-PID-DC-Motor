#pragma once
#include <Arduino.h>
#include "Config.h"

/**
 * ==============================================================================
 * MotorDriver - Hardware Abstraction for DRV8871 H-Bridge, Timer1 PWM & ADC
 * ==============================================================================
 */
class MotorDriver {
public:
    MotorDriver() = default;

    /**
     * Initializes Arduino Uno peripherals:
     * - Configures Motor & Sensor GPIOs
     * - Sets Timer1 to 31.25 kHz Phase-Correct PWM
     * - Configures ADC for high-speed sampling (Prescaler 32 = 500 kHz clock)
     */
    void begin();

    /**
     * Drives the motor in the specified direction at the given PWM duty cycle (0-255).
     * @param dir DIR_LEFT (Lock) or DIR_RIGHT (Unlock)
     * @param pwm Duty cycle (0 to 255)
     */
    void drive(MotorDirection dir, uint8_t pwm);

    /**
     * Enters High-Z coast mode (both DRV8871 inputs LOW).
     * Raises PIN_BEMF_SYNC HIGH for oscilloscope triggering.
     */
    void coast();

    /**
     * Completely stops motor drive, ensures High-Z coast and drops PIN_BEMF_SYNC LOW.
     */
    void stop();

    /**
     * Sets the oscilloscope sync / diagnostic trigger pin state.
     */
    void setScopeSync(bool high);

    /**
     * Executes a fast ADC conversion on Pin A0 (ADC0).
     * With prescaler 32 @ 16 MHz, conversion takes ~26 us.
     * @return 10-bit raw ADC reading (0 to 1023)
     */
    uint16_t readFastADC();

    /**
     * Checks if the Left Hall Endstop (Lock) is triggered with EMI glitch filtering.
     * @return true if magnetic field is sustained for at least 200 us
     */
    bool isLeftEndstopHit();

    /**
     * Checks if the Right Hall Endstop (Unlock) is triggered with EMI glitch filtering.
     * @return true if magnetic field is sustained for at least 200 us
     */
    bool isRightEndstopHit();
};
