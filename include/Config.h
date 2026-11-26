#pragma once
#include <Arduino.h>

/**
 * ==============================================================================
 * DC Motor Back-EMF Closed-Loop PID Controller - Configuration & Pin Definitions
 * Board: Arduino Uno (ATmega328P @ 16 MHz)
 * ==============================================================================
 */

namespace Config {

    // --- GPIO Pin Definitions ---
    constexpr uint8_t PIN_MOTOR_IN1  = 9;   // DRV8871 IN1 (Timer1 OC1A, 31.25 kHz PWM)
    constexpr uint8_t PIN_MOTOR_IN2  = 10;  // DRV8871 IN2 (Timer1 OC1B, 31.25 kHz PWM)
    constexpr uint8_t PIN_HALL_LEFT  = 2;   // Lock Endstop (AH1806 Hall switch)
    constexpr uint8_t PIN_HALL_RIGHT = 3;   // Unlock Endstop (AH1806 Hall switch)
    constexpr uint8_t PIN_BEMF_SYNC  = 4;   // Scope Trigger / Coast indicator (HIGH during coast)
    constexpr uint8_t PIN_BEMF_ADC   = A0;  // Back-EMF Analog Input

    // AH1806 open-drain digital switch pulls LOW when a magnetic field is present
    constexpr uint8_t ENDSTOP_TRIGGERED = LOW;

    // --- Timing Parameters (Microseconds) ---
    constexpr unsigned long DEFAULT_DRIVE_DURATION_US = 5000; // 5.0 ms active drive phase
    constexpr unsigned long DEFAULT_COAST_DURATION_US = 0;    // 0 = Auto / immediate resume (~320 us)
    constexpr unsigned long BLANKING_DURATION_US      = 120;  // 120 us flyback current blanking
    constexpr unsigned long SAMPLE_WINDOW_US          = 200;  // 200 us multi-sample ADC window
    constexpr unsigned long REVERSE_PAUSE_MS          = 120;  // 120 ms reverse pause to protect gearbox
    constexpr unsigned long STROKE_TIMEOUT_MS         = 20000;// 20s safety timeout per stroke

    // --- Glitch Filter Parameters ---
    constexpr uint8_t ENDSTOP_FILTER_SAMPLES          = 4;    // Consecutive checks required
    constexpr unsigned int ENDSTOP_FILTER_DELAY_US    = 50;   // 4 * 50 us = 200 us sustained signal

    // --- Speed Scaling & ADC Calibration ---
    // With 5.0V AREF -> 1023 counts. 2.8V max BEMF corresponds to (2.8 / 5.0) * 1023 = 573 counts.
    constexpr float DEFAULT_MAX_BEMF_ADC_COUNTS       = 573.0f;
    constexpr float DEFAULT_TARGET_SPEED              = 140.0f; // 0 - 255 scaled speed

    // --- Feed-Forward & PID Default Gains (Fitted to Geared DC Motor) ---
    constexpr float DEFAULT_KP                        = 2.5f;
    constexpr float DEFAULT_KI                        = 2.5f;
    constexpr float DEFAULT_KD                        = 0.0f;   // 0 to prevent commutator ripple spikes
    constexpr float DEFAULT_KFF                       = 0.54f;  // Feed-forward slope
    constexpr float DEFAULT_FFO                       = 158.0f; // Friction overcome base offset PWM
    constexpr float DEFAULT_MIN_ACTIVE_PWM            = 120.0f; // Minimum PWM threshold when active

    // --- Serial Interface ---
    constexpr unsigned long SERIAL_BAUD_RATE          = 115200;
    constexpr unsigned long PLOTTER_INTERVAL_MS       = 20;    // 50 Hz telemetry update rate
    constexpr size_t CLI_RX_BUFFER_SIZE               = 48;
}

// --- Direction & Motion Mode Enums ---
enum MotorDirection : uint8_t {
    DIR_IDLE,
    DIR_LEFT,   // LOCK direction (runs towards Left Endstop, Pin 2)
    DIR_RIGHT   // UNLOCK direction (runs towards Right Endstop, Pin 3)
};

enum MotionMode : uint8_t {
    MODE_IDLE,
    MODE_LOCK,      // Drive Left until Lock Endstop triggers
    MODE_UNLOCK,    // Drive Right until Unlock Endstop triggers
    MODE_TIMED,     // Drive Left or Right for specified milliseconds
    MODE_CYCLING    // Cycle between Lock & Unlock for N iterations
};
