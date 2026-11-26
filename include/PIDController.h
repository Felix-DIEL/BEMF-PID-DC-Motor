#pragma once
#include <Arduino.h>

/**
 * ==============================================================================
 * PIDController - Discrete PID with Feed-Forward (Offset + Slope) & Anti-Windup
 * ==============================================================================
 * 
 * Features:
 * 1. Feed-Forward (Friction Offset + Speed Slope):
 *    FF = Offset + (Kff * Setpoint)
 *    Provides immediate baseline PWM to overcome gearbox stiction and back-EMF,
 *    eliminating the slow ramp-up delay of pure integral control.
 * 
 * 2. Proportional Term:
 *    P = Kp * Error
 * 
 * 3. Conditional Anti-Windup Integral:
 *    I += Ki * Error * dt
 *    Integration halts when the control output is already saturated at limits.
 * 
 * 4. Filtered Derivative on Measurement:
 *    D = -Kd * lowPass((Input - prevInput) / dt)
 *    Computed on feedback measurement instead of error to eliminate setpoint kick,
 *    and filtered with an exponential moving average (alpha) to ignore brush ripple.
 * 
 * 5. Minimum Active Output (Deadband Jump):
 *    Ensures that whenever drive is non-zero, it never commands below minActiveOut.
 */
class PIDController {
public:
    PIDController() 
        : _kp(1.0f), _ki(0.1f), _kd(0.01f), _kff(0.0f), _ffOffset(0.0f),
          _minOut(0.0f), _maxOut(255.0f), _minActiveOut(30.0f),
          _integral(0.0f), _prevError(0.0f), _prevInput(0.0f),
          _filteredD(0.0f), _dFilterAlpha(0.25f) {}

    /**
     * Configures primary gains.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param kff Feed-forward slope (optional, ignored if negative)
     * @param ffOffset Feed-forward base offset PWM (optional, ignored if negative)
     */
    void setGains(float kp, float ki, float kd, float kff = -1.0f, float ffOffset = -1.0f) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        if (kff >= 0.0f) {
            _kff = kff;
        }
        if (ffOffset >= 0.0f) {
            _ffOffset = ffOffset;
        }
    }

    void setFeedForward(float kff) {
        if (kff >= 0.0f) {
            _kff = kff;
        }
    }

    void setFeedForwardOffset(float ffOffset) {
        if (ffOffset >= 0.0f) {
            _ffOffset = ffOffset;
        }
    }

    void setOutputLimits(float minOut, float maxOut, float minActiveOut) {
        _minOut = minOut;
        _maxOut = maxOut;
        _minActiveOut = minActiveOut;
    }

    void setDFilter(float alpha) {
        if (alpha > 0.0f && alpha <= 1.0f) {
            _dFilterAlpha = alpha;
        }
    }

    /**
     * Resets integral accumulator, derivative history, and previous error.
     */
    void reset() {
        _integral = 0.0f;
        _prevError = 0.0f;
        _prevInput = 0.0f;
        _filteredD = 0.0f;
    }

    /**
     * Calculates the PID + Feed-Forward output for the current time step.
     * @param setpoint Target speed
     * @param input Current filtered speed
     * @param dt Delta time in seconds
     * @return Output PWM (clamped between minOut and maxOut)
     */
    float compute(float setpoint, float input, float dt) {
        if (dt <= 0.0f) return _minOut;

        float error = setpoint - input;

        // 1. Feed-forward term provides immediate base PWM (friction offset + slope * setpoint)
        float ffTerm = (setpoint > 0.0f) ? (_ffOffset + (_kff * setpoint)) : 0.0f;

        // 2. Proportional term
        float pTerm = _kp * error;

        // 3. Derivative term computed on measurement (to prevent derivative kick)
        float dInput = (input - _prevInput) / dt;
        _filteredD = (_dFilterAlpha * dInput) + ((1.0f - _dFilterAlpha) * _filteredD);
        float dTerm = -(_kd * _filteredD);

        // 4. Pre-calculate output before updating integral for conditional anti-windup
        float preOutput = ffTerm + pTerm + _integral + dTerm;
        if (!((preOutput >= _maxOut && error > 0.0f) || (preOutput <= _minOut && error < 0.0f))) {
            _integral += _ki * error * dt;
            if (_integral > _maxOut) _integral = _maxOut;
            else if (_integral < -_maxOut) _integral = -_maxOut;
        }

        _prevError = error;
        _prevInput = input;

        float output = ffTerm + pTerm + _integral + dTerm;

        // 5. Apply output limits
        if (output > _maxOut) output = _maxOut;
        if (output < _minOut) output = _minOut;

        // 6. Overcome gearbox stiction: if output > 0, clamp to minimum active threshold
        if (output > 0.0f && output < _minActiveOut) {
            output = _minActiveOut;
        }

        return output;
    }

    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }
    float getKff() const { return _kff; }
    float getFfOffset() const { return _ffOffset; }
    float getIntegral() const { return _integral; }
    float getMinActiveOut() const { return _minActiveOut; }

private:
    float _kp;
    float _ki;
    float _kd;
    float _kff;
    float _ffOffset;
    float _minOut;
    float _maxOut;
    float _minActiveOut;
    float _integral;
    float _prevError;
    float _prevInput;
    float _filteredD;
    float _dFilterAlpha;
};
