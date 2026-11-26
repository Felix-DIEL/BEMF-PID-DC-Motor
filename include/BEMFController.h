#pragma once
#include <Arduino.h>
#include "Config.h"
#include "PIDController.h"
#include "MotorDriver.h"

/**
 * ==============================================================================
 * BEMFController - Back-EMF Chopping Motor Speed Regulator & Motion State Machine
 * ==============================================================================
 */
class BEMFController {
public:
    BEMFController();

    /**
     * Initializes the motor driver, PID controller, and calibration defaults.
     */
    void begin();

    /**
     * Executes the motion state machine and chopping engine. Must be called in loop().
     */
    void update();

    // --- Action Commands ---
    bool startLock();
    bool startUnlock();
    bool startTimed(MotorDirection dir, unsigned long durationMs);
    bool startCycling(uint16_t count);
    void stop();

    // --- Configuration & Tuning Setters ---
    void setTargetSpeed(float speed);
    void setDriveDuration(unsigned long us);
    void setCoastDuration(unsigned long us);
    void setMaxBemfCalibration(float counts);
    void setPidGains(float kp, float ki, float kd, float kff = -1.0f, float ffo = -1.0f);
    void setFeedForward(float kff);
    void setFeedForwardOffset(float ffo);
    void setMinActivePwm(float minPwm);

    // --- Status & Telemetry Getters ---
    MotionMode     getMode() const              { return _mode; }
    MotorDirection getDirection() const         { return _direction; }
    uint8_t        getCurrentPwm() const        { return _currentPwm; }
    float          getTargetSpeed() const       { return _targetSpeed; }
    float          getMeasuredSpeed() const     { return _lastMeasuredSpeed; }
    float          getVoltage() const           { return _lastVoltage; }
    float          getFilteredRawAdc() const    { return _lastFilteredRawAdc; }
    float          getMaxBemfCalibration() const{ return _maxBemfAdcCounts; }
    unsigned long  getDriveDuration() const     { return _driveDurationUs; }
    unsigned long  getCoastDuration() const     { return _coastDurationUs; }

    uint16_t       getLeftHitsRemaining() const { return _leftHitsRemaining; }
    uint16_t       getRightHitsRemaining() const{ return _rightHitsRemaining; }
    uint16_t       getTargetCycles() const      { return _targetCycles; }

    const PIDController& getPid() const         { return _pid; }
    MotorDriver&         getDriver()            { return _driver; }

private:
    void executeCoastAndPidStep();

    MotorDriver   _driver;
    PIDController _pid;

    MotorDirection _direction;
    MotionMode     _mode;
    uint8_t        _currentPwm;

    // Timing parameters
    unsigned long  _driveDurationUs;
    unsigned long  _coastDurationUs;
    unsigned long  _drivePhaseStartUs;
    unsigned long  _moveStartTimeMs;
    unsigned long  _moveDurationMs;
    unsigned long  _strokeStartTimeMs;

    // Cycling parameters
    uint16_t       _targetCycles;
    uint16_t       _leftHitsRemaining;
    uint16_t       _rightHitsRemaining;
    bool           _inReversePause;
    unsigned long  _pauseStartTimeMs;

    // BEMF sensing history & outputs
    float          _maxBemfAdcCounts;
    float          _targetSpeed;
    float          _bemfHistory[2];
    float          _lastCoastRawAvg;
    float          _lastFilteredRawAdc;
    float          _lastMeasuredSpeed;
    float          _lastVoltage;
};
