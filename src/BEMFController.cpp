#include "BEMFController.h"

BEMFController::BEMFController()
    : _direction(DIR_IDLE),
      _mode(MODE_IDLE),
      _currentPwm(0),
      _driveDurationUs(Config::DEFAULT_DRIVE_DURATION_US),
      _coastDurationUs(Config::DEFAULT_COAST_DURATION_US),
      _drivePhaseStartUs(0),
      _moveStartTimeMs(0),
      _moveDurationMs(0),
      _strokeStartTimeMs(0),
      _targetCycles(0),
      _leftHitsRemaining(0),
      _rightHitsRemaining(0),
      _inReversePause(false),
      _pauseStartTimeMs(0),
      _maxBemfAdcCounts(Config::DEFAULT_MAX_BEMF_ADC_COUNTS),
      _targetSpeed(Config::DEFAULT_TARGET_SPEED),
      _lastCoastRawAvg(0.0f),
      _lastFilteredRawAdc(0.0f),
      _lastMeasuredSpeed(0.0f),
      _lastVoltage(0.0f) {
    _bemfHistory[0] = 0.0f;
    _bemfHistory[1] = 0.0f;
}

void BEMFController::begin() {
    _driver.begin();

    // Configure default PID gains & feed-forward parameters
    _pid.setGains(Config::DEFAULT_KP, Config::DEFAULT_KI, Config::DEFAULT_KD,
                  Config::DEFAULT_KFF, Config::DEFAULT_FFO);
    _pid.setOutputLimits(0.0f, 255.0f, Config::DEFAULT_MIN_ACTIVE_PWM);

    stop();
}

void BEMFController::stop() {
    _driver.stop();
    _direction          = DIR_IDLE;
    _mode               = MODE_IDLE;
    _currentPwm         = 0;
    _inReversePause     = false;
    _leftHitsRemaining  = 0;
    _rightHitsRemaining = 0;
    _bemfHistory[0]     = 0.0f;
    _bemfHistory[1]     = 0.0f;
    _lastCoastRawAvg    = 0.0f;
    _lastFilteredRawAdc = 0.0f;
    _lastMeasuredSpeed  = 0.0f;
    _lastVoltage        = 0.0f;
    _pid.reset();
}

bool BEMFController::startLock() {
    if (_driver.isLeftEndstopHit()) {
        Serial.println(F("[BLOCKED] Lock endstop (Left) already triggered!"));
        return false;
    }
    _mode               = MODE_LOCK;
    _direction          = DIR_LEFT;
    _strokeStartTimeMs  = millis();
    _bemfHistory[0]     = 0.0f;
    _bemfHistory[1]     = 0.0f;
    _lastCoastRawAvg    = 0.0f;
    _lastFilteredRawAdc = 0.0f;
    _lastMeasuredSpeed  = 0.0f;
    _lastVoltage        = 0.0f;
    _pid.reset();

    _currentPwm         = (uint8_t)_pid.getMinActiveOut();
    _driver.drive(_direction, _currentPwm);
    _drivePhaseStartUs  = micros();
    Serial.println(F("[RUN] Locking attachment (heading LEFT)..."));
    return true;
}

bool BEMFController::startUnlock() {
    if (_driver.isRightEndstopHit()) {
        Serial.println(F("[BLOCKED] Unlock endstop (Right) already triggered!"));
        return false;
    }
    _mode               = MODE_UNLOCK;
    _direction          = DIR_RIGHT;
    _strokeStartTimeMs  = millis();
    _bemfHistory[0]     = 0.0f;
    _bemfHistory[1]     = 0.0f;
    _lastCoastRawAvg    = 0.0f;
    _lastFilteredRawAdc = 0.0f;
    _lastMeasuredSpeed  = 0.0f;
    _lastVoltage        = 0.0f;
    _pid.reset();

    _currentPwm         = (uint8_t)_pid.getMinActiveOut();
    _driver.drive(_direction, _currentPwm);
    _drivePhaseStartUs  = micros();
    Serial.println(F("[RUN] Unlocking attachment (heading RIGHT)..."));
    return true;
}

bool BEMFController::startTimed(MotorDirection dir, unsigned long durationMs) {
    if (durationMs == 0) return false;

    if (dir == DIR_LEFT && _driver.isLeftEndstopHit()) {
        Serial.println(F("[BLOCKED] Left endstop already hit!"));
        return false;
    }
    if (dir == DIR_RIGHT && _driver.isRightEndstopHit()) {
        Serial.println(F("[BLOCKED] Right endstop already hit!"));
        return false;
    }

    _mode               = MODE_TIMED;
    _direction          = dir;
    _moveDurationMs     = durationMs;
    _moveStartTimeMs    = millis();
    _bemfHistory[0]     = 0.0f;
    _bemfHistory[1]     = 0.0f;
    _lastCoastRawAvg    = 0.0f;
    _lastFilteredRawAdc = 0.0f;
    _lastMeasuredSpeed  = 0.0f;
    _lastVoltage        = 0.0f;
    _pid.reset();

    _currentPwm         = (uint8_t)_pid.getMinActiveOut();
    _driver.drive(_direction, _currentPwm);
    _drivePhaseStartUs  = micros();

    Serial.print(F("[RUN] Timed move "));
    Serial.print(dir == DIR_LEFT ? F("LEFT (Lock)") : F("RIGHT (Unlock)"));
    Serial.print(F(" for "));
    Serial.print(durationMs);
    Serial.println(F(" ms"));
    return true;
}

bool BEMFController::startCycling(uint16_t count) {
    if (count == 0) {
        Serial.println(F("[ERROR] Cycle count must be > 0."));
        return false;
    }
    _targetCycles       = count;
    _leftHitsRemaining  = count;
    _rightHitsRemaining = count;
    _mode               = MODE_CYCLING;
    _inReversePause     = false;

    // Start towards whichever endstop is not currently hit
    // If at Left (Lock), head Right (Unlock). Otherwise head Left (Lock).
    _direction = _driver.isLeftEndstopHit() ? DIR_RIGHT : DIR_LEFT;

    _strokeStartTimeMs  = millis();
    _bemfHistory[0]     = 0.0f;
    _bemfHistory[1]     = 0.0f;
    _lastCoastRawAvg    = 0.0f;
    _lastFilteredRawAdc = 0.0f;
    _lastMeasuredSpeed  = 0.0f;
    _lastVoltage        = 0.0f;
    _pid.reset();

    _currentPwm         = (uint8_t)_pid.getMinActiveOut();
    _driver.drive(_direction, _currentPwm);
    _drivePhaseStartUs  = micros();

    Serial.print(F("[CYCLE] Starting "));
    Serial.print(count);
    Serial.print(F(" cycles. First heading "));
    Serial.println(_direction == DIR_LEFT ? F("LEFT (Lock)") : F("RIGHT (Unlock)"));
    return true;
}

void BEMFController::executeCoastAndPidStep() {
    // 1. Enter coast
    _driver.coast();
    unsigned long coastStartUs = micros();

    // 2. Blanking period (120 us) to let flyback currents fully decay
    delayMicroseconds(Config::BLANKING_DURATION_US);

    // 3. Multi-sample ADC acquisition inside 200 us window
    // At prescaler 32, each conversion takes ~26 us.
    uint32_t adcSum = 0;
    uint8_t sampleCount = 0;
    unsigned long sampleStartUs = micros();

    while ((micros() - sampleStartUs) < Config::SAMPLE_WINDOW_US && sampleCount < 8) {
        adcSum += _driver.readFastADC();
        sampleCount++;
    }

    _lastCoastRawAvg = (sampleCount > 0) ? ((float)adcSum / (float)sampleCount) : 0.0f;

    // 4. 3-phase moving average: average with the last 2 coast phases
    _lastFilteredRawAdc = (_lastCoastRawAvg + _bemfHistory[0] + _bemfHistory[1]) / 3.0f;
    _bemfHistory[1] = _bemfHistory[0];
    _bemfHistory[0] = _lastCoastRawAvg;

    // Convert to 0-2.8V voltage and 0-255 scaled speed
    _lastVoltage = (_lastFilteredRawAdc * 5.0f) / 1023.0f;
    _lastMeasuredSpeed = (_lastFilteredRawAdc * 255.0f) / _maxBemfAdcCounts;
    if (_lastMeasuredSpeed < 0.0f) _lastMeasuredSpeed = 0.0f;
    if (_lastMeasuredSpeed > 255.0f) _lastMeasuredSpeed = 255.0f;

    // 5. PID Calculation
    // Total cycle time dt = drive time + actual coast duration
    unsigned long actualCoastUs = micros() - coastStartUs;
    if (_coastDurationUs > actualCoastUs) {
        while ((micros() - coastStartUs) < _coastDurationUs) {
            // Wait user-specified extended coast duration
        }
        actualCoastUs = micros() - coastStartUs;
    }
    float dt = (float)(_driveDurationUs + actualCoastUs) / 1000000.0f;
    float computedOut = _pid.compute(_targetSpeed, _lastMeasuredSpeed, dt);
    _currentPwm = (uint8_t)round(computedOut);

    // 6. Safety check: do not re-engage drive if target endstop was reached.
    // Keep in coast and let update() handle the state transition (e.g. reversal in cycling).
    if ((_direction == DIR_LEFT && _driver.isLeftEndstopHit()) ||
        (_direction == DIR_RIGHT && _driver.isRightEndstopHit())) {
        _driver.coast();
        return;
    }

    // 7. Resume drive phase
    _driver.drive(_direction, _currentPwm);
    _drivePhaseStartUs = micros();
}

void BEMFController::update() {
    if (_mode == MODE_IDLE) {
        return;
    }

    unsigned long nowMs = millis();

    // 1. Check Reverse Pause during cycling
    if (_inReversePause) {
        if (nowMs - _pauseStartTimeMs >= Config::REVERSE_PAUSE_MS) {
            _inReversePause     = false;
            _strokeStartTimeMs  = nowMs;
            _bemfHistory[0]     = 0.0f;
            _bemfHistory[1]     = 0.0f;
            _lastCoastRawAvg    = 0.0f;
            _lastFilteredRawAdc = 0.0f;
            _lastMeasuredSpeed  = 0.0f;
            _lastVoltage        = 0.0f;
            _pid.reset();

            _currentPwm         = (uint8_t)_pid.getMinActiveOut();
            _driver.drive(_direction, _currentPwm);
            _drivePhaseStartUs  = micros();
        }
        return;
    }

    // 2. Stroke Safety Timeout
    if (nowMs - _strokeStartTimeMs >= Config::STROKE_TIMEOUT_MS) {
        stop();
        Serial.println(F("[ABORT] Stroke safety timeout reached (20s)! Motor stopped."));
        return;
    }

    // 3. Endstop and Mode checks
    if (_mode == MODE_LOCK) {
        if (_driver.isLeftEndstopHit()) {
            stop();
            Serial.println(F("[DONE] Lock endstop reached! Attachment locked."));
            return;
        }
    } else if (_mode == MODE_UNLOCK) {
        if (_driver.isRightEndstopHit()) {
            stop();
            Serial.println(F("[DONE] Unlock endstop reached! Attachment released."));
            return;
        }
    } else if (_mode == MODE_TIMED) {
        if ((_direction == DIR_LEFT && _driver.isLeftEndstopHit()) ||
            (_direction == DIR_RIGHT && _driver.isRightEndstopHit())) {
            stop();
            Serial.println(F("[ENDSTOP] Hit during timed move! Stopped."));
            return;
        }
        if (nowMs - _moveStartTimeMs >= _moveDurationMs) {
            stop();
            Serial.println(F("[DONE] Timed movement completed."));
            return;
        }
    } else if (_mode == MODE_CYCLING) {
        if (_direction == DIR_LEFT && _driver.isLeftEndstopHit()) {
            if (_leftHitsRemaining > 0) _leftHitsRemaining--;
            Serial.print(F("[CYCLE] Left (Lock) hit! Left remaining: "));
            Serial.print(_leftHitsRemaining);
            Serial.print(F(", Right: "));
            Serial.println(_rightHitsRemaining);

            if (_leftHitsRemaining == 0 && _rightHitsRemaining == 0) {
                stop();
                Serial.println(F("[CYCLE] All cycles completed successfully!"));
                return;
            }
            _driver.coast();
            _inReversePause    = true;
            _pauseStartTimeMs  = nowMs;
            _direction         = DIR_RIGHT;
            return;
        } else if (_direction == DIR_RIGHT && _driver.isRightEndstopHit()) {
            if (_rightHitsRemaining > 0) _rightHitsRemaining--;
            Serial.print(F("[CYCLE] Right (Unlock) hit! Left remaining: "));
            Serial.print(_leftHitsRemaining);
            Serial.print(F(", Right: "));
            Serial.println(_rightHitsRemaining);

            if (_leftHitsRemaining == 0 && _rightHitsRemaining == 0) {
                stop();
                Serial.println(F("[CYCLE] All cycles completed successfully!"));
                return;
            }
            _driver.coast();
            _inReversePause    = true;
            _pauseStartTimeMs  = nowMs;
            _direction         = DIR_LEFT;
            return;
        }
    }

    // 4. Chopping Engine: execute coast sampling & PID when drive period expires
    if (micros() - _drivePhaseStartUs >= _driveDurationUs) {
        executeCoastAndPidStep();
    }
}

void BEMFController::setTargetSpeed(float speed) {
    if (speed >= 0.0f && speed <= 255.0f) {
        _targetSpeed = speed;
    }
}

void BEMFController::setDriveDuration(unsigned long us) {
    if (us >= 200) {
        _driveDurationUs = us;
    }
}

void BEMFController::setCoastDuration(unsigned long us) {
    _coastDurationUs = us;
}

void BEMFController::setMaxBemfCalibration(float counts) {
    if (counts >= 50.0f && counts <= 1023.0f) {
        _maxBemfAdcCounts = counts;
    }
}

void BEMFController::setPidGains(float kp, float ki, float kd, float kff, float ffo) {
    _pid.setGains(kp, ki, kd, kff, ffo);
}

void BEMFController::setFeedForward(float kff) {
    _pid.setFeedForward(kff);
}

void BEMFController::setFeedForwardOffset(float ffo) {
    _pid.setFeedForwardOffset(ffo);
}

void BEMFController::setMinActivePwm(float minPwm) {
    _pid.setOutputLimits(0.0f, 255.0f, minPwm);
}
