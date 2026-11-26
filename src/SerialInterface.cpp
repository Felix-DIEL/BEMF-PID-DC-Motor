#include "SerialInterface.h"

SerialInterface::SerialInterface(BEMFController& controller)
    : _controller(controller),
      _plotterMode(false),
      _lastPlotMs(0),
      _lastPlottedDirection(DIR_IDLE),
      _rxIndex(0) {
    _rxBuffer[0] = '\0';
}

void SerialInterface::begin(unsigned long baud) {
    Serial.begin(baud);
    while (!Serial && millis() < 2000);

    Serial.println();
    printHelp();
    printStatus();
}

void SerialInterface::update() {
    // 1. Read incoming serial bytes
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (_rxIndex > 0) {
                _rxBuffer[_rxIndex] = '\0';
                processCommand(_rxBuffer);
                _rxIndex = 0;
            }
        } else {
            if (_rxIndex < Config::CLI_RX_BUFFER_SIZE - 1) {
                _rxBuffer[_rxIndex++] = c;
            }
        }
    }

    // 2. Stream plotter telemetry when active
    streamPlotter();
}

void SerialInterface::streamPlotter() {
    if (!_plotterMode) {
        _lastPlottedDirection = DIR_IDLE;
        return;
    }

    MotorDirection currentDir = _controller.getDirection();
    if (currentDir != _lastPlottedDirection) {
        if (currentDir == DIR_LEFT) {
            Serial.println(F("Dir: LEFT (Lock)"));
        } else if (currentDir == DIR_RIGHT) {
            Serial.println(F("Dir: RIGHT (Unlock)"));
        }
        _lastPlottedDirection = currentDir;
    }

    if (_controller.getMode() != MODE_IDLE) {
        unsigned long now = millis();
        if (now - _lastPlotMs >= Config::PLOTTER_INTERVAL_MS) {
            _lastPlotMs = now;
            // Format: Target:val,Measured:val,PWM:val,Volts:val (Arduino Serial Plotter & human-readable)
            Serial.print(F("Target:"));
            Serial.print(_controller.getTargetSpeed(), 1);
            Serial.print(F(",Measured:"));
            Serial.print(_controller.getMeasuredSpeed(), 1);
            Serial.print(F(",PWM:"));
            Serial.print(_controller.getCurrentPwm());
            Serial.print(F(",Volts:"));
            Serial.println(_controller.getVoltage(), 2);
        }
    }
}

void SerialInterface::togglePlotter() {
    _plotterMode = !_plotterMode;
    Serial.print(F("[PLOT] Plotter mode: "));
    Serial.println(_plotterMode ? F("ON (Target,Speed,PWM,Voltage)") : F("OFF"));
}

void SerialInterface::printStatus() {
    Serial.println(F("\n================= SYSTEM STATUS ================="));
    Serial.print(F("Mode              : "));
    switch (_controller.getMode()) {
        case MODE_IDLE:    Serial.println(F("IDLE")); break;
        case MODE_LOCK:    Serial.println(F("LOCKING (Heading Left)")); break;
        case MODE_UNLOCK:  Serial.println(F("UNLOCKING (Heading Right)")); break;
        case MODE_TIMED:   Serial.println(F("TIMED MOVE")); break;
        case MODE_CYCLING: Serial.println(F("CYCLING")); break;
    }
    Serial.print(F("Direction         : "));
    Serial.println(_controller.getDirection() == DIR_LEFT ? F("LEFT (Lock)") :
                   (_controller.getDirection() == DIR_RIGHT ? F("RIGHT (Unlock)") : F("NONE")));
    Serial.print(F("Drive PWM         : "));
    Serial.print(_controller.getCurrentPwm());
    Serial.print(F(" / 255 ("));
    Serial.print((_controller.getCurrentPwm() * 100) / 255);
    Serial.println(F("%)"));

    Serial.print(F("Target Speed      : "));
    Serial.print(_controller.getTargetSpeed(), 1);
    Serial.println(F(" (scale 0-255)"));

    Serial.print(F("Measured Speed    : "));
    Serial.print(_controller.getMeasuredSpeed(), 1);
    Serial.print(F(" (scale 0-255) | "));
    Serial.print(_controller.getVoltage(), 2);
    Serial.print(F(" V | Raw ADC: "));
    Serial.println(_controller.getFilteredRawAdc(), 1);

    if (_controller.getMode() == MODE_IDLE) {
        uint16_t liveAdc = _controller.getDriver().readFastADC();
        float liveVolts = (liveAdc * 5.0f) / 1023.0f;
        Serial.print(F("Live A0 (Idle)    : "));
        Serial.print(liveVolts, 2);
        Serial.print(F(" V | Raw ADC: "));
        Serial.println(liveAdc);
    }

    const PIDController& pid = _controller.getPid();
    Serial.print(F("PID Gains         : Kp="));
    Serial.print(pid.getKp(), 3);
    Serial.print(F(" | Ki="));
    Serial.print(pid.getKi(), 3);
    Serial.print(F(" | Kd="));
    Serial.print(pid.getKd(), 4);
    Serial.print(F(" | Kff="));
    Serial.print(pid.getKff(), 3);
    Serial.print(F(" | FFo="));
    Serial.print(pid.getFfOffset(), 0);
    Serial.print(F(" | MinPWM="));
    Serial.println(pid.getMinActiveOut(), 0);

    Serial.print(F("Drive Duration    : "));
    Serial.print(_controller.getDriveDuration());
    Serial.println(F(" us"));

    Serial.print(F("Coast Duration    : "));
    if (_controller.getCoastDuration() == 0) {
        Serial.println(F("AUTO / Minimum (~320 us, immediate resume)"));
    } else {
        Serial.print(_controller.getCoastDuration());
        Serial.println(F(" us"));
    }

    Serial.print(F("Left Hall (Lock)  : "));
    Serial.println(_controller.getDriver().isLeftEndstopHit() ? F("TRIGGERED [LOW]") : F("CLEAR [HIGH]"));
    Serial.print(F("Right Hall (Unlock): "));
    Serial.println(_controller.getDriver().isRightEndstopHit() ? F("TRIGGERED [LOW]") : F("CLEAR [HIGH]"));
    Serial.print(F("Plotter Mode      : "));
    Serial.println(_plotterMode ? F("ON") : F("OFF"));
    Serial.println(F("=================================================\n"));
}

void SerialInterface::printHelp() {
    Serial.println(F("=================================================="));
    Serial.println(F("  DC Motor BEMF PID Controller Commands (115200) "));
    Serial.println(F("=================================================="));
    Serial.println(F("  l / lock / k  : Run Lock (drives Left to lock endstop)"));
    Serial.println(F("  u / unlock    : Run Unlock (drives Right to unlock endstop)"));
    Serial.println(F("  m<count>      : Cycle between endstops <count> times (e.g. m10)"));
    Serial.println(F("  l<ms>         : Move Left for <ms> milliseconds (e.g. l500)"));
    Serial.println(F("  r<ms>         : Move Right for <ms> milliseconds (e.g. r500)"));
    Serial.println(F("  s / stop      : Emergency Stop"));
    Serial.println(F("  run<us> / d<us>: Set Drive (Run) duration in us (e.g. run5000, d4000)"));
    Serial.println(F("  c<us> / coast : Set Coast duration in us (e.g. c600, or c0 for auto)"));
    Serial.println(F("  t<val>        : Set Target Speed setpoint (e.g. t140)"));
    Serial.println(F("  p<val>        : Set Kp gain (e.g. p1.8)"));
    Serial.println(F("  i<val>        : Set Ki gain (e.g. i0.8)"));
    Serial.println(F("  d<val>        : Set Kd gain (e.g. d0.0)"));
    Serial.println(F("  ff<val>       : Set Feed-Forward slope (e.g. ff0.54)"));
    Serial.println(F("  ffo<val>      : Set Feed-Forward offset PWM (e.g. ffo158)"));
    Serial.println(F("  min<val>      : Set Minimum Active PWM (e.g. min45)"));
    Serial.println(F("  cal<adc>      : Set Max BEMF ADC calibration counts (default 573)"));
    Serial.println(F("  plot          : Toggle Real-time Serial Plotter (Target,Speed,PWM,V)"));
    Serial.println(F("  ? or h        : Print Help & Current Status"));
    Serial.println(F("=================================================="));
}

void SerialInterface::processCommand(const char* cmd) {
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd == '\0') return;

    // 1. Multi-letter keywords and primary action commands
    if (strcasecmp(cmd, "lock") == 0 || strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0 || strcmp(cmd, "k") == 0 || strcmp(cmd, "K") == 0) {
        _controller.startLock();
    } else if (strcasecmp(cmd, "unlock") == 0 || strcmp(cmd, "u") == 0 || strcmp(cmd, "U") == 0) {
        _controller.startUnlock();
    } else if (strcasecmp(cmd, "stop") == 0 || strcmp(cmd, "s") == 0 || strcmp(cmd, "S") == 0) {
        _controller.stop();
        Serial.println(F("[STOP] Motor stopped / motion aborted."));
    } else if (strcasecmp(cmd, "plot") == 0) {
        togglePlotter();
    } else if (strncasecmp(cmd, "run", 3) == 0 || strncasecmp(cmd, "drive", 5) == 0) {
        const char* p = cmd + (strncasecmp(cmd, "run", 3) == 0 ? 3 : 5);
        while (*p == ' ' || *p == '\t' || *p == '=') p++;
        long val = atol(p);
        if (val < 200) {
            Serial.println(F("[ERROR] Run/Drive duration must be at least 200 us."));
        } else {
            _controller.setDriveDuration((unsigned long)val);
            Serial.print(F("[OK] Drive (Run) duration set to: "));
            Serial.print(_controller.getDriveDuration());
            Serial.println(F(" us"));
        }
    } else if (strncasecmp(cmd, "coast", 5) == 0) {
        const char* p = cmd + 5;
        while (*p == ' ' || *p == '\t' || *p == '=') p++;
        long val = atol(p);
        if (val < 0) {
            Serial.println(F("[ERROR] Coast duration cannot be negative."));
        } else {
            _controller.setCoastDuration((unsigned long)val);
            Serial.print(F("[OK] Coast duration set to: "));
            if (_controller.getCoastDuration() == 0) {
                Serial.println(F("AUTO / Minimum (~320 us, immediate resume)"));
            } else {
                Serial.print(_controller.getCoastDuration());
                Serial.println(F(" us"));
            }
        }
    } else if (strncasecmp(cmd, "ffo", 3) == 0) {
        float val = atof(cmd + 3);
        if (val < 0.0f || val > 255.0f) {
            Serial.println(F("[ERROR] Feed-Forward offset must be 0-255."));
        } else {
            _controller.setFeedForwardOffset(val);
            Serial.print(F("[OK] FFo set to: "));
            Serial.println(_controller.getPid().getFfOffset(), 1);
        }
    } else if (strncasecmp(cmd, "ff", 2) == 0) {
        float val = atof(cmd + 2);
        if (val < 0.0f) {
            Serial.println(F("[ERROR] Feed-Forward gain must be >= 0."));
        } else {
            _controller.setFeedForward(val);
            Serial.print(F("[OK] Kff set to: "));
            Serial.println(_controller.getPid().getKff(), 3);
        }
    } else if (strncasecmp(cmd, "min", 3) == 0) {
        float val = atof(cmd + 3);
        if (val < 0.0f || val > 255.0f) {
            Serial.println(F("[ERROR] Min PWM must be 0-255."));
        } else {
            _controller.setMinActivePwm(val);
            Serial.print(F("[OK] Min active PWM set to: "));
            Serial.println(val, 0);
        }
    } else if (strncasecmp(cmd, "cal", 3) == 0 || strncasecmp(cmd, "max", 3) == 0) {
        const char* valPtr = cmd + 3;
        while (*valPtr == ' ' || *valPtr == '\t' || *valPtr == '=') valPtr++;
        float val = atof(valPtr);
        if (val <= 50.0f || val > 1023.0f) {
            Serial.println(F("[ERROR] Calibration ADC counts must be 50-1023."));
        } else {
            _controller.setMaxBemfCalibration(val);
            Serial.print(F("[OK] Max BEMF ADC scale set to: "));
            Serial.println(_controller.getMaxBemfCalibration(), 1);
        }
    } else if (strcasecmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "H") == 0) {
        printHelp();
        printStatus();
    } 
    // 2. Single-letter prefix commands (m, l, u, r, c, t, p, i, d)
    else if (cmd[0] == 'm' || cmd[0] == 'M') {
        long count = atol(cmd + 1);
        if (count <= 0) {
            Serial.println(F("[ERROR] Cycle count must be positive integer. E.g. m10"));
        } else {
            _controller.startCycling((uint16_t)count);
        }
    } else if (cmd[0] == 'l' || cmd[0] == 'L') {
        const char* p = cmd + 1;
        while (*p == ' ' || *p == '\t') p++;
        long duration = atol(p);
        if (duration <= 0) {
            _controller.startLock();
        } else {
            _controller.startTimed(DIR_LEFT, (unsigned long)duration);
        }
    } else if (cmd[0] == 'u' || cmd[0] == 'U') {
        const char* p = cmd + 1;
        while (*p == ' ' || *p == '\t') p++;
        long duration = atol(p);
        if (duration <= 0) {
            _controller.startUnlock();
        } else {
            _controller.startTimed(DIR_RIGHT, (unsigned long)duration);
        }
    } else if (cmd[0] == 'r' || cmd[0] == 'R') {
        long duration = atol(cmd + 1);
        if (duration <= 0) {
            Serial.println(F("[ERROR] Duration must be positive ms. E.g. r500"));
        } else {
            _controller.startTimed(DIR_RIGHT, (unsigned long)duration);
        }
    } else if ((cmd[0] == 'c' || cmd[0] == 'C') && strncasecmp(cmd, "cal", 3) != 0) {
        const char* p = cmd + 1;
        while (*p == ' ' || *p == '\t' || *p == '=') p++;
        long val = atol(p);
        if (val < 0) {
            Serial.println(F("[ERROR] Coast duration cannot be negative."));
        } else {
            _controller.setCoastDuration((unsigned long)val);
            Serial.print(F("[OK] Coast duration set to: "));
            if (_controller.getCoastDuration() == 0) {
                Serial.println(F("AUTO / Minimum (~320 us, immediate resume)"));
            } else {
                Serial.print(_controller.getCoastDuration());
                Serial.println(F(" us"));
            }
        }
    } else if (cmd[0] == 't' || cmd[0] == 'T') {
        float val = atof(cmd + 1);
        if (val < 0.0f || val > 255.0f) {
            Serial.println(F("[ERROR] Target speed must be between 0 and 255."));
        } else {
            _controller.setTargetSpeed(val);
            Serial.print(F("[OK] Target speed set to: "));
            Serial.println(_controller.getTargetSpeed(), 1);
        }
    } else if (cmd[0] == 'p' || cmd[0] == 'P') {
        float val = atof(cmd + 1);
        const PIDController& pid = _controller.getPid();
        _controller.setPidGains(val, pid.getKi(), pid.getKd());
        Serial.print(F("[OK] Kp set to: "));
        Serial.println(_controller.getPid().getKp(), 4);
    } else if (cmd[0] == 'i' || cmd[0] == 'I') {
        float val = atof(cmd + 1);
        const PIDController& pid = _controller.getPid();
        _controller.setPidGains(pid.getKp(), val, pid.getKd());
        Serial.print(F("[OK] Ki set to: "));
        Serial.println(_controller.getPid().getKi(), 4);
    } else if (cmd[0] == 'd' || cmd[0] == 'D') {
        const char* p = cmd + 1;
        while (*p == ' ' || *p == '\t' || *p == '=') p++;
        float val = atof(p);
        if (val >= 100.0f && strchr(p, '.') == nullptr) {
            _controller.setDriveDuration((unsigned long)val);
            Serial.print(F("[OK] Drive (Run) duration set to: "));
            Serial.print(_controller.getDriveDuration());
            Serial.println(F(" us"));
        } else {
            const PIDController& pid = _controller.getPid();
            _controller.setPidGains(pid.getKp(), pid.getKi(), val);
            Serial.print(F("[OK] Kd set to: "));
            Serial.println(_controller.getPid().getKd(), 4);
        }
    } else {
        Serial.print(F("[ERROR] Unknown command: '"));
        Serial.print(cmd);
        Serial.println(F("'. Send '?' for help."));
    }
}
