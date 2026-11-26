#include <Arduino.h>

/*
 * ==============================================================================
 * DC Motor Control with DRV8871, AH1806-Z-7 Endstops & Back-EMF Coast Windows
 * Board: Arduino Uno
 *
 * --- WIRING GUIDE ---
 * 1. DRV8871 Motor Driver:
 *    - IN1         -> Arduino Pin 9 (Timer1 PWM, configured to 31.25 kHz)
 *    - IN2         -> Arduino Pin 10 (Timer1 PWM, configured to 31.25 kHz)
 *    - GND         -> Common Ground with Arduino GND & Motor Power Supply GND
 *    - VM          -> Motor Power Supply (6.5V - 45V)
 *    - MOTOR (+/-) -> DC Motor terminals
 *
 * 2. AH1806-Z-7 Digital Hall Sensors (Omnipolar switch with Open-Drain output):
 *    - VDD         -> Arduino 5V
 *    - GND         -> Arduino GND
 *    - OUTPUT Left  -> Arduino Pin 2 (Configured as INPUT, external pullup used)
 *    - OUTPUT Right -> Arduino Pin 3 (Configured as INPUT, external pullup used)
 *    Note: Open-drain output pulls LOW when magnet is present.
 *
 * 3. BEMF Scope / Measurement Sync Output (Optional):
 *    - Pin 4       -> Goes HIGH during the coast (BEMF) window, LOW during drive.
 *                     Ideal for triggering an oscilloscope or external ADC.
 *
 * --- SERIAL COMMANDS (Baud: 115200) ---
 *    - m<count>: Cycle to each endstop <count> times (e.g. "m10")
 *    - l<ms>   : Move Left for <ms> milliseconds (e.g. "l100")
 *    - r<ms>   : Move Right for <ms> milliseconds (e.g. "r13")
 *    - v<0-255>: Set global motor speed/PWM (e.g. "v200", default: 255)
 *    - c<us>   : Set BEMF coast duration in microseconds (e.g. "c100", default: 100)
 *    - d<us>   : Set drive phase duration in microseconds (e.g. "d200", default: 200)
 *    - s       : Emergency Stop / Abort current motion or cycling
 *    - ? or h  : Status & Help
 * ==============================================================================
 */

// --- Pin Definitions ---
constexpr uint8_t PIN_MOTOR_IN1   = 9;   // DRV8871 IN1 (Timer1 OC1A)
constexpr uint8_t PIN_MOTOR_IN2   = 10;  // DRV8871 IN2 (Timer1 OC1B)
constexpr uint8_t PIN_HALL_LEFT   = 2;   // Left Endstop (AH1806)
constexpr uint8_t PIN_HALL_RIGHT  = 3;   // Right Endstop (AH1806)
constexpr uint8_t PIN_BEMF_SYNC   = 4;   // HIGH during BEMF coast window for scope trigger

// AH1806 open-drain output pulls LOW when magnetic field is detected
constexpr uint8_t ENDSTOP_TRIGGERED = LOW;

// --- Configurable Parameters ---
uint8_t       motorSpeed        = 255;  // PWM duty cycle (0 - 255)
unsigned long driveDurationUs   = 200;  // Drive phase duration in microseconds
unsigned long coastDurationUs   = 100;  // Coast / BEMF measurement window in microseconds

// --- State Machine ---
enum MotorDirection {
    DIR_IDLE,
    DIR_LEFT,
    DIR_RIGHT
};

enum MotionMode {
    MODE_IDLE,
    MODE_TIMED,
    MODE_CYCLING
};

enum BemfPhase {
    PHASE_DRIVE,
    PHASE_COAST
};

MotorDirection currentDirection = DIR_IDLE;
MotionMode     currentMode      = MODE_IDLE;
BemfPhase      currentBemfPhase = PHASE_DRIVE;

// Timed move parameters
unsigned long moveStartTimeMs   = 0;
unsigned long moveDurationMs    = 0;
unsigned long phaseStartTimeUs  = 0;

// Cycling parameters
uint16_t targetCycles           = 0;
uint16_t leftHitsRemaining      = 0;
uint16_t rightHitsRemaining     = 0;
bool     inReversePause         = false;
unsigned long pauseStartTimeMs  = 0;
unsigned long strokeStartTimeMs = 0;

constexpr unsigned long REVERSE_PAUSE_MS = 100;    // Coast delay before reversing to avoid gear/current shock
constexpr unsigned long STROKE_TIMEOUT_MS = 30000; // Safety timeout per stroke during cycling (30s)

// --- Serial RX Buffer ---
constexpr size_t BUFFER_SIZE = 32;
char rxBuffer[BUFFER_SIZE];
size_t rxIndex = 0;

// --- Endstop Helpers ---
inline bool isLeftEndstopHit() {
    return digitalRead(PIN_HALL_LEFT) == ENDSTOP_TRIGGERED;
}

inline bool isRightEndstopHit() {
    return digitalRead(PIN_HALL_RIGHT) == ENDSTOP_TRIGGERED;
}

// --- Low-Level Driver Controls ---
void applyCoast() {
    // Both inputs LOW -> DRV8871 outputs enter High-Z (coast)
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    digitalWrite(PIN_BEMF_SYNC, HIGH); // Signal scope: BEMF measurement active
}

void applyDrive() {
    digitalWrite(PIN_BEMF_SYNC, LOW);  // Signal scope: Drive active

    if (currentDirection == DIR_LEFT) {
        digitalWrite(PIN_MOTOR_IN2, LOW);
        if (motorSpeed == 255) {
            digitalWrite(PIN_MOTOR_IN1, HIGH);
        } else if (motorSpeed == 0) {
            digitalWrite(PIN_MOTOR_IN1, LOW);
        } else {
            analogWrite(PIN_MOTOR_IN1, motorSpeed);
        }
    } else if (currentDirection == DIR_RIGHT) {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        if (motorSpeed == 255) {
            digitalWrite(PIN_MOTOR_IN2, HIGH);
        } else if (motorSpeed == 0) {
            digitalWrite(PIN_MOTOR_IN2, LOW);
        } else {
            analogWrite(PIN_MOTOR_IN2, motorSpeed);
        }
    }
}

void motorStop() {
    applyCoast();
    digitalWrite(PIN_BEMF_SYNC, LOW);
    currentDirection   = DIR_IDLE;
    currentMode        = MODE_IDLE;
    currentBemfPhase   = PHASE_DRIVE;
    inReversePause     = false;
    leftHitsRemaining  = 0;
    rightHitsRemaining = 0;
}

void printStatus() {
    Serial.println(F("\n--- CURRENT CONFIG & STATUS ---"));
    Serial.print(F("Mode              : "));
    switch (currentMode) {
        case MODE_IDLE:    Serial.println(F("IDLE")); break;
        case MODE_TIMED:   Serial.println(F("TIMED MOVE")); break;
        case MODE_CYCLING: {
            Serial.print(F("CYCLING (Target: "));
            Serial.print(targetCycles);
            Serial.print(F(" | Remaining: Left="));
            Serial.print(leftHitsRemaining);
            Serial.print(F(", Right="));
            Serial.print(rightHitsRemaining);
            if (inReversePause) {
                Serial.print(F(" [PAUSE/REVERSE]"));
            }
            Serial.println(F(")"));
            break;
        }
    }

    Serial.print(F("Direction         : "));
    switch (currentDirection) {
        case DIR_IDLE:  Serial.println(F("NONE")); break;
        case DIR_LEFT:  Serial.println(F("LEFT")); break;
        case DIR_RIGHT: Serial.println(F("RIGHT")); break;
    }

    Serial.print(F("Speed (0-255)     : "));
    Serial.print(motorSpeed);
    Serial.print(F(" ("));
    Serial.print((motorSpeed * 100) / 255);
    Serial.println(F("%)"));

    Serial.print(F("Drive Phase (d)   : "));
    Serial.print(driveDurationUs);
    Serial.println(F(" us"));

    Serial.print(F("Coast / BEMF (c)  : "));
    Serial.print(coastDurationUs);
    Serial.println(F(" us"));

    Serial.print(F("Left Hall   (D2)  : "));
    Serial.println(isLeftEndstopHit() ? F("TRIGGERED (Magnet Present)") : F("CLEAR"));

    Serial.print(F("Right Hall  (D3)  : "));
    Serial.println(isRightEndstopHit() ? F("TRIGGERED (Magnet Present)") : F("CLEAR"));
    Serial.println(F("-------------------------------\n"));
}

void printHelp() {
    Serial.println(F("=== DRV8871 Motor & BEMF Measurement Controller ==="));
    Serial.println(F("Commands:"));
    Serial.println(F("  m<count>  : Cycle to each endstop <count> times (e.g. m10)"));
    Serial.println(F("  l<ms>     : Move Left for <ms> milliseconds (e.g. l100)"));
    Serial.println(F("  r<ms>     : Move Right for <ms> milliseconds (e.g. r13)"));
    Serial.println(F("  v<0-255>  : Set global speed (e.g. v200 or v255)"));
    Serial.println(F("  c<us>     : Set BEMF coast duration in us (e.g. c100)"));
    Serial.println(F("  d<us>     : Set drive interval in us (e.g. d200)"));
    Serial.println(F("  s         : Emergency Stop / Abort"));
    Serial.println(F("  ? or h    : Print Status & Help"));
    Serial.println(F("===================================================="));
}

void startMovement(MotorDirection dir, unsigned long durationMs) {
    if (durationMs == 0) {
        Serial.println(F("[WARN] Duration must be > 0 ms."));
        return;
    }

    if (dir == DIR_LEFT) {
        if (isLeftEndstopHit()) {
            Serial.println(F("[BLOCKED] Left endstop already triggered! Movement aborted."));
            return;
        }
    } else if (dir == DIR_RIGHT) {
        if (isRightEndstopHit()) {
            Serial.println(F("[BLOCKED] Right endstop already triggered! Movement aborted."));
            return;
        }
    }

    currentMode      = MODE_TIMED;
    currentDirection = dir;
    moveDurationMs   = durationMs;
    moveStartTimeMs  = millis();

    // Start with drive phase
    currentBemfPhase = PHASE_DRIVE;
    phaseStartTimeUs = micros();
    applyDrive();

    Serial.print(F("[RUN] Moving "));
    Serial.print(dir == DIR_LEFT ? F("LEFT") : F("RIGHT"));
    Serial.print(F(" for "));
    Serial.print(durationMs);
    Serial.print(F(" ms | Speed="));
    Serial.print(motorSpeed);
    Serial.print(F(" | Drive="));
    Serial.print(driveDurationUs);
    Serial.print(F("us | Coast(BEMF)="));
    Serial.print(coastDurationUs);
    Serial.println(F("us"));
}

void startCycling(uint16_t count) {
    if (count == 0) {
        Serial.println(F("[ERROR] Cycle count must be > 0."));
        return;
    }

    targetCycles       = count;
    leftHitsRemaining  = count;
    rightHitsRemaining = count;
    currentMode        = MODE_CYCLING;
    inReversePause     = false;

    // Determine initial direction:
    // If already touching Left endstop, move Right. Otherwise start towards Left.
    if (isLeftEndstopHit()) {
        currentDirection = DIR_RIGHT;
    } else {
        currentDirection = DIR_LEFT;
    }

    currentBemfPhase  = PHASE_DRIVE;
    phaseStartTimeUs  = micros();
    strokeStartTimeMs = millis();
    applyDrive();

    Serial.print(F("[CYCLE] Starting: "));
    Serial.print(count);
    Serial.print(F(" trips to each endstop. Heading "));
    Serial.print(currentDirection == DIR_LEFT ? F("LEFT") : F("RIGHT"));
    Serial.print(F(" first (Speed="));
    Serial.print(motorSpeed);
    Serial.println(F(")..."));
}

void processCommand(const char* cmd) {
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd == '\0') return;

    char action = *cmd;

    if (action == 's' || action == 'S') {
        motorStop();
        Serial.println(F("[STOP] Motor stopped / motion aborted."));
    } else if (action == '?' || action == 'h' || action == 'H') {
        printHelp();
        printStatus();
    } else if (action == 'm' || action == 'M') {
        const char* numStr = cmd + 1;
        while (*numStr == ' ' || *numStr == '\t') numStr++;
        if (*numStr == '\0') {
            Serial.println(F("[ERROR] Missing cycle count. Example usage: m10"));
            return;
        }
        long count = atol(numStr);
        if (count <= 0) {
            Serial.println(F("[ERROR] Cycle count must be a positive integer."));
            return;
        }
        startCycling((uint16_t)count);
    } else if (action == 'v' || action == 'V' || action == 'p' || action == 'P') {
        const char* valStr = cmd + 1;
        while (*valStr == ' ' || *valStr == '\t') valStr++;
        long val = atol(valStr);
        if (val < 0 || val > 255) {
            Serial.println(F("[ERROR] Speed must be between 0 and 255."));
        } else {
            motorSpeed = (uint8_t)val;
            Serial.print(F("[OK] Global speed set to: "));
            Serial.print(motorSpeed);
            Serial.print(F(" ("));
            Serial.print((motorSpeed * 100) / 255);
            Serial.println(F("%)"));
            // Update active drive if motor is currently running
            if (currentDirection != DIR_IDLE && currentBemfPhase == PHASE_DRIVE && !inReversePause) {
                applyDrive();
            }
        }
    } else if (action == 'c' || action == 'C') {
        const char* valStr = cmd + 1;
        while (*valStr == ' ' || *valStr == '\t') valStr++;
        long val = atol(valStr);
        if (val < 10) {
            Serial.println(F("[ERROR] Coast duration must be at least 10 us."));
        } else {
            coastDurationUs = (unsigned long)val;
            Serial.print(F("[OK] BEMF Coast duration set to: "));
            Serial.print(coastDurationUs);
            Serial.println(F(" us"));
        }
    } else if (action == 'd' || action == 'D') {
        const char* valStr = cmd + 1;
        while (*valStr == ' ' || *valStr == '\t') valStr++;
        long val = atol(valStr);
        if (val < 10) {
            Serial.println(F("[ERROR] Drive duration must be at least 10 us."));
        } else {
            driveDurationUs = (unsigned long)val;
            Serial.print(F("[OK] Drive phase duration set to: "));
            Serial.print(driveDurationUs);
            Serial.println(F(" us"));
        }
    } else if (action == 'l' || action == 'L' || action == 'r' || action == 'R') {
        const char* numStr = cmd + 1;
        while (*numStr == ' ' || *numStr == '\t') numStr++;
        if (*numStr == '\0') {
            Serial.println(F("[ERROR] Missing duration. E.g.: l100 or r13"));
            return;
        }
        long duration = atol(numStr);
        if (duration <= 0) {
            Serial.println(F("[ERROR] Invalid duration. Must be positive integer ms."));
            return;
        }
        startMovement((action == 'l' || action == 'L') ? DIR_LEFT : DIR_RIGHT, (unsigned long)duration);
    } else {
        Serial.print(F("[ERROR] Unknown command: '"));
        Serial.print(cmd);
        Serial.println(F("'. Send '?' for help."));
    }
}

void readSerialCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxIndex > 0) {
                rxBuffer[rxIndex] = '\0';
                processCommand(rxBuffer);
                rxIndex = 0;
            }
        } else {
            if (rxIndex < BUFFER_SIZE - 1) {
                rxBuffer[rxIndex++] = c;
            }
        }
    }
}

void updateMotorMotion() {
    if (currentMode == MODE_IDLE) {
        return;
    }

    // =========================================================================
    // 1. TIMED MODE (l<ms> or r<ms>)
    // =========================================================================
    if (currentMode == MODE_TIMED) {
        unsigned long nowMs = millis();
        unsigned long elapsedMs = nowMs - moveStartTimeMs;

        // Check endstops immediately
        if (currentDirection == DIR_LEFT && isLeftEndstopHit()) {
            motorStop();
            Serial.print(F("[ENDSTOP] Left endstop reached! Stopped after "));
            Serial.print(elapsedMs);
            Serial.println(F(" ms."));
            return;
        } else if (currentDirection == DIR_RIGHT && isRightEndstopHit()) {
            motorStop();
            Serial.print(F("[ENDSTOP] Right endstop reached! Stopped after "));
            Serial.print(elapsedMs);
            Serial.println(F(" ms."));
            return;
        }

        // Check movement duration timeout
        if (elapsedMs >= moveDurationMs) {
            motorStop();
            Serial.print(F("[DONE] Movement finished after "));
            Serial.print(elapsedMs);
            Serial.println(F(" ms."));
            return;
        }
    }

    // =========================================================================
    // 2. CYCLING MODE (m<count>)
    // =========================================================================
    else if (currentMode == MODE_CYCLING) {
        unsigned long nowMs = millis();

        // Handle settle pause before reversing direction
        if (inReversePause) {
            if (nowMs - pauseStartTimeMs >= REVERSE_PAUSE_MS) {
                inReversePause    = false;
                strokeStartTimeMs = nowMs;
                currentBemfPhase  = PHASE_DRIVE;
                phaseStartTimeUs  = micros();
                applyDrive();
            }
            return; // Stay in coast during reverse pause
        }

        // Check safety timeout for single stroke (e.g. if sensor disconnected)
        if (nowMs - strokeStartTimeMs >= STROKE_TIMEOUT_MS) {
            motorStop();
            Serial.println(F("[ERROR] Stroke timeout exceeded (30s)! Motion aborted for safety."));
            return;
        }

        // Check active endstop
        if (currentDirection == DIR_LEFT && isLeftEndstopHit()) {
            if (leftHitsRemaining > 0) leftHitsRemaining--;

            Serial.print(F("[CYCLE] Left endstop hit! Left left: "));
            Serial.print(leftHitsRemaining);
            Serial.print(F(" | Right left: "));
            Serial.println(rightHitsRemaining);

            if (leftHitsRemaining == 0 && rightHitsRemaining == 0) {
                motorStop();
                Serial.print(F("[CYCLE] Completed all "));
                Serial.print(targetCycles);
                Serial.println(F(" cycles to each endstop successfully!"));
                return;
            }

            // Settle in coast before heading Right
            applyCoast();
            inReversePause    = true;
            pauseStartTimeMs  = nowMs;
            currentDirection  = DIR_RIGHT;
            return;
        } else if (currentDirection == DIR_RIGHT && isRightEndstopHit()) {
            if (rightHitsRemaining > 0) rightHitsRemaining--;

            Serial.print(F("[CYCLE] Right endstop hit! Left left: "));
            Serial.print(leftHitsRemaining);
            Serial.print(F(" | Right left: "));
            Serial.println(rightHitsRemaining);

            if (leftHitsRemaining == 0 && rightHitsRemaining == 0) {
                motorStop();
                Serial.print(F("[CYCLE] Completed all "));
                Serial.print(targetCycles);
                Serial.println(F(" cycles to each endstop successfully!"));
                return;
            }

            // Settle in coast before heading Left
            applyCoast();
            inReversePause    = true;
            pauseStartTimeMs  = nowMs;
            currentDirection  = DIR_LEFT;
            return;
        }
    }

    // =========================================================================
    // 3. BEMF DRIVE / COAST MICROSECOND CHOPPING
    // =========================================================================
    unsigned long nowUs = micros();
    unsigned long phaseElapsedUs = nowUs - phaseStartTimeUs;

    if (currentBemfPhase == PHASE_DRIVE) {
        if (phaseElapsedUs >= driveDurationUs) {
            currentBemfPhase = PHASE_COAST;
            phaseStartTimeUs = nowUs;
            applyCoast();
        }
    } else { // PHASE_COAST
        if (phaseElapsedUs >= coastDurationUs) {
            currentBemfPhase = PHASE_DRIVE;
            phaseStartTimeUs = nowUs;
            applyDrive();
        }
    }
}

void setup() {
    // Pin configuration
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    pinMode(PIN_BEMF_SYNC, OUTPUT);
    motorStop();

    // Configure Hall sensor pins as standard inputs (no internal pull-up)
    pinMode(PIN_HALL_LEFT, INPUT);
    pinMode(PIN_HALL_RIGHT, INPUT);

    /*
     * Increase Timer1 PWM frequency to 31.25 kHz on Pins 9 & 10.
     * Arduino Uno default is 490 Hz (prescaler 64). A 490 Hz period is 2040 us,
     * which is too slow for a 200 us drive pulse.
     * Setting prescaler to 1 gives 16 MHz / 510 = 31.25 kHz (period = 32 us),
     * allowing clean PWM duty cycle control inside the 200 us drive window!
     */
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;

    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println();
    printHelp();
    printStatus();
}

void loop() {
    readSerialCommands();
    updateMotorMotion();
}
