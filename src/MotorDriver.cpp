#include "MotorDriver.h"

void MotorDriver::begin() {
    // 1. Motor control pins
    pinMode(Config::PIN_MOTOR_IN1, OUTPUT);
    pinMode(Config::PIN_MOTOR_IN2, OUTPUT);
    pinMode(Config::PIN_BEMF_SYNC, OUTPUT);
    stop();

    // 2. Hall Sensors (External pull-up resistors installed on AH1806 open-drain outputs)
    pinMode(Config::PIN_HALL_LEFT, INPUT);
    pinMode(Config::PIN_HALL_RIGHT, INPUT);

    // 3. Configure Timer1 for 31.25 kHz Phase-Correct PWM on Pins 9 & 10
    // Setting prescaler to 1 gives 16 MHz / 512 = 31.25 kHz (period = 32 us)
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;

    // 4. Configure ATmega328P ADC for high-speed sampling
    // Reference: AVCC with external capacitor on AREF pin (5.0V)
    // Channel  : ADC0 (Pin A0)
    ADMUX = (1 << REFS0);
    // Prescaler: 32 -> 16 MHz / 32 = 500 kHz ADC clock (~26 us per conversion)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);

    // Dummy ADC reads to stabilize analog circuitry & reference
    readFastADC();
    delay(10);
    readFastADC();
}

void MotorDriver::drive(MotorDirection dir, uint8_t pwm) {
    setScopeSync(false);

    if (dir == DIR_LEFT) {
        // Drive Left (Lock): IN1 = PWM, IN2 = LOW
        digitalWrite(Config::PIN_MOTOR_IN2, LOW);
        if (pwm == 255) {
            digitalWrite(Config::PIN_MOTOR_IN1, HIGH);
        } else if (pwm == 0) {
            digitalWrite(Config::PIN_MOTOR_IN1, LOW);
        } else {
            analogWrite(Config::PIN_MOTOR_IN1, pwm);
        }
    } else if (dir == DIR_RIGHT) {
        // Drive Right (Unlock): IN1 = LOW, IN2 = PWM
        digitalWrite(Config::PIN_MOTOR_IN1, LOW);
        if (pwm == 255) {
            digitalWrite(Config::PIN_MOTOR_IN2, HIGH);
        } else if (pwm == 0) {
            digitalWrite(Config::PIN_MOTOR_IN2, LOW);
        } else {
            analogWrite(Config::PIN_MOTOR_IN2, pwm);
        }
    } else {
        stop();
    }
}

void MotorDriver::coast() {
    // Both driver inputs LOW -> DRV8871 outputs enter High-Z (coast)
    digitalWrite(Config::PIN_MOTOR_IN1, LOW);
    digitalWrite(Config::PIN_MOTOR_IN2, LOW);
    setScopeSync(true);
}

void MotorDriver::stop() {
    digitalWrite(Config::PIN_MOTOR_IN1, LOW);
    digitalWrite(Config::PIN_MOTOR_IN2, LOW);
    setScopeSync(false);
}

void MotorDriver::setScopeSync(bool high) {
    digitalWrite(Config::PIN_BEMF_SYNC, high ? HIGH : LOW);
}

uint16_t MotorDriver::readFastADC() {
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for hardware conversion flag
    return ADC;
}

bool MotorDriver::isLeftEndstopHit() {
    if (digitalRead(Config::PIN_HALL_LEFT) != Config::ENDSTOP_TRIGGERED) {
        return false;
    }
    // Glitch filter: Require sustained signal across 200 us to reject motor switching/brush EMI spikes
    for (uint8_t i = 0; i < Config::ENDSTOP_FILTER_SAMPLES; i++) {
        delayMicroseconds(Config::ENDSTOP_FILTER_DELAY_US);
        if (digitalRead(Config::PIN_HALL_LEFT) != Config::ENDSTOP_TRIGGERED) {
            return false;
        }
    }
    return true;
}

bool MotorDriver::isRightEndstopHit() {
    if (digitalRead(Config::PIN_HALL_RIGHT) != Config::ENDSTOP_TRIGGERED) {
        return false;
    }
    // Glitch filter: Require sustained signal across 200 us to reject motor switching/brush EMI spikes
    for (uint8_t i = 0; i < Config::ENDSTOP_FILTER_SAMPLES; i++) {
        delayMicroseconds(Config::ENDSTOP_FILTER_DELAY_US);
        if (digitalRead(Config::PIN_HALL_RIGHT) != Config::ENDSTOP_TRIGGERED) {
            return false;
        }
    }
    return true;
}
