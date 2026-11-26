#include <Arduino.h>
#include "BEMFController.h"
#include "SerialInterface.h"

/**
 * ==============================================================================
 * BEMF Closed-Loop PID DC Motor Controller with DRV8871 & Hall Endstops
 * Board: Arduino Uno (ATmega328P @ 16 MHz)
 * ==============================================================================
 */

BEMFController  motor;
SerialInterface serialUI(motor);

void setup() {
    motor.begin();
    serialUI.begin(Config::SERIAL_BAUD_RATE);
}

void loop() {
    serialUI.update();
    motor.update();
}
