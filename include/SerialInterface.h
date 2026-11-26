#pragma once
#include <Arduino.h>
#include "Config.h"
#include "BEMFController.h"

/**
 * ==============================================================================
 * SerialInterface - CLI Command Parser, Status Reporting & Plotter Streamer
 * ==============================================================================
 */
class SerialInterface {
public:
    explicit SerialInterface(BEMFController& controller);

    /**
     * Initializes the serial port at specified baud rate and prints startup banner.
     */
    void begin(unsigned long baud = Config::SERIAL_BAUD_RATE);

    /**
     * Polls incoming serial commands and streams plotter telemetry. Call in loop().
     */
    void update();

    /**
     * Parses and executes a single text command.
     */
    void processCommand(const char* cmd);

    /**
     * Prints complete help menu and commands list.
     */
    void printHelp();

    /**
     * Prints current system parameters, sensor readings, and live voltage.
     */
    void printStatus();

    /**
     * Toggles Arduino Serial Plotter real-time telemetry stream.
     */
    void togglePlotter();

    bool isPlotterActive() const { return _plotterMode; }

private:
    void streamPlotter();

    BEMFController& _controller;
    bool            _plotterMode;
    unsigned long   _lastPlotMs;
    MotorDirection  _lastPlottedDirection;

    char   _rxBuffer[Config::CLI_RX_BUFFER_SIZE];
    size_t _rxIndex;
};
