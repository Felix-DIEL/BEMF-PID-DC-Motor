# DC Motor BEMF Closed-Loop PID Controller with DRV8871 & Hall Endstops

A high-performance embedded control firmware for an Arduino Uno (ATmega328P) regulating the speed of a heavily geared DC motor via **sensorless Back-EMF (BEMF) feedback**, driving a TI **DRV8871** H-bridge, and managing two **AH1806** digital Hall-effect endstops for an automated attachment locking system.

---

## Features

- **Microsecond Chopping BEMF Sensing**:
  - **Drive Phase (5.0 ms)**: High-frequency **31.25 kHz** PWM (Timer1) for silent, smooth motor torque.
  - **Flyback Blanking (120 µs)**: High-Z coast window allowing diode flyback and recirculation currents to fully decay before analog measurement.
  - **High-Speed Multi-Sample ADC (200 µs)**: ATmega328P ADC prescaler set to 32 (500 kHz clock, ~26 µs/sample), capturing and averaging 6–7 consecutive BEMF voltage samples.
  - **3-Phase Historical Moving Average**: Smooths brush noise and commutator ripple by filtering the current coast reading across the last 2 coast phases.
  - **Zero Dead-Coast Overhead**: Drive re-engages immediately after PID calculation (~320 µs total coast window) for maximum torque and minimal speed droop.
- **Feed-Forward + PID Algorithm**:
  - Linear friction offset + speed slope feed-forward ($\text{FF} = \text{FF}_o + K_{ff} \cdot \text{Target}$) eliminates the multi-second startup lag of pure integral control on geared motors.
  - Conditional integrator anti-windup prevents saturation overshoot.
  - Filtered derivative on measurement prevents setpoint kick and brush-ripple jitter.
- **Robust EMI Glitch Filtering**:
  - 200 µs sustained digital verification filter on Hall sensor inputs to reject motor brush arcing and high-current PWM switching spikes.
- **Comprehensive CLI & Real-time Plotting**:
  - Live tuning of setpoint, drive/coast timing, PID gains, and feed-forward parameters over 115200 baud serial.
  - Compatible with Arduino Serial Plotter (`Target`, `Measured`, `PWM`, `Volts`).

---

## Hardware Architecture & Pinout

```
                           +-------------------+
                           |    Arduino Uno    |
                           |   (ATmega328P)    |
                           +-------------------+
                               |     |     |
                 Timer1 PWM    |     |     |    Timer1 PWM
                 Pin 9 (OC1A)  |     |     |    Pin 10 (OC1B)
                       +-------+     |     +-------+
                       |             |             |
                       v             |             v
                 +-----------+       |       +------------+
                 | DRV8871   |       |       | DRV8871    |
                 | IN1 (Lock)|       |       | IN2(Unlock)|
                 +-----------+       |       +------------+
                       |             |             |
                       +-------[ DC Motor ]--------+
                                     |
                                     | Motor BEMF Divider
                                     v
                           +-------------------+
                           | Analog Pin A0     |
                           +-------------------+
                               |           |
            Hall Left (Lock)   |           |   Hall Right (Unlock)
            Pin 2 (AH1806) <---+           +---> Pin 3 (AH1806)
                               |
            Scope Sync Trigger |
            Pin 4 (HIGH @ coast)+--------------> Oscilloscope Ch1 Trigger
```

| Pin | Function | Peripheral / Description |
| :--- | :--- | :--- |
| **Pin 9** | Motor IN1 | DRV8871 IN1 (Timer1 OC1A, 31.25 kHz Phase-Correct PWM) |
| **Pin 10** | Motor IN2 | DRV8871 IN2 (Timer1 OC1B, 31.25 kHz Phase-Correct PWM) |
| **Pin 2** | Hall Left (Lock) | AH1806 digital Hall switch (Active LOW, 200 µs glitch filter) |
| **Pin 3** | Hall Right (Unlock) | AH1806 digital Hall switch (Active LOW, 200 µs glitch filter) |
| **Pin 4** | Scope Sync | Diagnostic trigger (HIGH during coast window, LOW during drive) |
| **Pin A0** | BEMF Sense | Analog input (0–2.8 V signal, 5.0 V AREF default, prescaler 32) |

---

## Codebase Structure

The project is structured into modular libraries for maintainability and testability:

```text
├── include/
│   ├── Config.h             # Pin assignments, timing defaults, scaling constants
│   ├── PIDController.h      # Feed-forward + anti-windup PID class
│   ├── MotorDriver.h        # DRV8871 H-bridge, Timer1 PWM, and fast ADC hardware
│   ├── BEMFController.h     # BEMF chopping engine & motion state machine
│   └── SerialInterface.h    # CLI parser, status printer & plotter telemetry
├── src/
│   ├── MotorDriver.cpp      # Low-level timer, ADC and GPIO implementations
│   ├── BEMFController.cpp   # Speed filtering, PID step, cycling & endstop transitions
│   ├── SerialInterface.cpp  # Command dispatcher & 50 Hz telemetry streamer
│   └── main.cpp             # Clean entry point (setup & loop)
├── platformio.ini           # PlatformIO project configuration
└── README.md                # Project documentation
```

---

## Back-EMF Sensing & Chopping Cycle

A single chopping cycle consists of the following microsecond sequence:

```text
+------------------------------+-------+--------------------+-------+
|         5.0 ms DRIVE         | 120us |    ~180-200 us     |       |
|  Timer1 PWM @ 31.25 kHz      | Blank | ADC Fast Sampling  | PID   | -> DRIVE RESUMES
|  (DRV8871 actively powered)  | Coast | (6-7 samples @ A0) | Math  |
+------------------------------+-------+--------------------+-------+
                               |<---------- Total Coast: ~320 us --------->|
```

1. **Drive Phase (`driveDurationUs`, default: 5000 µs)**: Motor receives power via PWM.
2. **Coast Switch**: DRV8871 inputs pulled `LOW` (High-Z coast), `PIN_BEMF_SYNC` pulls `HIGH`.
3. **Blanking (`120 µs`)**: No ADC reads occur while the inductive flyback current recirculates and decays through the driver body diodes.
4. **Sampling Window (`200 µs`)**: ATmega328P takes consecutive ADC conversions (~26 µs each), averaging 6 to 7 clean BEMF voltage samples.
5. **Filtering**: The coast average is smoothed with the previous 2 coast averages:
   $$\text{BEMF}_{\text{filtered}} = \frac{\text{Coast}_n + \text{Coast}_{n-1} + \text{Coast}_{n-2}}{3}$$
6. **PID & Feed-Forward**: Output PWM is calculated:
   $$\text{PWM} = \text{FF}_o + (K_{ff} \cdot \text{Target}) + K_p \cdot e + \int K_i \cdot e \, dt - K_d \cdot \frac{d(\text{BEMF})}{dt}$$
7. **Immediate Drive Resume**: Drive re-engages immediately with the updated PWM (or after an extended user-configured coast time via `c<us>`).

---

## Serial Command Reference (115200 Baud)

All commands are case-insensitive and can be entered with or without spaces.

### Motion Commands
| Command | Description | Example |
| :--- | :--- | :--- |
| `l` / `lock` / `k` | Run **Lock** sequence (drives Left until Pin 2 triggers) | `l` |
| `u` / `unlock` | Run **Unlock** sequence (drives Right until Pin 3 triggers) | `u` |
| `m<count>` | Cycle back and forth `<count>` times between endstops | `m10` |
| `l<ms>` | Drive Left for `<ms>` milliseconds | `l500` |
| `r<ms>` / `u<ms>` | Drive Right for `<ms>` milliseconds | `r500` |
| `s` / `stop` | **Emergency Stop** (instantly halts drive & resets PID) | `s` |

### Timing & Speed Commands
| Command | Description | Default | Example |
| :--- | :--- | :--- | :--- |
| `t<val>` | Set target speed setpoint (0–255 scale) | `140.0` | `t140` |
| `run<us>` / `d<us>` | Set active **Drive (Run)** phase duration in µs | `5000` | `run5000` or `d4000` |
| `c<us>` / `coast` | Set **Coast** duration in µs (`0` = auto/minimum ~320 µs) | `0` | `c600` or `c0` |
| `cal<adc>` | Set max BEMF ADC counts corresponding to 2.8 V | `573.0` | `cal573` |

### PID & Feed-Forward Tuning Commands
| Command | Description | Default | Example |
| :--- | :--- | :--- | :--- |
| `ffo<val>` | Set Feed-Forward base offset PWM (overcomes stiction) | `158.0` | `ffo158` |
| `ff<val>` | Set Feed-Forward slope gain | `0.54` | `ff0.54` |
| `p<val>` | Set Proportional gain ($K_p$) | `2.5` | `p2.5` |
| `i<val>` | Set Integral gain ($K_i$) | `2.5` | `i2.5` |
| `d<val>` | Set Derivative gain ($K_d$) | `0.0` | `d0.0` |
| `min<val>` | Set Minimum Active PWM clamp | `120.0` | `min120` |

### Diagnostics & Plotting
| Command | Description |
| :--- | :--- |
| `plot` | Toggle 50 Hz real-time telemetry stream (`Target,Measured,PWM,Volts`) |
| `?` or `h` | Print complete system status, sensor states, and live A0 voltage |

---

## Logging & Data Capture

To log live telemetry directly into a file for analysis:
```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -f default -f log2file
```
PlatformIO automatically creates a timestamped log file inside the `logs/` directory.

---

## Build & Upload

Using PlatformIO CLI:
```powershell
# Compile firmware
pio run

# Upload to Arduino Uno
pio run -t upload

# Open Serial Monitor at 115200 baud
pio device monitor
```
