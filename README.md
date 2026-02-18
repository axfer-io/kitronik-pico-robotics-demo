# Pico C/C++ Firmware Template — Kitronik Robotics (I2C PCA9685)

Embedded systems · Control · Debugging ⚙️  
I/O 🔌, firmware 🔧, and systems that actually run 🚀

---

## Overview

This example demonstrates deterministic control of:

- Servos (with per‑channel calibration support)
- DC motors
- Stepper motors

using the **Kitronik Robotics board with PCA9685 PWM controller** over I2C.

The RP2040 communicates via I2C to a dedicated PWM controller, allowing precise timing and reliable multi‑channel motor and servo control.

New in this version:

- Per‑servo calibration support
- Global servo calibration support
- Direct microsecond pulse control for precision tuning
- Safe mechanical limit enforcement

---

## Example Code

```cpp
#include "pico/stdlib.h"
#include "kitronik_pico_robotics.hpp"

int main() {
    stdio_init_all();
    sleep_ms(200);

    // Initialize Kitronik Robotics board via I2C
    // i2c0, address 0x6C, SDA=8, SCL=9, 100kHz
    KitronikPicoRobotics board(i2c0, 0x6C, 8, 9, 100000);

    // =========================
    // Servo calibration
    // =========================

    // Per‑servo calibration (recommended when servos differ)
    // Limits servo 1 to safe mechanical range
    board.setServoPulseRangeUs(1, 600, 2300);

    // Optional: calibrate all servos globally
    // board.setAllServoPulseRangeUs(500, 2300);

    while (true) {

        // =========================
        // Servo control (degrees)
        // =========================

        board.servoWrite(1, 0);
        board.servoWrite(2, 0);
        sleep_ms(1000);

        board.servoWrite(1, 180);
        board.servoWrite(2, 180);
        sleep_ms(1000);

        // =========================
        // Motor control
        // =========================

        board.motorOn(1, 'f', 60);
        sleep_ms(500);

        board.motorOn(1, 'r', 40);
        sleep_ms(500);

        board.motorOff(1);
        sleep_ms(500);

        // =========================
        // Stepper control
        // =========================

        board.step(1, 'f', 100, 20, false);
        sleep_ms(500);

        board.step(1, 'r', 100, 20, false);
        sleep_ms(500);

        board.stepAngle(1, 'f', 90.0f, 20, false, 200);
        sleep_ms(1000);
    }
}
```

---

## Servo Calibration Explained

Servos vary between manufacturers and models. Some may:

- Not reach full 0–180° range
- Hit mechanical limits early
- Require adjusted pulse widths

This library allows safe calibration.

### Per‑servo calibration

```cpp
board.setServoPulseRangeUs(servo, min_us, max_us);
```

Example:

```cpp
board.setServoPulseRangeUs(1, 600, 2300);
```

This prevents mechanical overtravel for that servo.

---

### Global calibration

```cpp
board.setAllServoPulseRangeUs(min_us, max_us);
```

Applies calibration to all servos.

---

### Direct pulse control (advanced)

```cpp
board.servoWriteUs(1, 1500); // center position
```

Useful for:

- Calibration
- Precision positioning
- Non‑standard servos

---

## Hardware Architecture

| Device   | Method |
|--------|--------|
| Servos | PCA9685 |
| Motors | PCA9685 |
| Steppers | PCA9685 |
| Communication | I2C |

---

## I2C Wiring

| Signal | Pico Pin |
|------|-----------|
| SDA  | GP8 |
| SCL  | GP9 |
| Address | 0x6C |

---

## Build and Flash

```bash
cmake --preset pico2-release
cmake --build --preset build-pico2-release
picotool load build/pico2-release/app.uf2 -f
picotool reboot
```

---

## When to use this version

Use this firmware when:

- Using Kitronik Robotics board with PCA9685
- Multiple servos require calibration
- Different servo brands are used together
- Precise motor and stepper control is required

---

## Author

axfer‑io  
Embedded systems · Control · Debugging

---

## License

MIT

