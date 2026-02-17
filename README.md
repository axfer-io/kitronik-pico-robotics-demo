# Pico C/C++ Firmware Template --- Kitronik Robotics (I2C PCA9685)

Embedded systems · Control · Debugging ⚙️\
I/O 🔌, firmware 🔧, and systems that actually run 🚀

------------------------------------------------------------------------

## Overview

This example demonstrates control of:

-   Servos
-   DC motors
-   Stepper motors

using the **Kitronik Robotics board with PCA9685 PWM controller** over
I2C.

The RP2040 communicates via I2C to a dedicated PWM driver.

------------------------------------------------------------------------

## Example Code

``` cpp
#include "pico/stdlib.h"
#include "kitronik_pico_robotics.hpp"

int main() {
    stdio_init_all();
    sleep_ms(200);

    KitronikPicoRobotics board(i2c0, 0x6C, 8, 9, 100000);

    while (true) {

        // Servo control
        board.servoWrite(1, 0);
        sleep_ms(500);

        board.servoWrite(1, 180);
        sleep_ms(500);

        // Motor control
        board.motorOn(1, 'f', 60);
        sleep_ms(500);

        board.motorOff(1);
        sleep_ms(500);

        // Stepper control
        board.step(1, 'f', 100, 20, false);
        sleep_ms(1000);
    }
}
```

------------------------------------------------------------------------

## Hardware Architecture

Control method:

  Device          Method
  --------------- ---------
  Servos          PCA9685
  Motors          PCA9685
  Steppers        PCA9685
  Communication   I2C

------------------------------------------------------------------------

## I2C Wiring

  Signal    Pico Pin
  --------- ----------
  SDA       GP8
  SCL       GP9
  Address   0x6C

------------------------------------------------------------------------

## Build and Flash

``` bash
cmake --preset pico2-release
cmake --build --preset build-pico2-release
picotool load build/pico2-release/app.uf2 -f
picotool reboot
```

------------------------------------------------------------------------

## When to use this version

Use this firmware when:

-   Hardware includes PCA9685
-   Robotics board communicates over I2C
-   Multiple servos/motors must be controlled efficiently

------------------------------------------------------------------------

## Author

**axfer-io**  
Embedded systems · Control · Debugging  
Firmware and systems that actually run.

------------------------------------------------------------------------

## License

MIT
