#include "pico/stdlib.h"
#include "kitronik_pico_robotics.hpp"

int main() {
    stdio_init_all();
    sleep_ms(200);

    // Inicializa la board I2C PCA9685
    // i2c0, dirección 0x6C (108), SDA=8, SCL=9, 100kHz
    KitronikPicoRobotics board(i2c0, 0x6C, 8, 9, 100000);

    while (true) {

        // =========================
        // SERVO (1..8)
        // =========================

        board.servoWrite(1, 0);
        sleep_ms(500);
        board.servoWrite(1, 180);
        sleep_ms(500);
        board.servoWrite(1, 90);
        sleep_ms(500);


        // =========================
        // MOTOR DC (1..4)
        // =========================

        // Motor 1 forward 60%
        board.motorOn(1, 'f', 60);
        sleep_ms(500);
        // Motor 1 reverse 40%
        board.motorOn(1, 'r', 40);
        sleep_ms(500);
        // Apagar motor
        board.motorOff(1);
        sleep_ms(500);


        // =========================
        // STEPPER (1 o 2)
        // =========================

        // Stepper 1 forward, 100 pasos
        board.step(1, 'f', 100, 20, false);
        sleep_ms(500);
        // Stepper 1 reverse, 100 pasos
        board.step(1, 'r', 100, 20, false);
        sleep_ms(500);

        // También puedes usar ángulo:
        board.stepAngle(1, 'f', 90.0f, 20, false, 200);
        sleep_ms(1000);
    }
}
