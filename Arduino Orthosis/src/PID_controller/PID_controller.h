#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    // Constructor: Initializes the motor pins
    MotorController(int pwmPin, int dirPin);

    // Initializes the motor (sets pin modes)
    void begin();

    // Moves the motor forward at a given speed
    void moveForward(int speed);

    // Moves the motor in reverse at a given speed
    void moveReverse(int speed);

    // Stops the motor
    void stop();

private:
    int _pwmPin; // PWM pin for speed control
    int _dirPin; // Direction pin
    int _speed;  // Current speed
};

#endif // PID_CONTROLLER_H
