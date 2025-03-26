#include "PID_controller.h"

// Constructor: Store pin values
MotorController::MotorController(int pwmPin, int dirPin) {
    _pwmPin = pwmPin;
    _dirPin = dirPin;
}

// Initialize motor pins
void MotorController::begin() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
}

// Move forward at a given speed
void MotorController::moveForward(int speed) {
    digitalWrite(_dirPin, HIGH);  // Set direction to forward
    analogWrite(_pwmPin, speed);  // Set PWM speed
}

// Move in reverse at a given speed
void MotorController::moveReverse(int speed) {
    digitalWrite(_dirPin, LOW);   // Set direction to reverse
    analogWrite(_pwmPin, speed);  // Set PWM speed
}

// Stop the motor
void MotorController::stop() {
    analogWrite(_pwmPin, 0);  // Set speed to zero
}
