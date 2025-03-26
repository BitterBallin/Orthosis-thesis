

// #include <Wire.h>

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);  // Wait for Serial Monitor
//   delay(1000);      // Delay for serial stability

//   Serial.println("Starting I2C Scanner...");
  
//   Wire.begin();  // Start I2C communication
  
//   for (byte address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     if (Wire.endTransmission() == 0) {
//       Serial.print("Found I2C device at address 0x");
//       Serial.println(address, HEX);
//     }
//   }
  
//   Serial.println("Scan complete.");
// }

// void loop() {}
#include <Arduino.h>
#include <Wire.h>
#include "PID_controller/Magnetic_angle_sensor.h"
#include "PID_controller/PID_controller.h"

// Create an instance of MotorController
MotorController motor(4, 7);  // PWM on pin 4, direction on pin 7


MagneticAngleSensor sensor; // Create an instance of the class

// Defining speed
int motorSpeed = 0; // 40% duty cycle (102/255)



void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial Monitor
    delay(1000);

    // Initialize the motor
    motor.begin();


    Serial.println("Initializing AS5600...");
    if (!sensor.begin()) {
        Serial.println("Failed to initialize AS5600 sensor.");
    }
}

void loop() {
  static int rotationCount = 0; // Stores number of full rotations
  static float lastAngle = 0;   // Stores previous angle to detect overflow
  float currentAngle = sensor.getAngle(); // Get current angle

  // Print sensor data
  Serial.print("Angle: ");
  Serial.print(currentAngle);
  Serial.print("\tRotations: ");
  Serial.println(rotationCount);

  // Detect when sensor moves from ~360° back to ~0° (clockwise rotation)
  if (lastAngle > 300 && currentAngle < 60) {
      rotationCount++; // Increase rotation count
  }
  // Detect when sensor moves from ~0° back to ~360° (counterclockwise rotation)
  else if (lastAngle < 60 && currentAngle > 300) {
      rotationCount--; // Decrease rotation count (if reversing)
  }

  lastAngle = currentAngle; // Store the last angle for the next loop

  // Move motor forward
  motor.moveForward(motorSpeed);
  delay(2000);

  // Stop motor after 100 rotations
  if (rotationCount >= 100) {
      motor.stop();
      Serial.println("Motor stopped after 100 rotations.");
      while (true); // Stop execution
  }

  // Reverse motor direction
  motor.moveReverse(motorSpeed);
  delay(2000);

  // Stop motor after returning to starting position
  if (rotationCount == 0) {
      motor.stop();
      Serial.println("Motor stopped after returning to start.");
      while (true); // Stop execution
  }

  motor.stop();
  delay(1000);
}




