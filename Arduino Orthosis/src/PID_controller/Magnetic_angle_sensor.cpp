#include "Magnetic_angle_sensor.h"
#include <Arduino.h>

MagneticAngleSensor::MagneticAngleSensor() {
    // Constructor - No initialization done here
}

bool MagneticAngleSensor::begin() {
    Wire.begin();
    sensor.begin(4);  // Set direction pin
    sensor.setDirection(AS5600_CLOCK_WISE); // Explicitly set direction

    if (!sensor.isConnected()) {
        Serial.println("AS5600 sensor not detected!");
        return false;
    }
    Serial.println("AS5600 sensor connected!");
    return true;
}

float MagneticAngleSensor::getAngle() {
    return sensor.readAngle()* AS5600_RAW_TO_DEGREES; // Read the sensor angle
}

float MagneticAngleSensor::getRawAngle() {
    return sensor.rawAngle();
}
