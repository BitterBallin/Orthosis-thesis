#ifndef MAGNETIC_ANGLE_SENSOR_H
#define MAGNETIC_ANGLE_SENSOR_H

#include <Wire.h>
#include <AS5600.h>

class MagneticAngleSensor {
public:
    MagneticAngleSensor(); // Constructor
    bool begin();          // Initialize sensor
    float getAngle();      // Get the current angle
    float getRawAngle();   // Get raw sensor value

private:
    AS5600 sensor;
};

#endif // MAGNETIC_ANGLE_SENSOR_H
