#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// #include "PID_controller/Magnetic_angle_sensor.h"
// #include "PID_controller/PID_controller.h"
#include <AS5600.h>

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
  
  
  // Create an instance of MotorController
  MotorController motor(11, 12);  // PWM on pin 11, direction on pin 12
  
  // MagneticAngleSensor sensor; // Create an instance of the class
AS5600 sensor;

int PWMValue = 0; //0-255 PWM value for speed, external PWM boards can go higher (e.g. PCA9685: 12-bit => 0-4095)


void setup() {

    Serial.begin(115200);
    delay(2000); //Delay start of script so that python script can catch beginning.
    Wire.begin(); //start i2C
    Wire.setClock(400000L); //faster clock speed
    while (!Serial); // Wait for Serial Monitor
    startTime = micros();  // Store start time in microseconds
    uint8_t as5600_addr = 0;
    int control_reg_0, control_reg_1;

    //Starting Sensor
    Serial.println("Initializing AS5600...");
    bool ok = sensor.begin();  // Only call once!
    
    if (!ok) {
        Serial.println("Failed to initialize AS5600 sensor.");
    } else {
        Serial.println("AS5600 initialized.");
    }

    int b = sensor.isConnected();  // Check connection status
    control_reg_0 = sensor.getConfigure();  // Read current CONF register

    sensor.setConfigure(0x1F00);  // Fast Filter Threshold = 10 LSBs, Slow Filter = 2x, rest default

    control_reg_1 = sensor.getConfigure();  // Read CONF register again to verify

    as5600_addr = sensor.getAddress();  // Get I2C address

    Serial.print("AS5600 Connect: ");
    Serial.print(b);
    Serial.print(", addr=0x");
    Serial.print(as5600_addr, HEX);
    Serial.print(", confb4=0x");
    Serial.print(control_reg_0, HEX);
    Serial.print(", confaf=0x");
    Serial.println(control_reg_1, HEX);

    delay(1000);

    // Initialize the motor
    motor.begin();

}



float prevAngle = 0.0;
int rotationCount = 0;

//Code to check how often get_angle gets called
unsigned long last = 0;

void get_angle()
{
    unsigned long now = micros();
    // Serial.println(now - last);  // prints loop time in µs
    last = now;
    // 1. Read current angle from AS5600
    float currentAngle = sensor.readAngle()* AS5600_RAW_TO_DEGREES;  // in degrees (0 to 360)

    // 2. Calculate difference from previous angle
    float AngleDelta = currentAngle - prevAngle;

    // Detect when sensor moves from ~360° back to ~0° (clockwise rotation)
    if (AngleDelta < -300 ) {
        rotationCount++; // Increase rotation count
    }
    // Detect when sensor moves from ~0° back to ~360° (counterclockwise rotation)
    else if (AngleDelta > 300) {
        rotationCount--; // Decrease rotation count (if reversing)
    }

    // 4. Save current angle as previous for next loop
    prevAngle = currentAngle;

}


void Step_response() {
    unsigned long currentTime = micros();
    
    // Apply step only once at the beginning
    if (!step_applied && currentTime - startTime < step_duration) {
        motor.moveForward(stepPWM);
        step_applied = true;
    }

    // Stop motor after step_duration
    if (currentTime - startTime >= step_duration) {
        motor.stop();
    }
}





void loop() {
    get_angle();  // Update rotationCount and prevAngle

    Step_response();  // Apply step and stop after duration

    // Print data at fixed rate (10 Hz here)
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();
    float print_Hz = 500;  // Print rate (adjust as needed)

    if (now - lastPrintTime >= 1000 / print_Hz) {
        lastPrintTime = now;
        unsigned long elapsedTime = micros() - startTime;

        // Compute total angle (including rotation count) in degrees or radians
        float absoluteAngle = rotationCount * 360.0 + prevAngle;  // degrees
        float absoluteAngle_rad = absoluteAngle * DEG_TO_RAD;     // radians

        // Print: time(s), angle(rad)
        Serial.print(elapsedTime / 1000000.0, 6);  // time in seconds
        Serial.print(", ");
        Serial.println(absoluteAngle_rad, 6);  // angle in radians
    }

    // Stop entire loop after test ends (for safety/logging)
    unsigned long totalElapsed = micros() - startTime;
    if (totalElapsed >= 2 * step_duration) {  // Add some buffer
        motor.stop();
        Serial.println("END");
        while (true);  // Freeze loop
    }
}

