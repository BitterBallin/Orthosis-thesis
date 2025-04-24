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
  

    // Declare these globally or at the top of your file
bool step_applied = false;
unsigned long stepStartTime = 0;
float step_duration = 5.0;         // seconds
float stepPWM = 255.0;             // PWM value (0–255)


  // Call this function in setup() or before the test
void startStepResponse() {
    step_applied = false;
    stepStartTime = micros();     // Time in microseconds
}

// Create an instance of MotorController
MotorController motor(11, 12);  // PWM on pin 11, direction on pin 12
const int pwmPin = 11;
const int dirPin = 12;
  // MagneticAngleSensor sensor; // Create an instance of the class
AS5600 sensor;

int PWMValue = 0; //0-255 PWM value for speed, external PWM boards can go higher (e.g. PCA9685: 12-bit => 0-4095)

unsigned long startTime = 0;

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

    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    startStepResponse();

    // Initialize the motor
    motor.begin();
    startStepResponse();  // Start timing when the test begins


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



// This gets called repeatedly in loop()
void Step_response() {
    unsigned long currentTime = micros();
    float elapsedTime = (currentTime - stepStartTime) / 1000000.0; // Convert to seconds

    if (elapsedTime < step_duration) {
        motor.moveForward((int)stepPWM);  // Keep motor ON
    } else {
        motor.stop();  // Stop it after step duration
    }
}


// void Step_response() {
//     unsigned long currentTime = micros();
//     float elapsedTime = (currentTime - stepStartTime) / 1000000.0;

//     if (elapsedTime < step_duration) {
//         digitalWrite(dirPin, HIGH);        // Forward direction
//         analogWrite(pwmPin, stepPWM);      // Constant PWM
//     } else {
//         analogWrite(pwmPin, 0);            // Stop motor
//     }
// }






void loop() {
    get_angle();
    Step_response();

    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();
    float print_Hz = 500;

    if (now - lastPrintTime >= 1000 / print_Hz) {
        lastPrintTime = now;
        unsigned long elapsedTime = micros() - startTime;

        float absoluteAngle = rotationCount * 360.0 + prevAngle;
        float absoluteAngle_rad = absoluteAngle * DEG_TO_RAD;

        Serial.print(elapsedTime / 1000000.0, 6);
        Serial.print(", ");
        Serial.println(absoluteAngle_rad, 6);
    }

    // ❗ FIXED CONDITION:
    unsigned long totalElapsed = micros() - stepStartTime;
    if (totalElapsed >= (unsigned long)(2 * step_duration * 1e6)) {
        motor.stop();
        Serial.println("END");
        while (true);
    }
}


