

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
#include <math.h>
#include "PID_controller/Magnetic_angle_sensor.h"
#include "PID_controller/PID_controller.h"
// Create an instance of MotorController
MotorController motor(4, 7);  // PWM on pin 4, direction on pin 7


MagneticAngleSensor sensor; // Create an instance of the class

// // Defining speed
// int motorSpeed = 200; // duty cycle (0/255)
// int rotationsLimit = 10; // Number of rotations to perform

int PWMValue = 0; //0-255 PWM value for speed, external PWM boards can go higher (e.g. PCA9685: 12-bit => 0-4095)


// PID controller parameters
float proportional = 0.5; //k_p = 0.5
float integral = 3; //k_i = 3
float derivative = 1; //k_d = 1
float controlSignal = 0; //u - Also called as process variable (PV)

//PID-related
float previousTime = 0; //for calculating delta t
float previousError = 0; //for calculating the derivative (edot)
float errorIntegral = 0; //integral error
float currentTime = 0; //time in the moment of calculation
float deltaTime = 0; //time difference
float errorValue = 0; //error
float edot = 0; //derivative (de/dt)


// Targets
float targetPosition = 0.1 ; //target position in [m]

//Measured values
volatile float Position = 0; //position based on the encoder
float previousPosition = -1; //helps to keep track of changes (useful for the display update)



void setup() {
    Serial.begin(115200);
    Wire.begin(); //start i2C
    Wire.setClock(800000L); //faster clock speed
    while (!Serial); // Wait for Serial Monitor

    delay(1000);

    // Initialize the motor
    motor.begin();


    Serial.println("Initializing AS5600...");
    if (!sensor.begin()) {
        Serial.println("Failed to initialize AS5600 sensor.");
    }
}



float prevAngle = 0.0;
int rotationCount = 0;

void get_angle()
{
    // 1. Read current angle from AS5600
    float currentAngle = sensor.getAngle(); // in degrees (0 to 360)

    // 2. Calculate difference from previous angle
    float AngleDelta = currentAngle - prevAngle;

    // Detect when sensor moves from ~360° back to ~0° (clockwise rotation)
    if (prevAngle > 300 && currentAngle < 60) {
        rotationCount++; // Increase rotation count
    }
    // Detect when sensor moves from ~0° back to ~360° (counterclockwise rotation)
    else if (prevAngle < 60 && currentAngle > 300) {
        rotationCount--; // Decrease rotation count (if reversing)
    }

    // 4. Save current angle as previous for next loop
    prevAngle = currentAngle;

    // 5. Print angle info
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print("° | Angle Delta: ");
    Serial.print(AngleDelta);
    Serial.print("° | Rotation Count: ");
    Serial.println(rotationCount);

}


void calculate_PID() {

  float rotationCount_rad = rotationCount * 2 * PI; //convert the rotation count to radians

  // Defining static wire characteristics
  float K = 300000; // [N/m] Stifness of wire
  float Fi = 0; // static load [N]
  float L = 0.65; // original wire length in [m]
  float Lc = L  + Fi/K; // wire length in [m] with load
  float r0 = 0.28 * pow(10, -3);  // wire diameter in [m]

  // init variables 
    float  X = Lc;
    float rvar = r0;
    float Position = 0;
// Dynamic characteristics
if (rotationCount < 1) {
    X = Lc;
    rvar = r0;
    Position = 0;

    Serial.println("[DEBUG] Rotation count < 1 → setting initial values.");
    Serial.print("X = "); Serial.println(X, 6);
    Serial.print("rvar = "); Serial.println(rvar, 6);
    Serial.print("Position = "); Serial.println(Position, 6);
} else {
    rvar = r0 * sqrt(Lc / X);

    // Check whether Rvar is inf because X is not yet calculated
    if (isinf(rvar)) {
        rvar = r0;
    }

        X = sqrt(pow(Lc, 2) - pow(rotationCount_rad * rvar, 2));  // length of wire in [m]
   
        float Delta_X = Lc - X; // Delta_X is zero at start when X = Lc, then increases as wire and X contracts.
    Position = Delta_X;

    Serial.println("[DEBUG] Rotation count ≥ 1 → calculating wire contraction.");
    Serial.print("rotationCount_rad = "); Serial.println(rotationCount_rad, 6);
    Serial.print("rvar = "); Serial.println(rvar, 6);
    Serial.print("X = "); Serial.println(X, 6);
    Serial.print("Delta_X (Position) = "); Serial.println(Position, 6);
}



  //Determining the elapsed time
  currentTime = micros(); //current time
  deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  //---
  errorValue = Position - targetPosition; //Current position - target position (or setpoint)

  edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term

  errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

  previousError = errorValue; //save the error for the next iteration to get the difference (for edot)
}

void DriveMotor()
    {
        //Speed
        PWMValue = (int)fabs(controlSignal); //PWM values cannot be negative and have to be integers
        if (PWMValue > 255) //fabs() = floating point absolute value
        {
          PWMValue = 255; //capping the PWM signal - 8 bit
        }
      
        if (PWMValue < 200 && errorValue != 0)
        {
          PWMValue = 200;
        }

        //Determine speed and direction based on the value of the control signal
        //direction
        if (controlSignal < 0) //negative value: CCW
        {
            motor.moveReverse(PWMValue);
        }
        else if (controlSignal > 0) //positive: CW
        {
            motor.moveForward(PWMValue);

        }
        else //0: STOP - this might be a bad practice when you overshoot the setpoint
        {
            motor.stop();
        }
   
  //Optional printing on the terminal to check what's up
  
        Serial.print(" |ErrorValue: ");
        Serial.print(errorValue);
        Serial.print(" |PWMValue: ");
        Serial.print(PWMValue);
        Serial.print(" |TargetPosition: ");
        Serial.print(targetPosition);
        Serial.print(" |Position: ");
        Serial.print(Position);
        Serial.println();

    }



void loop() {

    // Sensor readout
    get_angle();

    //PID controller
    calculate_PID();

    //Driving motor
    DriveMotor();

}



// void loop() {
//     static int rotationCount = 0; // Stores number of full rotations
//     static float lastAngle = 0;   // Stores previous angle to detect overflow
//     float currentAngle = sensor.getAngle(); // Get current angle
  
//     // Print sensor data
//     Serial.print("Angle: ");
//     Serial.print(currentAngle);
//     Serial.print("\tRotations: ");
//     Serial.println(rotationCount);

//   // Detect when sensor moves from ~360° back to ~0° (clockwise rotation)
//   if (lastAngle > 300 && currentAngle < 60) {
//       rotationCount++; // Increase rotation count
//   }
//   // Detect when sensor moves from ~0° back to ~360° (counterclockwise rotation)
//   else if (lastAngle < 60 && currentAngle > 300) {
//       rotationCount--; // Decrease rotation count (if reversing)
//   }

//   lastAngle = currentAngle; // Store the last angle for the next loop

//   // Move motor forward
//   motor.moveForward(motorSpeed);
//   delay(2000);

//   // Stop motor after 100 rotations
//   if (rotationCount >= rotationsLimit) {
//       motor.stop();
//       Serial.println("Motor stopped after 100 rotations.");
//       while (true); // Stop execution
//   }
//   // Reverse motor direction
//   motor.moveReverse(motorSpeed);
//   delay(2000);

//   // Stop motor after returning to starting position
//   if (rotationCount == 0) {
//       motor.stop();
//       Serial.println("Motor stopped after returning to start.");
//       while (true); // Stop execution
//   }

//   motor.stop();
//   delay(1000);


  // Keeping loop stuck so it only runs once
//   while(true);
// }




