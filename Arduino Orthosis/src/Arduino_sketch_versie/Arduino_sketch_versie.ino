

// // // #include <Wire.h>

// // // void setup() {
// // //   Serial.begin(115200);
// // //   while (!Serial);  // Wait for Serial Monitor
// // //   delay(1000);      // Delay for serial stability

// // //   Serial.println("Starting I2C Scanner...");
  
// // //   Wire.begin();  // Start I2C communication
  
// // //   for (byte address = 1; address < 127; address++) {
// // //     Wire.beginTransmission(address);
// // //     if (Wire.endTransmission() == 0) {
// // //       Serial.print("Found I2C device at address 0x");
// // //       Serial.println(address, HEX);
// // //     }
// // //   }
  
// // //   Serial.println("Scan complete.");
// // // }

// // // void loop() {}
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
// Create an instance of MotorController
MotorController motor(11, 12);  // PWM on pin 11, direction on pin 12


// MagneticAngleSensor sensor; // Create an instance of the class
AS5600 sensor;

// // Defining speed
// int motorSpeed = 200; // duty cycle (0/255)
// int rotationsLimit = 10; // Number of rotations to perform

int PWMValue = 0; //0-255 PWM value for speed, external PWM boards can go higher (e.g. PCA9685: 12-bit => 0-4095)


// PID controller parameters 
// need to be scaled from error to pwm value, from 0.1~ to 0 - 255
float proportional = 700; //k_p = 0.5
float integral = 100; //k_i = 3
float derivative = 700; //k_d = 1
float controlSignal = 0; //u - Also called as process variable (PV)

//PID-related
float previousTime = 0; //for calculating delta t
float previousError = 0; //for calculating the derivative (edot)
float errorIntegral = 0; //integral error
float currentTime = 0; //time in the moment of calculation
float deltaTime = 0; //time difference
float errorValue = 0; //error
float edot = 0; //derivative (de/dt)




// Initiate time variable
unsigned long startTime;


void setup() {

    Serial.begin(115200);
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

// const int angleHistorySize = 1000;  // Max number of saved angles
// float angleHistory[angleHistorySize];
// int angleIndex = 0;

//Code to check how often get_angle gets called
unsigned long last = 0;

void get_angle()
{
    unsigned long now = micros();
    Serial.println(now - last);  // prints loop time in µs
    last = now;
    // 1. Read current angle from AS5600
    float currentAngle = sensor.readAngle()* AS5600_RAW_TO_DEGREES;  // in degrees (0 to 360)

    // // 2. Save current angle in history array
    // angleHistory[angleIndex] = currentAngle;
    // angleIndex = (angleIndex + 1) % angleHistorySize;  // Wrap around if full

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

    // // 5. Print angle info
    // Serial.print("Current Angle: ");
    // Serial.print(currentAngle);
    // Serial.print("° | Angle Delta: ");
    // Serial.print(AngleDelta);
    Serial.print("° | Rotation Count: ");
    Serial.println(rotationCount);

}




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


// Targets
float targetPosition = 0.1 ; //target position in [m]


void calculate_PID() {
    float rotationCount_rad = rotationCount * 2 * PI; //convert the rotation count to radians

    // Dynamic characteristics
    if (abs(rotationCount) < 1) {
        X = Lc;
        rvar = r0;
        Position = 0;

        Serial.println("[DEBUG] Rotation count < 1 → setting initial values.");
        Serial.print("X = "); Serial.println(X, 6);
        // Serial.print("rvar = "); Serial.println(rvar, 6);
        // Serial.print("Position = "); Serial.println(Position, 6);
    } else {
        rvar = r0 * sqrt(Lc / X);

        // Check whether Rvar is inf because X is not yet calculated
        if (isinf(rvar)) {
            rvar = r0;
        }

            X = sqrt(pow(Lc, 2) - (pow(rotationCount_rad, 2)*pow(rvar,2)));  // length of wire in [m]
    
            float Delta_X = Lc - X; // Delta_X is zero at start when X = Lc, then increases as wire and X contracts.
        Position = Delta_X;

        // Serial.println("[DEBUG] Rotation count ≥ 1 → calculating wire contraction.");
        // Serial.print("rotationCount_rad = "); Serial.println(rotationCount_rad, 6);
        // Serial.print("rvar = "); Serial.println(rvar, 6);
        // Serial.print("X = "); Serial.println(X, 6);
        Serial.print("Delta_X (Position) = "); Serial.println(Position, 6);
}



    //Determining the elapsed time
    currentTime = micros(); //current time
    deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
    previousTime = currentTime; //save the current time for the next iteration to get the time difference
    //---
    errorValue = - Position + targetPosition; //Current position - target position (or setpoint)

    edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term

    errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

    controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

    // controlSignal = 0;

    previousError = errorValue; //save the error for the next iteration to get the difference (for edot)
    Serial.print("errorValue: ");
    Serial.print(errorValue);
    // Serial.print("Control Signal:"); Serial.print(controlSignal);


    // Serial Prints in Plot format
    //-----------------------------
    Serial.print(errorValue);
    Serial.print(",");
    Serial.print(controlSignal);
    Serial.print(",");
    Serial.println(Position);
    //-----------------------------

    // Serial.print(" edot:"); Serial.print(edot);
    // Serial.print(" errorIntegral:"); Serial.print(errorIntegral);
}

void DriveMotor()
    {
        //Speed
        PWMValue = (int)fabs(controlSignal); //PWM values cannot be negative and have to be integers
        if (PWMValue > 255) //fabs() = floating point absolute value
        {
          PWMValue = 255; //capping the PWM signal - 8 bit
        }
      
        if (PWMValue < 90 && errorValue != 0)
        {
          PWMValue = 90;
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
  
        // Serial.print(" |ErrorValue: ");
        // Serial.print(errorValue);
        // Serial.print(" |PWMValue: ");
        // Serial.print(PWMValue);
        // Serial.print("Current time: ");
        // Serial.print(currentTime);
        // Serial.print("Current time: ");
        // Serial.print(micros());
        // Serial.print(" |TargetPosition: ");
        // Serial.print(targetPosition);
        // Serial.print(" |Position: ");
        // Serial.print(Position);
        // Serial.println();

    }

    // void printAngleHistory() {
    //     Serial.println("\n=== Angle History ===");
    //     for (int i = 0; i < 1000; i++) {
    //         Serial.print(i);
    //         Serial.print(": ");
    //         Serial.println(angleHistory[i], 3);  // 3 decimal places
    //     }
    // }

    void loop() {

        // Sensor readout
        get_angle();
    
        // PID controller
        calculate_PID();
    
        // Driving motor
        DriveMotor();
    
        // Stop after 10 rotations
        // if (abs(rotationCount) >= 10) {
        //     motor.stop();
        //     Serial.println("Stopped after 10 rotations.");
        //     while (true);
        // }
    
      //  Stop after 10 seconds
       unsigned long elapsedTime = micros() - startTime;
       if (elapsedTime >= 20000000UL) {
           motor.stop();
           Serial.println("Stopped after 10 seconds.");
        //    printAngleHistory();
          //  Serial.print(rotationCount);
          //  Serial.stop()
           while (true);
       }



       
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


// //TEST CODE FOR SENSOR
// //--------------------------------------------------


// // #include <Wire.h>
// // #include <AS5600.h>

// // AS5600 sensor;

// // void setup() {
// //   Serial.begin(115200);
// //   Wire.begin();
// //   Wire.setClock(100000L);
// //   sensor.begin();

// //   uint8_t as5600_addr = 0;
// //   int control_reg_0, control_reg_1;


// //   int b = sensor.isConnected();  // Check connection status
// //   control_reg_0 = sensor.getConfigure();  // Read current CONF register

// //   sensor.setConfigure(0x0000);  // Default settings

// //   control_reg_1 = sensor.getConfigure();  // Read CONF register again to verify

// //   as5600_addr = sensor.getAddress();  // Get I2C address

// //   Serial.print("AS5600 Connect: ");
// //   Serial.print(b);
// //   Serial.print(", addr=0x");
// //   Serial.print(as5600_addr, HEX);
// //   Serial.print(", confb4=0x");
// //   Serial.print(control_reg_0, HEX);
// //   Serial.print(", confaf=0x");
// //   Serial.println(control_reg_1, HEX);

// // }

// // void loop() {
// //   float angle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;
// //   Serial.println(angle);
// //   delay(1000);
// // }

// // PWM TEST CODE
// //--------------------------------------------------------
// #include <Wire.h>
// #include <Arduino.h>

// const int pwmPin = 11;         // Connect your motor control signal here
// const int minPWM = 0;
// const int maxPWM = 255;
// const int step = 5;           // How much to increase per step
// const int delayTime = 500;    // Time to wait at each step (ms)

// void setup() {
//   pinMode(pwmPin, OUTPUT);
//   Serial.begin(9600);
//   Serial.println("Starting PWM test...");
// }

// void loop() {
//   for (int pwmVal = minPWM; pwmVal <= maxPWM; pwmVal += step) {
//     analogWrite(pwmPin, pwmVal);
//     Serial.print("PWM Value: ");
//     Serial.println(pwmVal);

//     delay(delayTime);

//     // Optional: stop motor after a short delay for safety
//     analogWrite(pwmPin, 0); 
//     delay(300);  // pause between tests
//   }

//   Serial.println("Test complete. Restarting...");
//   delay(3000);  // Wait before restarting test
// }