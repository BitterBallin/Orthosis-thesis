#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// #include "PID_controller/Magnetic_angle_sensor.h"
// #include "PID_controller/PID_controller.h"
#include <AS5600.h>

const int loadCellPin = A1;  // Analog pin for force
float forceValue = 0.0;      // Processed force in Newtons (after scaling)
float analogReadValue = 0.0;



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



// Initiate time variable
unsigned long startTime;
unsigned long t0;

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

    t0 = millis();  // set t=0 for motion profile


}





float readForce() {
    int raw = analogRead(loadCellPin);  // 0–1023
    
    float restsignal = 300; // Signal to A0 at 0 excitation
    float force = ((raw - restsignal)/(1023.0 - restsignal))*200;

    return raw;
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




// Defining static wire characteristics
float K = 300000; // [N/m] Stifness of wire
float Fi = 10; // static load [N]
float braid_factor = 1; //Increasing wire length based on braided wire structure to compensate real life length
float L = 1.7*braid_factor; // original wire length in [m]
float Lc = L  + Fi/K; // wire length in [m] with load
float r0 = 0.28 * pow(10, -3);  // wire diameter in [m]

// init variables 
double  X = Lc;
float rvar = r0;
double Position = 0;

// Derivative values for output usage instead of error
double PreviousPosition = 0;
double Delta_ouput = 0;

// Targets
// float targetPosition = 0.05 ; //target position in [m]
// float reverse_threshold = 0.0005; //Threshold for reversing motion in [m]
// bool goingForward = true;  // Direction flag

// Smoothing of target
float target_max = 0.08;  // peak target (meters)
// unsigned long t0 = 0;     // start time (set in setup)
float t_ramp = 2.0;       // ramp time in seconds
float t_hold = 7.5;       // hold time in seconds
float t_total = 2*t_ramp + t_hold;

float smoothed_target = 0;


// PID controller parameters 
// need to be scaled from error to pwm value, from 0.1~ to 0 - 255
float proportional = 1.5*255/target_max; //k_p = 0.5
float integral = 1000; //k_i = 3
float derivative = 2500; //k_d = 1
float controlSignal = 0; //u - Also called as process variable (PV)

//PID-related
float previousTime = 0; //for calculating delta t
float previousError = 0; //for calculating the derivative (edot)
float errorIntegral = 0; //integral error
float currentTime = 0; //time in the moment of calculation
float deltaTime = 0; //time difference
float errorValue = 0; //error
float edot = 0; //derivative (de/dt)
float DeltaError = 0; //

//Filtering logic for derivative
double filtered_edot = 0;
double previous_filtered_edot = 0;
double tau_d = 0.05;  // 20 ms time constant — tune this!

//Filtering for complete control signal
float filteredControlSignal = 0;


void updateSmoothedTarget() {
    float t_now = (millis() - t0) / 1000.0;  // seconds since start

    if (t_now < 0) {
        smoothed_target = 0;
    } else if (t_now < t_ramp) {
        // Ramp up
        smoothed_target = target_max * 0.5 * (1 - cos(PI * t_now / t_ramp));
    } else if (t_now < t_ramp + t_hold) {
        // Hold
        smoothed_target = target_max;
    } else if (t_now < t_total) {
        // Ramp down
        float t_down = t_now - (t_ramp + t_hold);
        smoothed_target = target_max * 0.5 * (1 + cos(PI * t_down / t_ramp));
    } else {
        // Finished
        smoothed_target = 0;
    }
}

void calculate_PID() {


    // float rotationCount_rad = rotationCount * 2 * PI; //convert the rotation count to radians
    float angle = sensor.readAngle() * AS5600_RAW_TO_RADIANS;  // use raw angle in radians
    float rotationCount_rad = (rotationCount * 2 * PI) + angle;
    //Determining the elapsed time
    currentTime = micros(); //current time
    static bool pidInitialized = false;
    if (!pidInitialized) {
        previousTime = currentTime;
        errorIntegral = 0;
        pidInitialized = true;
        PreviousPosition = 0;
        return;  // Skip this first call, everything is now set up
    }

    // Dynamic characteristics
    if (abs(rotationCount) < 1) {
        X = Lc;
        rvar = r0;
        Position = 0;
    } else {
        rvar = r0 * sqrt(Lc / X);

        // Check whether Rvar is inf because X is not yet calculated
        if (isinf(rvar)) {
            rvar = r0;
        }

            X = sqrt(pow(Lc, 2) - (pow(rotationCount_rad, 2)*pow(rvar,2)));  // length of wire in [m]
    
            float Delta_X = Lc - X; // Delta_X is zero at start when X = Lc, then increases as wire and X contracts.
            Position = Delta_X;
}


    deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
    previousTime = currentTime; //save the current time for the next iteration to get the time difference
    //---
    // Calling smoothed target function

    // updateSmoothedTarget();
    errorValue = -Position + smoothed_target;

    // Making the derivative based on the output instead of error to prevent spikes
    Delta_ouput = Position - PreviousPosition;


    //Current position - target position (or setpoint)
    // DeltaError = errorValue - previousError;  


    //Reversing logic of target, now obsolete due to target smoothing function
    // if (goingForward && abs(errorValue) < reverse_threshold) {
    //     goingForward = false;
    //     targetPosition = 0.0;
    //     errorIntegral = 0;
    //     previousError = errorValue;

    // } 



    // Calculate raw edot
    edot = - Delta_ouput / deltaTime;

    // Low-pass filter on edot
    double alpha = deltaTime / (tau_d + deltaTime);
    filtered_edot = alpha * edot + (1 - alpha) * previous_filtered_edot;
    previous_filtered_edot = filtered_edot;


    // if(goingForward==false){
    //     edot = -edot;
    // }

    // Dump integral sum at target
    if (abs(errorValue) < 0.00005) {
        // errorIntegral *= 0.995;
        errorIntegral =0;
        edot = 0;
        // edot = 0;
    }

    errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

    controlSignal = (proportional * errorValue) + (derivative * filtered_edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

    // // Control signal filtering for smoother control
    // float alpha_c = 0.1;  // smoothing factor: 0.1 = heavy smoothing, 0.9 = minimal smoothing

    // filteredControlSignal = alpha_c * controlSignal + (1 - alpha_c) * filteredControlSignal;

    // controlSignal = filteredControlSignal;


    // controlSignal = 0;
    if (Position != PreviousPosition) {
        PreviousPosition = Position;
    }

}

void DriveMotor()



    {
        //Speed
        PWMValue = (int)fabs(controlSignal); //PWM values cannot be negative and have to be integers

        //Uncomment to reverse motor
        //------------------------------------------
        // PWMValue = -(int)fabs(controlSignal);

        if (PWMValue > 255) //fabs() = floating point absolute value
        {
          PWMValue = 255; //capping the PWM signal - 8 bit
        }

      
        // if (PWMValue < -255) //fabs() = floating point absolute value
        // {
        //   PWMValue = -255; //capping the PWM signal - 8 bit
        // }

        // if (PWMValue < 90 && abs(errorValue) > 0.0001)
        // {
        //   PWMValue = 90;
        // }

        // if (PWMValue < -90 && errorValue != 0)
        // {
        //   PWMValue = -90;
        // }

        //Determine speed and direction based on the value of the control signal
        //direction
        if (controlSignal<0) //negative value: CCW
        {
            motor.moveReverse(PWMValue);
            // Serial.print("Reversing motor");
        }
        else if (controlSignal>0) //positive: CW
        {
            motor.moveForward(PWMValue);
            // Serial.print("Forward movement motor");
        }
        // else //0: STOP - this might be a bad practice when you overshoot the setpoint
        // {
        //     motor.stop();
        // }

    }

    void loop() {

        // Sensor readout
        get_angle();
    
        // PID controller
        updateSmoothedTarget();
        calculate_PID();
    
        // Driving motor
        DriveMotor();

        // Readout force sensor
        forceValue = readForce();


        static unsigned long lastPrintTime = 0;
        unsigned long now = millis();
        float print_Hz = 10;

        if (now - lastPrintTime >= 1000/print_Hz) {
            // updateSmoothedTarget();
            lastPrintTime = now;
            unsigned long elapsedTime = micros() - startTime;
            Serial.print(elapsedTime / 1000000.0, 2); // in seconds with 2 decimal places
            Serial.print(",");
            Serial.print(errorValue,5);
            Serial.print(",");
            // Serial.print("Force Value:");
            Serial.print("raw force A0:");

            Serial.print(forceValue, 5);  // Add to the existing serial data
            Serial.print(",");
            Serial.print(controlSignal);
            Serial.print(",");
            Serial.print(Position,5);
            // Serial.print(",");
            // Serial.println(rotationCount * 2 * PI, 5);
            Serial.print(",");
            Serial.print(smoothed_target);
            Serial.print(",");
            // Serial.print("DeltaTime: ");
            // Serial.print(deltaTime, 10);
            // Serial.print(",");
            Serial.print(proportional*errorValue);
            Serial.print(",");
            Serial.print(errorIntegral*integral);

            // float currentAngle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;

            // Serial.print("Raw Angle: ");
            // Serial.print(currentAngle, 2);
            // Serial.print("\tRotationCount: ");
            // Serial.println(rotationCount);
            // Serial.print("RotationCount: "); Serial.print(rotationCount);
            // Serial.print("\tPosition: "); Serial.print(Position, 10);
            // Serial.print("\tPreviousPosition: "); Serial.print(PreviousPosition, 10);
            // Serial.print("\tDelta_output: "); Serial.println(Delta_ouput, 6);
            // Serial.print(raw);

            Serial.print(",");
            // Serial.print("Edot: ");
            Serial.println(filtered_edot*derivative, 10);

        }

      //  Stop after 10 seconds
       unsigned long elapsedTime = micros() - startTime;
       if (elapsedTime >= 20000000UL) {
           motor.stop();
           Serial.println("END");
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






// // PWM SIGNAL TEST CODE, HIGH VS 255
// //--------------------------------------------------------
// #include <Arduino.h>
// #include <Wire.h>
// #include <AS5600.h>

// AS5600 sensor;

// // Motor pins
// const int pwmPin = 11;
// const int dirPin = 12;

// // Test config
// const unsigned long testDuration = 10 * 1000000UL; // 10 seconds in microseconds
// bool useAnalogWrite = false; // Set to false to use digitalWrite(HIGH)
// bool testRunning = false;

// // Rotation tracking
// float prevAngle = 0.0;
// int rotationCount = 0;
// unsigned long startTime;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   sensor.begin();

//   pinMode(pwmPin, OUTPUT);
//   pinMode(dirPin, OUTPUT);

//   digitalWrite(dirPin, HIGH); // Set forward direction
//   delay(1000);

//   // Read initial angle
//   prevAngle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;

//   // Start motor
//   if (useAnalogWrite) {
//     analogWrite(pwmPin, 255);  // PWM 100% duty cycle
//     Serial.println("Running test with analogWrite(255)");
//   } else {
//     digitalWrite(pwmPin, HIGH);  // Constant HIGH
//     Serial.println("Running test with digitalWrite(HIGH)");
//   }

//   startTime = micros();
//   testRunning = true;
// }

// void loop() {
//   if (!testRunning) return;

//   unsigned long now = micros();
//   float currentAngle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;
//   float angleDelta = currentAngle - prevAngle;

//   // Handle wraparound
//   if (angleDelta < -300) rotationCount++;
//   else if (angleDelta > 300) rotationCount--;

//   prevAngle = currentAngle;

//   // Stop after test duration
//   if (now - startTime >= testDuration) {
//     if (useAnalogWrite) {
//       analogWrite(pwmPin, 0);
//     } else {
//       digitalWrite(pwmPin, LOW);
//     }

//     Serial.println("Test completed.");
//     Serial.print("Rotation count: ");
//     Serial.println(rotationCount);

//     testRunning = false;
//   }
// }
// //------------------------------------------------------





// //MOTOR Driver buttons torque test
// //---------------------------------------
// #include <Arduino.h>
// #include <Wire.h>
// #include <AS5600.h>

// AS5600 sensor;

// float prevAngle = 0.0;
// int rotationCount = 0;
// unsigned long startTime;
// const unsigned long testDuration = 10 * 1000000UL; // 5 seconds in microseconds

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   sensor.begin();

//   delay(10000); // Wait for sensor to stabilize
//   prevAngle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;

//   Serial.println("Ready! Press the MD13S test buttons now...");
//   startTime = micros();
// }

// void loop() {
//   unsigned long now = micros();
//   if (now - startTime >= testDuration) {
//     Serial.println("Test finished!");
//     Serial.print("Total rotations: ");
//     Serial.println(rotationCount);
//     while (true); // Halt here after test
//   }

//   float currentAngle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;
//   float angleDelta = currentAngle - prevAngle;

//   // Wraparound detection
//   if (angleDelta < -300) rotationCount++;
//   else if (angleDelta > 300) rotationCount--;

//   prevAngle = currentAngle;
// }
// //------------------------------------------------------