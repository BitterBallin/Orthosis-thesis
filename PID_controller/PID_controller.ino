// Basic LED Blink Test
// Blinks the built-in LED every second

// void setup() {
//     pinMode(LED_BUILTIN, OUTPUT); // Set built-in LED pin as output
//   }
  
//   void loop() {
//     digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
//     delay(100);                     // Wait 1 second (1000 = 1 second)
//     digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
//     delay(100);                     // Wait 1 second
//   }


// int pwmPin = 4; // Use a PWM-capable pin
// int dirPin = 3; // Direction pin
// int speed = 25; // PWM value (0-255) ~10% duty cycle

// void setup() {
//   pinMode(dirPin, OUTPUT);
//   pinMode(pwmPin, OUTPUT);
// }

// void loop() {
//   //  Move Forward
//   digitalWrite(dirPin, HIGH);
//   analogWrite(pwmPin, speed);
//   delay(2000);

//   // Stop
//   analogWrite(pwmPin, 0);
//   delay(1000);

//   // Reverse
//   digitalWrite(dirPin, LOW);
//   analogWrite(pwmPin, speed);
//   delay(2000);

//   // Stop again
//   analogWrite(pwmPin, 0);
//   delay(1000);
// }


int pwmPin = 4; // PWM pin (make sure it's ~ capable)
int dirPin = 3; // Direction pin
int speed = 0; // PWM value (0-255) ~40% duty cycle

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
}

void loop() {
  // Move Forward
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, speed);
  delay(2000);

  // Stop
  analogWrite(pwmPin, 0);
  delay(1000);

  // Reverse
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, speed);
  delay(2000);

  // Stop again
  analogWrite(pwmPin, 0);
  delay(1000);
}


// void setup() {
//   pinMode(dirPin, OUTPUT);
//   pinMode(pwmPin, OUTPUT);

//   digitalWrite(dirPin, HIGH);   // Set direction
//   analogWrite(pwmPin, 0);     // PWM value (0-255)
// }

// void loop() {
//   // Do nothing, just keep motor running
// }
