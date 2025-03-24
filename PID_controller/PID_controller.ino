// Basic LED Blink Test
// Blinks the built-in LED every second

void setup() {
    pinMode(LED_BUILTIN, OUTPUT); // Set built-in LED pin as output
  }
  
  void loop() {
    digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
    delay(1000);                     // Wait 1 second
    digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
    delay(1000);                     // Wait 1 second
  }
  