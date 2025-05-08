#include <Arduino.h>

// Constants
const int tipLoadCellPin = A2;     // Fingertip force sensor pin
const float ramp30 = 5.9069;       // Calibration gain in N/V

void setup() {
  Serial.begin(115200);            // Start serial monitor
  delay(1000);                     // Optional delay to ensure serial connects
  Serial.println("Fingertip Force Sensor Readout (A2)");
}

void loop() {
  int raw = analogRead(tipLoadCellPin);            // Raw ADC value (0-1023)
  float voltage = (raw / 1023.0) * 5.0;             // Convert to voltage
  float force = voltage * ramp30;                  // Convert to Newtons

  // Print the values
  Serial.print("Raw ADC: ");
  Serial.print(raw);
  Serial.print("  |  Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V  |  Force: ");
  Serial.print(force, 2);
  Serial.println(" N");

  delay(250);  // Read 4 times per second
}
