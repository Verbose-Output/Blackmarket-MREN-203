#include <Arduino.h>

 // Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN = A0;

// Variables to store the proximity measurement
int sharp_val = 0;    // integer read from analog pin
float sharp_range;   // range measurement [cm]

// ------------------------- Functions ----------------------------------------//
float SharpRead(char pin){
  int value = analogRead(pin);
  float voltage = value * (5000.0 / 1023.0);
  voltage = voltage / 1000.0; // Convert mV to V
  return voltage;
}

float distanceConv(float voltage){
  return pow(voltage / 13.425, -1.261);
}

float dist_Conv2(float voltage){
  return 26.505 * pow(voltage, -1.252);
}

void setup()
{
  // Open the serial port at 115200 bps
  Serial.begin(115200);
}

void loop()
{
  // Read the sensor output (0–1023, which is 10 bits and fits inside an Arduino int-type)
  sharp_val = analogRead(SHARP_PIN);

  // Print all values
  Serial.print(SharpRead(SHARP_PIN));
  Serial.print(", ");
  Serial.print(distanceConv(SharpRead(SHARP_PIN)));
  Serial.print(", ");
  Serial.print(dist_Conv2(SharpRead(SHARP_PIN)));
  Serial.print("\n");

  // Delay for a bit before reading the sensor again
  delay(500);
}
