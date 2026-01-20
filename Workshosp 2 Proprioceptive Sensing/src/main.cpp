#include <Arduino.h>

// Encoder digital pins
const byte encoderPinA = 8;
const byte encoderPinB = 9;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;

// Counter to keep track of the last number of ticks [integer]
long latestNumTicks = 0;

// This function is called when encoderPinA goes HIGH
void enocderISR()
{
    if (digitalRead(encoderPinB) == LOW)
    {
        // encoderPinA leads encoderPinB, so count one way
        encoder_ticks--;
    }
    else
    {
        // encoderPinB leads encoderPinA, so count the other way
        encoder_ticks++;
    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the encoders
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

    // Every time encoderPinA goes HIGH, this is a pulse
    attachInterrupt(encoderPinA, enocderISR, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Do this if the encoder has moved
    if (encoder_ticks != latestNumTicks)
    {
        // Print some stuff to the serial monitor
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\n");

        // Record the current number of encoder ticks
        latestNumTicks = encoder_ticks;
    }

    // Short delay [ms]
    delay(100);
}
