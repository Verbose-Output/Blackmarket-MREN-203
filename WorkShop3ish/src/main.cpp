// // // Encoder digital pins
// // const byte encoderPinA = 11;
// // const byte encoderPinB = 12;

// // // Counter to keep track of encoder ticks [integer]
// // volatile long encoder_ticks = 0;

// // // Counter to keep track of the last number of ticks [integer]
// // long latestNumTicks = 0;

// // // This function is called when encoderPinA goes HIGH
// // void enocderISR()
// // {
// //     if (digitalRead(encoderPinB) == LOW)
// //     {
// //         // encoderPinA leads encoderPinB, so count one way
// //         encoder_ticks--;
// //     }
// //     else
// //     {
// //         // encoderPinB leads encoderPinA, so count the other way
// //         encoder_ticks++;
// //     }
// // }

// // void setup()
// // {
// //     // Open the serial port at 9600 bps
// //     Serial.begin(9600);

// //     // Set the pin modes for the encoders
// //     pinMode(encoderPinA, INPUT);
// //     pinMode(encoderPinB, INPUT);

// //     // Every time encoderPinA goes HIGH, this is a pulse
// //     attachInterrupt(encoderPinA, enocderISR, RISING);

// //     // Print a message
// //     Serial.print("Program initialized.");
// //     Serial.print("\n");
// // }

// // void loop()
// // {
// //     // Do this if the encoder has moved
// //     if (encoder_ticks != latestNumTicks)
// //     {
// //         // Print some stuff to the serial monitor
// //         Serial.print("Encoder ticks: ");
// //         Serial.print(encoder_ticks);
// //         Serial.print("\n");

// //         // Record the current number of encoder ticks
// //         latestNumTicks = encoder_ticks;
// //     }

// //     // Short delay [ms]
// //     delay(100);
// // }

// // /*
// // The 100 pulses per revolution  + the gear ratio actually results in approx 3000 ticks per output shaft revolution.*/

/*
// Wheel PWM pin (must be a PWM pin)
int EA = 6;
int EB = 5;

// Wheel direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 8;
int I4 = 2;
// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A = 10;
const byte SIGNAL_B = 11;

const byte RIGHT_SIGNAL_A = 12;
const byte RIGHT_SIGNAL_B = 13;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3150;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks++;
    }
    if(digitalRead(RIGHT_SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        right_encoder_ticks++;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        right_encoder_ticks--;

    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the motor driver
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(RIGHT_SIGNAL_A, INPUT);
    pinMode(RIGHT_SIGNAL_B, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(SIGNAL_A, decodeEncoderTicks, RISING);
    attachInterrupt(RIGHT_SIGNAL_A, decodeEncoderTicks, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        // Print some stuff to the serial monitor
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\t");
        Serial.print("Right encoder ticks: ");
        Serial.print(right_encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n");



        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;

    // Select a direction, RIGHT wheels
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);

    //LEFT wheels
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    // PWM command to the motor driver
    analogWrite(EA, u);
    analogWrite(EB, u);
}

#include <Arduino.h>
// Wheel PWM pin (must be a PWM pin)
int EA = 6;
int EB = 5;

// Wheel direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 8;
int I4 = 2;
// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A = 10;
const byte SIGNAL_B = 11;

const byte RIGHT_SIGNAL_A = 12;
const byte RIGHT_SIGNAL_B = 13;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3150;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

//speed Varaibles
int translational_speed_L = 0;
int translational_speed_R = 0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks++;
    }
    if(digitalRead(RIGHT_SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        right_encoder_ticks++;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        right_encoder_ticks--;

    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the motor driver
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(RIGHT_SIGNAL_A, INPUT);
    pinMode(RIGHT_SIGNAL_B, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(SIGNAL_A, decodeEncoderTicks, RISING);
    attachInterrupt(RIGHT_SIGNAL_A, decodeEncoderTicks, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        //Estimate the translational speed
        translational_speed_L = omega_L * 0.0625;
        translational_speed_R = omega_R * 0.0625;

        // Print some stuff to the serial monitor
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\t");
        Serial.print("Right encoder ticks: ");
        Serial.print(right_encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n");

        // Encoder ticks: 0 Estimated left wheel speed: 0.00 rad/s  Right encoder ticks: 0  Estimated right wheel speed: 0.00 rad/s

        //Speedometer Print
        Serial.print("Right wheel speed:");
        Serial.print(translational_speed_R);
        Serial.print(" m/s");
        Serial.print("\n");

        Serial.print("Left wheel speed:" );
        Serial.print(translational_speed_L);
        Serial.print(" m/s");
        Serial.print("\n");


        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;

    // Select a direction, RIGHT wheels
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);

    //LEFT wheels
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    // PWM command to the motor driver
    analogWrite(EA, u);
    analogWrite(EB, u);
}
*/