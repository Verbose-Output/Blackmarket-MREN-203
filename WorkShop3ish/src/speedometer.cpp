#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
// Wheel PWM pin (must be a PWM pin)
int EA = 6;
int EB = 5;

// Wheel direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 8;
int I4 = 2;
// Motor PWM command variable [0-255]
byte U_R = 0;
byte U_L = 0;

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
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

//speed Varaibles
double translational_speed_L = 0.0;
double translational_speed_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 100;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

//vehicle track length l = 00.2775
const double ELL = 0.2775;

//vehicle speed and turn rate
double v = 0;
double omega = 0;

// Varaibles from IMU: ##################################
float omega_x, omega_y, omega_z;
// Gyro biases
float x_bias = -0.18;
float y_bias = -0.31;
float z_bias = -0.43;
// Variables to store accelerations [g's]
float a_x, a_y, a_z;
//variables to store sample rates from sensor [Hz]
float a_f, g_f;

//############## PID PARAMETERS #####################
double v_D = 0.0; // desired vehicle speed [m/s]
double omega_D = 0.0; // desired vehicle turn rate [rad/s]
double v_L = 0.0; // desired left wheel speed [m/s]
double v_R = 0.0; // desired right wheel speed [m/s]


// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        left_encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        left_encoder_ticks++;
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

//compute vehicle speed [m/s]
double compute_vehicle_speed(double translational_speed_L, double translational_speed_R){
    double v;
    v = 0.5 * (translational_speed_L+ translational_speed_R);
    return v;
}

//compute vehicle turn rate [rad/s]
double compute_vehicle_rate(double translational_speed_L, double translational_speed_R){
    double omega;
    omega = 1.0 / ELL * (translational_speed_R - translational_speed_L);
    return omega;
}

double yawRate(const float alpha, const double translational_speed_L, const double translational_speed_R){
    // YawRate = a * Gyro + (1-a) * encoder
    float gyro = (omega_z - z_bias) * PI / 180.0; // Convert to rad/s
    float encoder = compute_vehicle_rate(translational_speed_L, translational_speed_R);
    if (abs(encoder) < 0.06) {
        encoder = 0.0;
    }   
    return alpha * gyro + (1 - alpha) * encoder;
}

//Activity 1 Workshop 3: compute desired wheel parameters
void compute_desired_wheel_parameters(double v_D, double omega_D, double& v_L, double& v_R){
    v_L = v_D - (0.5 * ELL * omega_D); //
    v_R = v_D + (0.5 * ELL * omega_D); //
}

//Activity 2 Workshop 3: compute desired wheel PWM commands
int controller_P_only(double error, double K_P){
    int U;
    U = (int)(K_P * error);
    //saturate U to be between 0 and 255
    if (U > 255){
        U = 255;
        
        // i3, i4 are 0,1
        // i1, i2 are 1,0
    }
    else if (U < 0){
        U = abs(U);
    }
    return U;
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

    if (!IMU.begin())
    {
        // Print an error message if the IMU is not ready
        Serial.print("Failed to initialize IMU :(");
        Serial.print("\n");
        while (1)
        {
            delay(10);
        }
    }

    // Read the sample rate of the accelerometer and gyroscope
    a_f = IMU.accelerationSampleRate();
    g_f = IMU.gyroscopeSampleRate();

    // Print these values to the serial window
    Serial.print("Accelerometer sample rate: ");
    Serial.println(a_f);
    Serial.print("Gyroscope sample rate: ");
    Serial.println(g_f);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(a_x, a_y, a_z);

        // Print the accelerometer measurements to the Serial Monitor
        /*
        Serial.print(a_x);
        Serial.print("\t");
        Serial.print(a_y);
        Serial.print("\t");
        Serial.print(a_z);
        Serial.print(" g\t\t");
        */
    }

    // Read from the gyroscope
    if (IMU.gyroscopeAvailable())
    {
        IMU.readGyroscope(omega_x, omega_y, omega_z);

        // Print the gyroscope measurements to the Serial Monitor
        /*
        Serial.print(omega_x-x_bias);
        Serial.print("\t");
        Serial.print(omega_y-y_bias);
        Serial.print("\t");
        Serial.print(omega_z-z_bias);
        Serial.print(" deg/s\n");
        */
    }

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)left_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        
        //Estimate the translational speed [m/s]
        translational_speed_L = omega_L * RHO;
        translational_speed_R = omega_R * RHO;

        v = compute_vehicle_speed(translational_speed_L, translational_speed_R);
        omega = compute_vehicle_rate(translational_speed_L, translational_speed_R);
        
        Serial.println("Yaw Rate:");
        Serial.print("Combined Yaw Rate:");
        Serial.print(yawRate(0.98, translational_speed_L, translational_speed_R));
        Serial.print("rad/s \n");
        Serial.print("Gyro Only Yaw Rate:");
        Serial.print(omega_z - z_bias);
        Serial.print(" rad/s\n");
        Serial.print("Encoder Only Yaw Rate:");
        Serial.print(compute_vehicle_rate(translational_speed_L, translational_speed_R));
        Serial.print(" rad/s");
        Serial.print("\n");
        
        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    U_R = 128;
    U_L = 128;

    // Select a direction, RIGHT wheels
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);

    //LEFT wheels
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);

    // PWM command to the motor driver
    analogWrite(EA, U_L);
    analogWrite(EB, U_R);
}
