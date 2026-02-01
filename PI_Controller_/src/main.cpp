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
double v_desired = 1; // desired vehicle speed [m/s]
double omega_D = 0.0; // desired vehicle turn rate [rad/s]
double v_L_desired = 0.0; // desired left wheel speed [m/s]
double v_R_desired = 0.0; // desired right wheel speed [m/s]
const double K_P = 80.0; // Proportional gain for speed controller
double error_L = 0.0; // speed error left wheel [m/s]
double error_R = 0.0; // speed error right wheel [m/s]
int L_output = 0; // Left wheel controller output [0-255]
int R_output = 0; // Right wheel controller output [0-255]

double K_I = 20.0; // Integral gain for speed controller
double e_int_L = 0.0;  // integral of left wheel error
double e_int_R = 0.0;  // integral of right wheel error

const int PWM_MIN = 100; // Minimum PWM to overcome motor deadzone

// This function is called when SIGNAL_A goes HIGH


void leftEncoderISR(){
    if(digitalRead(SIGNAL_B)){
        left_encoder_ticks++;
    }
    else{
        left_encoder_ticks--;
    }
}

void rightEncoderISR(){
    if(digitalRead(RIGHT_SIGNAL_B)){
        right_encoder_ticks++;
    }
    else{
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
double comput_vehicle_encoder_turn_rate(double translational_speed_L, double translational_speed_R){
    double omega;
    omega = 1.0 / ELL * (translational_speed_R - translational_speed_L);
    return omega;
}

double yawRate(const float alpha, const double translational_speed_L, const double translational_speed_R){
    // YawRate = a * Gyro + (1-a) * encoder
    float gyro = (omega_z - z_bias) * PI / 180.0; // Convert to rad/s
    float encoder = comput_vehicle_encoder_turn_rate(translational_speed_L, translational_speed_R);
    if (abs(encoder) < 0.06) {
        encoder = 0.0;
    }   
    return alpha * gyro + (1 - alpha) * encoder;
}

//Activity 1 Workshop 3: compute desired wheel parameters
void compute_desired_wheel_parameters(double v_desired, double omega_D, double& v_L_desired, double& v_R_desired){
    v_L_desired = v_desired - (0.5 * ELL * omega_D); //
    v_R_desired = v_desired + (0.5 * ELL * omega_D); //
}

//Activity 2 Workshop 3: compute desired wheel PWM commands
short controller_PI(double e_now, double K_P, double e_int, double K_I){
    short U = (short)(K_P * e_now + K_I * e_int);
    //saturate U to be between 0 and 255
    if (U > 255){
        U = 255;
    }
    else if (U < -255){
        U = -255;
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
    attachInterrupt(SIGNAL_A, leftEncoderISR, RISING);
    attachInterrupt(RIGHT_SIGNAL_A, rightEncoderISR, RISING);

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
        omega = comput_vehicle_encoder_turn_rate(translational_speed_L, translational_speed_R);
        
        //################ NEW PID STUFF #####################
        compute_desired_wheel_parameters(v_desired, omega_D, v_L_desired, v_R_desired);
        error_L = v_L_desired - translational_speed_L;
        error_R = v_R_desired - translational_speed_R;

        // Sample time in seconds
        double dt = (double)(t_now - t_last) / 1000.0;

        e_int_L += error_L * dt;
        e_int_R += error_R * dt;

        double e_int_max = 255.0 / max(K_I, 1); // crude bound
        if (e_int_L > e_int_max){
            e_int_L = e_int_max;
        }
        else if (e_int_L < -e_int_max){
            e_int_L = -e_int_max;
        }
        if (e_int_R > e_int_max){
            e_int_R = e_int_max;
        }
        else if (e_int_R < -e_int_max){
            e_int_R = -e_int_max;
        }

        //----------P Controller----------------
        L_output = controller_PI(error_L, K_P, e_int_L, K_I);
        R_output = controller_PI(error_R, K_P, e_int_R, K_I);

        // --- Simple anti‑windup: stop integrating if output saturates ---
        if (L_output == 255 || L_output == -255) {
            // roll back last integral step
            e_int_L -= error_L * dt;
        }
        if (R_output == 255 || R_output == -255) {
            e_int_R -= error_R * dt;
        }
        
        //Print Speed Control loop information
        Serial.print("t="); Serial.print(t_now);
        Serial.print(" ticksL="); Serial.print(left_encoder_ticks);
        Serial.print(" ticksR="); Serial.print(right_encoder_ticks);
        Serial.print(" vL="); Serial.print(translational_speed_L, 4);
        Serial.print(" uL="); Serial.println(L_output);

        

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    if(R_output >=0){
        // Select a direction, RIGHT wheels
        //Forward
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
    }
    else{
        // Select a direction, RIGHT wheels
        //Reverse
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    }
    analogWrite(EB, abs(R_output));

    if(L_output >=0){
        //LEFT wheels, Forward
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    }
    else{
        //LEFT wheels, Reverse
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
    }
    analogWrite(EA, abs(L_output));

}
