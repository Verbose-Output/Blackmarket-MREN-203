#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// Wheel PWM pins (must be PWM-capable)
int EA = 6;
int EB = 5;

// Wheel direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 8;
int I4 = 2;

// Left wheel encoder pins
const byte SIGNAL_A       = 10;
const byte SIGNAL_B       = 11;

// Right wheel encoder pins
const byte RIGHT_SIGNAL_A = 12;
const byte RIGHT_SIGNAL_B = 13;

// ============================================================
// ROBOT PARAMETERS
// ============================================================

// Encoder ticks per full motor revolution
const int TPR = 3150;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle track width (distance between wheels) [m]
const double ELL = 0.2775;

// Sampling interval [ms]
const int T = 100;

// ============================================================
// ENCODER STATE  (volatile — modified inside ISRs)
// ============================================================

volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;

// ============================================================
// SPEED ESTIMATES
// ============================================================

double omega_L = 0.0;              // Left  wheel angular speed  [rad/s]
double omega_R = 0.0;              // Right wheel angular speed  [rad/s]
double translational_speed_L = 0.0; // Left  wheel linear speed   [m/s]
double translational_speed_R = 0.0; // Right wheel linear speed   [m/s]
double v     = 0.0;                // Vehicle forward speed       [m/s]
double omega = 0.0;                // Vehicle yaw rate            [rad/s]

// ============================================================
// IMU VARIABLES
// ============================================================

float omega_x, omega_y, omega_z;
float a_x, a_y, a_z;
float a_f, g_f;

// Gyro biases (measured at rest)
float x_bias = -0.18;
float y_bias = -0.31;
float z_bias = -0.43;

// ============================================================
// PI CONTROLLER PARAMETERS
// ============================================================

// Setpoints — updated via serial commands from the Raspberry Pi
double v_desired   = 0.0;   // desired vehicle speed    [m/s]
double omega_D     = 0.0;   // desired vehicle yaw rate [rad/s]

// Derived wheel setpoints
double v_L_desired = 0.0;
double v_R_desired = 0.0;

// Gains
const double K_P = 80.0;    // Proportional gain
double       K_I = 20.0;    // Integral gain

// Errors
double error_L = 0.0;
double error_R = 0.0;

// Integral accumulators
double e_int_L = 0.0;
double e_int_R = 0.0;

// Controller outputs [0-255]
int L_output = 0;
int R_output = 0;

// Minimum PWM to overcome motor deadzone
const int PWM_MIN = 100;

// Timing
long t_now  = 0;
long t_last = 0;

// ============================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================

void leftEncoderISR() {
    if (digitalRead(SIGNAL_B)) {
        left_encoder_ticks++;
    } else {
        left_encoder_ticks--;
    }
}

void rightEncoderISR() {
    if (digitalRead(RIGHT_SIGNAL_B)) {
        right_encoder_ticks++;
    } else {
        right_encoder_ticks--;
    }
}

// ============================================================
// KINEMATICS HELPERS
// ============================================================

double compute_vehicle_speed(double v_L, double v_R) {
    return 0.5 * (v_L + v_R);
}

double compute_vehicle_turn_rate(double v_L, double v_R) {
    return (1.0 / ELL) * (v_R - v_L);
}

void compute_desired_wheel_parameters(double v_d, double omega_d,
                                      double &v_L, double &v_R) {
    v_L = v_d - 0.5 * ELL * omega_d;
    v_R = v_d + 0.5 * ELL * omega_d;
}

// ============================================================
// PI CONTROLLER
// ============================================================

short controller_PI(double e_now, double K_P, double e_int, double K_I) {
    short U = (short)(K_P * e_now + K_I * e_int);
    if (U >  255) U =  255;
    if (U < -255) U = -255;
    return U;
}

// ============================================================
// SERIAL COMMAND PARSER
// Parse format: "v:{float},w:{float}\n"  sent by arduino_bridge_node.py
// ============================================================

void parseSerialCommand(const String &cmd) {
    // Expect "v:X.XXXX,w:Y.YYYY"
    if (!cmd.startsWith("v:")) return;

    int comma = cmd.indexOf(',');
    if (comma < 0) return;

    v_desired = cmd.substring(2, comma).toFloat();
    omega_D   = cmd.substring(comma + 3).toFloat();  // skip "w:"

    // Reset integrals when a new setpoint arrives to avoid windup carry-over
    e_int_L = 0.0;
    e_int_R = 0.0;
}

// ============================================================
// MOTOR DRIVER HELPERS
// ============================================================

void setMotors(int l_cmd, int r_cmd) {
    // Right wheels
    if (r_cmd >= 0) {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
    } else {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    }
    analogWrite(EB, abs(r_cmd));

    // Left wheels
    if (l_cmd >= 0) {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    } else {
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
    }
    analogWrite(EA, abs(l_cmd));
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(9600);

    // Motor driver pins
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Encoder pins
    pinMode(SIGNAL_A,       INPUT);
    pinMode(SIGNAL_B,       INPUT);
    pinMode(RIGHT_SIGNAL_A, INPUT);
    pinMode(RIGHT_SIGNAL_B, INPUT);

    // Attach interrupts — separate ISRs for each wheel
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A),       leftEncoderISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_SIGNAL_A), rightEncoderISR, RISING);

    // Initialise IMU
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU");
        while (1) delay(10);
    }

    a_f = IMU.accelerationSampleRate();
    g_f = IMU.gyroscopeSampleRate();

    Serial.println("Arduino ready. Awaiting commands from Pi.");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
    t_now = millis();

    // ----------------------------------------------------------
    // 1. Read incoming serial command from Raspberry Pi
    // ----------------------------------------------------------
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();
        parseSerialCommand(incoming);
    }

    // ----------------------------------------------------------
    // 2. Read IMU (as fast as available — used for future fusion)
    // ----------------------------------------------------------
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(a_x, a_y, a_z);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(omega_x, omega_y, omega_z);
    }

    // ----------------------------------------------------------
    // 3. Control loop — runs every T ms
    // ----------------------------------------------------------
    if (t_now - t_last >= T) {
        double dt = (double)(t_now - t_last) / 1000.0;

        // --- Speed estimation ---
        omega_L = 2.0 * PI * ((double)left_encoder_ticks  / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        translational_speed_L = omega_L * RHO;
        translational_speed_R = omega_R * RHO;

        v     = compute_vehicle_speed(translational_speed_L, translational_speed_R);
        omega = compute_vehicle_turn_rate(translational_speed_L, translational_speed_R);

        // --- Desired wheel speeds ---
        compute_desired_wheel_parameters(v_desired, omega_D, v_L_desired, v_R_desired);

        // --- Errors ---
        error_L = v_L_desired - translational_speed_L;
        error_R = v_R_desired - translational_speed_R;

        // --- Integrate errors ---
        e_int_L += error_L * dt;
        e_int_R += error_R * dt;

        // --- Integral clamping (anti-windup) ---
        double e_int_max = 255.0 / max(K_I, 1.0);
        e_int_L = constrain(e_int_L, -e_int_max, e_int_max);
        e_int_R = constrain(e_int_R, -e_int_max, e_int_max);

        // --- PI controller ---
        L_output = controller_PI(error_L, K_P, e_int_L, K_I);
        R_output = controller_PI(error_R, K_P, e_int_R, K_I);

        // --- Back-calculation anti-windup: undo last integral step if saturated ---
        if (L_output ==  255 || L_output == -255) e_int_L -= error_L * dt;
        if (R_output ==  255 || R_output == -255) e_int_R -= error_R * dt;

        // --- Update timing and reset encoder counts ---
        t_last = t_now;
        left_encoder_ticks  = 0;
        right_encoder_ticks = 0;
    }

    // ----------------------------------------------------------
    // 4. Apply motor commands (runs every loop iteration for
    //    smooth PWM output, not just every T ms)
    // ----------------------------------------------------------
    setMotors(L_output, R_output);
}
