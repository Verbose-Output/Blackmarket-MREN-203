#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

int EA = 6;
int EB = 5;

int I1 = 7;
int I2 = 4;
int I3 = 8;
int I4 = 2;

const byte SIGNAL_A       = 10;
const byte SIGNAL_B       = 11;
const byte RIGHT_SIGNAL_A = 12;
const byte RIGHT_SIGNAL_B = 13;

// ============================================================
// ROBOT PARAMETERS
// ============================================================

const int    TPR = 3150;
const double RHO = 0.0625;
const double ELL = 0.2775;
const int    T   = 100;

// Sensor fusion weight
const double ALPHA = 0.7; // encoder weight

// ============================================================
// ENCODER STATE
// ============================================================

volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;

// ============================================================
// SPEED ESTIMATES
// ============================================================

double omega_L = 0.0;
double omega_R = 0.0;
double translational_speed_L = 0.0;
double translational_speed_R = 0.0;

double v     = 0.0;
double omega = 0.0;

// ============================================================
// IMU VARIABLES
// ============================================================

float omega_x, omega_y, omega_z;
float a_x, a_y, a_z;

float z_bias = -0.43;

// ============================================================
// CONTROL PARAMETERS
// ============================================================

double v_desired   = 0.0;
double omega_D     = 0.0;

double v_L_desired = 0.0;
double v_R_desired = 0.0;

const double K_P = 80.0;
double       K_I = 20.0;

double error_L = 0.0;
double error_R = 0.0;

double e_int_L = 0.0;
double e_int_R = 0.0;

int L_output = 0;
int R_output = 0;

const int PWM_MIN = 150;

// ============================================================
// TIMING
// ============================================================

long t_now      = 0;
long t_last     = 0;
long t_last_cmd = 0;
const long CMD_TIMEOUT_MS = 500;

// ============================================================
// INTERRUPTS
// ============================================================

void leftEncoderISR() {
    if (digitalRead(SIGNAL_B)) left_encoder_ticks++;
    else                       left_encoder_ticks--;
}

void rightEncoderISR() {
    if (digitalRead(RIGHT_SIGNAL_B)) right_encoder_ticks++;
    else                             right_encoder_ticks--;
}

// ============================================================
// HELPERS
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
// PI CONTROLLER (WITH DEADZONE COMPENSATION)
// ============================================================

short controller_PI(double e_now, double K_P, double e_int, double K_I) {
    double u = K_P * e_now + K_I * e_int;

    if (u >  255.0) u =  255.0;
    if (u < -255.0) u = -255.0;

    if (u > 0.0) {
        u = PWM_MIN + (255.0 - PWM_MIN) * (u / 255.0);
    } 
    else if (u < 0.0) {
        u = -PWM_MIN + (255.0 - PWM_MIN) * (u / 255.0);
    }

    return (short)u;
}

// ============================================================
// SERIAL PARSER
// ============================================================

void parseSerialCommand(const String &cmd) {
    if (!cmd.startsWith("v:")) return;

    int comma = cmd.indexOf(',');
    if (comma < 0) return;

    double new_v = cmd.substring(2, comma).toFloat();
    double new_w = cmd.substring(comma + 3).toFloat();

    if (abs(new_v - v_desired) > 0.01 || abs(new_w - omega_D) > 0.01) {
        e_int_L = 0.0;
        e_int_R = 0.0;
    }

    v_desired  = new_v;
    omega_D    = new_w;
    t_last_cmd = millis();
}

// ============================================================
// MOTOR DRIVER
// ============================================================

void setMotors(int l_cmd, int r_cmd) {

    if (r_cmd >= 0) {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
    } else {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    }
    analogWrite(EB, abs(r_cmd));

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
    Serial.begin(115200);

    pinMode(EA, OUTPUT); pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT); pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT); pinMode(I4, OUTPUT);

    pinMode(SIGNAL_A,       INPUT);
    pinMode(SIGNAL_B,       INPUT);
    pinMode(RIGHT_SIGNAL_A, INPUT);
    pinMode(RIGHT_SIGNAL_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(SIGNAL_A),       leftEncoderISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_SIGNAL_A), rightEncoderISR, RISING);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU");
        while (1) delay(10);
    }

    Serial.println("Arduino ready. Waiting for v command to spin...");

    // ---------------------------
    // Startup spin loop: parse serial commands
    // ---------------------------
    v = 0.0;
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    while (v_desired < 0.5) {
        // Read Serial input
        if (Serial.available() > 0) {
            String incoming = Serial.readStringUntil('\n');
            incoming.trim();
            parseSerialCommand(incoming);  // updates v_desired
        }

        // Compute wheel speeds from encoders
        omega_L = 2.0 * PI * ((double)left_encoder_ticks  / TPR) * 1000.0 / T;
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / TPR) * 1000.0 / T;

        translational_speed_L = omega_L * RHO;
        translational_speed_R = omega_R * RHO;

        v = compute_vehicle_speed(translational_speed_L, translational_speed_R);

        // Spin motors gently in opposite directions
        setMotors(200, -200);

        delay(T);  // wait for next measurement
        left_encoder_ticks = 0;
        right_encoder_ticks = 0;
    }

    // Stop motors once v_desired >= 0.5
    setMotors(0, 0);

    Serial.println("Startup spin complete. Entering normal loop.");
    t_last_cmd = millis();
    t_last = millis();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    t_now = millis();

    // Serial input
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();
        parseSerialCommand(incoming);
    }

    // Watchdog
    if (t_now - t_last_cmd > CMD_TIMEOUT_MS) {
        v_desired = 0.0;
        omega_D   = 0.0;
        e_int_L   = 0.0;
        e_int_R   = 0.0;
    }


    // Read IMU
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(omega_x, omega_y, omega_z);
    }

    // Control loop
    if (t_now - t_last >= T) {
        double dt = (double)(t_now - t_last) / 1000.0;

        // Encoder speeds
        omega_L = 2.0 * PI * ((double)left_encoder_ticks  / TPR) * 1000.0 / (t_now - t_last);
        omega_R = 2.0 * PI * ((double)right_encoder_ticks / TPR) * 1000.0 / (t_now - t_last);

        translational_speed_L = omega_L * RHO;
        translational_speed_R = omega_R * RHO;

        // Encoder-based omega
        double omega_enc = compute_vehicle_turn_rate(translational_speed_L, translational_speed_R);

        // IMU omega (convert deg/s → rad/s)
        double omega_imu = (omega_z - z_bias) * DEG_TO_RAD;

        // Sensor fusion
        omega = ALPHA * omega_enc + (1.0 - ALPHA) * omega_imu;

        v = compute_vehicle_speed(translational_speed_L, translational_speed_R);

        // Desired wheel speeds
        compute_desired_wheel_parameters(v_desired, omega_D, v_L_desired, v_R_desired);

        // Turning correction using fused omega
        double omega_error = omega_D - omega;

        v_L_desired -= 0.5 * ELL * omega_error;
        v_R_desired += 0.5 * ELL * omega_error;

        // Errors
        error_L = v_L_desired - translational_speed_L;
        error_R = v_R_desired - translational_speed_R;

        e_int_L += error_L * dt;
        e_int_R += error_R * dt;

        double e_int_max = 255.0 / max(K_I, 1.0);
        e_int_L = constrain(e_int_L, -e_int_max, e_int_max);
        e_int_R = constrain(e_int_R, -e_int_max, e_int_max);

        L_output = controller_PI(error_L, K_P, e_int_L, K_I);
        R_output = controller_PI(error_R, K_P, e_int_R, K_I);

        if (L_output == 255 || L_output == -255) e_int_L -= error_L * dt;
        if (R_output == 255 || R_output == -255) e_int_R -= error_R * dt;

        if (v_desired == 0.0 && omega_D == 0.0) {
            L_output = 0;
            R_output = 0;
        }

        t_last = t_now;
        left_encoder_ticks  = 0;
        right_encoder_ticks = 0;
    }

    setMotors(L_output, R_output);
}
