// #include <Arduino.h>
// #include <Arduino_LSM6DS3.h>

// // Variables to store angular rates from the gyro [degrees/s]
// float omega_x, omega_y, omega_z;

// float x_bias = -0.18;
// float y_bias = -0.31;
// float z_bias = -0.43;

// // Variables to store accelerations [g's]
// float a_x, a_y, a_z;

// // Variables to store sample rates from sensor [Hz]
// float a_f, g_f;

// void setup()
// {
//     // Open the serial port at 115200 bps
//     Serial.begin(115200);

//     // Wait for serial connection before starting
//     while (!Serial)
//     {
//         delay(10);
//     }

//     Serial.println();

//     // Check that the board is initialized
//     if (!IMU.begin())
//     {
//         // Print an error message if the IMU is not ready
//         Serial.print("Failed to initialize IMU :(");
//         Serial.print("\n");
//         while (1)
//         {
//             delay(10);
//         }
//     }

//     // Read the sample rate of the accelerometer and gyroscope
//     a_f = IMU.accelerationSampleRate();
//     g_f = IMU.gyroscopeSampleRate();

//     // Print these values to the serial window
//     Serial.print("Accelerometer sample rate: ");
//     Serial.println(a_f);
//     Serial.print("Gyroscope sample rate: ");
//     Serial.println(g_f);
// }

// // double yawRate(const float alpha){
// //     // YawRate = a * Gyro + (1-a) * encoder
// //     float gyro = omega_z - z_bias;
// //     float encoder = compute_vehicle_rate(translational_speed_L, translational_speed_R);
// //     return alpha * gyro + (1 - alpha) * encoder;
// // }

// void loop()
// {
//     // Timing in the loop is controlled by the IMU reporting when
//     // it is ready for another measurement.
//     // The accelerometer and gyroscope output at the same rate and
//     // will give us their measurements at a steady frequency.

//     // Read from the accelerometer
//     if (IMU.accelerationAvailable())
//     {
//         IMU.readAcceleration(a_x, a_y, a_z);

//         /// Print the accelerometer measurements to the Serial Monitor
//         Serial.print(a_x);
//         Serial.print("\t");
//         Serial.print(a_y);
//         Serial.print("\t");
//         Serial.print(a_z);
//         Serial.print(" g\t\t");
//     }

//     // Read from the gyroscope
//     if (IMU.gyroscopeAvailable())
//     {
//         IMU.readGyroscope(omega_x, omega_y, omega_z);

//         // Print the gyroscope measurements to the Serial Monitor
//         Serial.print(omega_x-x_bias);
//         Serial.print("\t");
//         Serial.print(omega_y-y_bias);
//         Serial.print("\t");
//         Serial.print(omega_z-z_bias);
//         Serial.print(" deg/s\n");
//     }
// }