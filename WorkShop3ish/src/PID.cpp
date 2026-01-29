#include <Arduino.h>


// Compute vehicle speed [m/s]
 double compute_vehicle_speed(double v_L, double v_R)
 {
    double v;
    v = 0.5 * (translational_speed_L + translational_speed_R);
    return v;
 }



 // Compute vehicle turning rate [rad/s]
 double compute_vehicle_rate(double v_L, double v_R)
 {
    double omega;
    omega = 1.0 / 0.2775 * (translational_speed_R - translational_speed_L);
    return omega;
 }



 //Compute Wheel Speed Left with desired VELOCITY and desired TURNING RATE
 double compute_wheel_speed_Left(double v_D, double omega_D)
 {
    double v;
    v = 1.0 / 0.0625(v_D - 0.2775*omega_D/2);
    return v;
 }



 //Compute Wheel Speed Right with desired VELOCITY and desired TURNING RATE
 double compute_wheel_speed_Left(double v_D, double omega_D)
 {
    double v;
    v = 1.0 / 0.0625(v_D + 0.2775*omega_D/2);
    return v;
 }


 //th

