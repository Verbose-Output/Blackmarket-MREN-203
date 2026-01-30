#include <Arduino.h>


// Compute vehicle speed [m/s]
 double compute_vehicle_speed(double v_L, double v_R)
 {
   return 0.5 * (v_L + v_R);
 }



 // Compute vehicle turning rate [rad/s]
 double compute_vehicle_rate(double v_L, double v_R)
 {
   return 1.0 / 0.2775 * (v_R - v_L);
 }



 //Compute Wheel Speed Left with desired VELOCITY and desired TURNING RATE
 double compute_wheel_speed_Left(double v_D, double omega_D)
 {
   return 1.0 / 0.0625 * (v_D - 0.2775*omega_D/2);
 }



 //Compute Wheel Speed Right with desired VELOCITY and desired TURNING RATE
 double compute_wheel_speed_Right(double v_D, double omega_D)
 {
   return 1.0 / 0.0625 * (v_D + 0.2775*omega_D/2);
 }

 //the PID controller function
 //v_D: Desired Velocity [m/s]  omega_D: Desired Turning Rate [rad/s]
 //v_L: Current Left Wheel Speed [m/s]  v_R: Current Right Wheel Speed [m/s]
 //kp_v, ki_v, kd_v: PID gains for Velocity control
 //kp_omega, ki_omega, kd_omega: PID gains for Turning Rate control
 //dt: time step [s]
 //v_L_cmd: OUTPUT Left Wheel Speed Command [m/s]  v_R_cmd: OUTPUT Right Wheel Speed Command [m/s]
 //NEED TO PUT OUTPUT VARIABLES IN THESE PARAMETERS ^^^^^^
 double PID_Controller(double v_D, double omega_D, double v_L, double v_R, double kp_v, double ki_v, double kd_v, double kp_omega, double ki_omega, double kd_omega, double dt, double &v_L_cmd, double &v_R_cmd) 
 {
   //Current vehicle States
   double v_current = compute_vehicle_speed(v_L, v_R);
   double omega_current = compute_vehicle_rate(v_L, v_R);

   //Errors
   double v_error = v_D - v_current;
   double omega_error = omega_D - omega_current;

   //Integral computed here
   static double integral_v_error = 0.0;
   static double integral_omega_error = 0.0;
   integral_v_error += v_error * dt;
   integral_omega_error += omega_error * dt;


   //Derivative computed here
   static double last_v_error = 0.0;
   static double last_omega_error = 0.0;
   double derivative_v_error = (v_error - last_v_error) / dt;
   double derivative_omega_error = (omega_error - last_omega_error) / dt;
   last_v_error = v_error;  //save v error for next time
   last_omega_error = omega_error; //save omega error for next time

   //PID calculations
   double v_cmd = kp_v * v_error + ki_v * integral_v_error + kd_v * derivative_v_error; // + ki_v * integral_v_error + kd_v * derivative_v_error;
   double omega_cmd = kp_omega * omega_error + ki_omega * integral_omega_error + kd_omega * derivative_omega_error; // + ki_omega * integral_omega_error + kd_omega * derivative_omega_error;

   //Compute Wheel Speed
   v_L_cmd = compute_wheel_speed_Left(v_cmd, omega_cmd);
   v_R_cmd = compute_wheel_speed_Right(v_cmd, omega_cmd);
 }

