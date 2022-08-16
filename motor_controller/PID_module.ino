/*
 * ESC setup:
 *    FRONT
 *  1  /\  4
 *   \ || /
 *    \  /
 *     \/
 *     /\
 *    /  \
 *   /    \
 *  3      2
 *  
 *  ESC_1, ESC_2 rotate clockwise
 *  ESC_3, ESC_4 rotate counter-clockwise
 *  These are named this way because the first two and last two have the same
 *  rotation direction
 */

// TODO: check angles and errors (how they are calculated)
// 
#define I_ERROR_CALC_TH 3 // Threshold for calculating the I error
#define MAX_PID_VALUE 300 // Maximum value a PID error can have

// Variables
// Data
double desired_pitch = 0, desired_roll = 0, desired_yaw_rate = 0; // Desired values of pitch, roll, and yaw rate sent from the controller
double angle_pitch, angle_roll, yaw_rate; // Actual values of pitch, roll and yaw read from the sensors
double desired_alt; // Desired altitude sent from the controller that the drone should maintain
double actual_alt;  // Actual altitude read from the barometer

// PID calculation
double error_pitch, error_roll, error_yaw_rate; // Error values (difference between desired and actual) for the pitch, roll, and yaw rate
double prev_error_pitch, prev_error_roll, prev_error_yaw_rate; // Previous values of the errors used in PID calculation (for calculating the D factor)
double error_alt; // Error value of the altitude (difference between current and desired altitude)
double prev_error_alt; // Previous error value of the altitude used for kd calculation
// Pitch values
double pid_p_pitch, pid_i_pitch,  pid_d_pitch; // Calculated P, I, and D values for the pitch
double kp_pitch = 0; // kp, ki, kd values for pitch that specify how much each error is worth in the PID error calculation
double ki_pitch = 0;
double kd_pitch = 0;
// Roll values
double pid_p_roll, pid_i_roll,  pid_d_roll; // Calculated P, I, and D values for the roll
double kp_roll = 0; // kp, ki, kd values for roll that specify how much each error is worth in the PID error calculation
double ki_roll = 0;
double kd_roll = 0;
// Yaw values
double pid_p_yaw, pid_i_yaw,  pid_d_yaw; // Calculated P, I, and D values for the yaw rate
double kp_yaw = 0; // kp, ki, kd values for yaw rate that specify how much each error is worth in the PID error calculation
double ki_yaw = 0;
double kd_yaw = 0;
// Altitude values
double pid_p_alt, pid_i_alt,  pid_d_alt; // Calculated P, I, and D values for the altitude
double kp_alt = 0; // kp, ki, kd values for altitude that specify how much each error is worth in the PID error calculation
double ki_alt = 0;
double kd_alt = 0;
// Total PID values
double PID_pitch, PID_roll, PID_yaw_rate, PID_alt; // PID values calculated for pitch, roll, yaw rate, and the altitude

// Time variables
unsigned long current_time_PID = 0; // Current time in the loop in microseconds
unsigned long previous_time_PID; // Previous time variables used to run independent conditions
double time_difference_PID;      // Time difference between PID function runs used for PID calculation

// This function calculates pid values and in accordence to the calculated values sets given motor speeds to each drone motor
void calculate_pid_set_values(Servo esc_1_module, Servo esc_2_module, Servo esc_3_module, Servo esc_4_module, int* data_array) {
  previous_time_PID = current_time_PID; // First set the previous time to the time the previous function run (this will give me the difference between that time and the current time)
  current_time_PID = micros(); // Measure the current time, this will stay the same until next time this function is called, so it will be saved to previous_time_PID in the next iteration
  time_difference_PID = (current_time_PID - previous_time_PID) / 1000000; // Calculate the time difference in s between previous and this function call

  // Get the current data TODO

  // PID pitch calculation
  error_pitch = angle_pitch - desired_pitch; // Calculate error for pitch as difference between actual and desired values
  pid_p_pitch = kp_pitch * error_pitch;  // Calculate the proportional error (P value) for pitch as kp (some scaling factor) times the error
  // If the error is small enough calculate the integral error (I value)
  // Calculate the error by incrementing it by some factor times current error
  if(error_pitch > I_ERROR_CALC_TH * -1 && error_pitch < I_ERROR_CALC_TH) {
    pid_i_pitch += ki_pitch * error_pitch;
  }
  // Calculate the derivative error (D value) by taking the difference between current and previous errors and dividing it by the time (first time derivative) (multiplied by factor kd)
  pid_d_pitch = kd_pitch * (error_pitch - prev_error_pitch) / time_difference_PID;
  prev_error_pitch = error_pitch; // Save the current error as the previous error for pitch
  // Calculate the PID for pitch as the sum of P I and D parts
  PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  // Also limit the PID values to a set min/max value so that it doesn't spiral out of control
  if(PID_pitch < -1 * MAX_PID_VALUE) {
    PID_pitch = -1 * MAX_PID_VALUE;
  }
  if(PID_pitch > MAX_PID_VALUE) {
    PID_pitch = MAX_PID_VALUE;
  }

  // PID roll calculation
  
  
}
