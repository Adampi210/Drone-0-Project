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
 *  rotation direction.
 *  The angles: 
 *    Pitch: when front goes down pitch is positive, when front goes up its negative
 *    Roll: Looking from the back of the drone in the direction of the front turning clockwise is positive, counter-clockwise is negative
 *    Yaw rate: Looking from the top of the drone down on it, the clockwise turn is positive and the counterclockwise is negative
 */

 
#define I_ERROR_CALC_TH 3 // Threshold for calculating the I error
#define MAX_PID_VALUE 300 // Maximum value a PID error can have

#define MIN_PWM_VALUE 1000
#define MAX_PWM_VALUE 2000

// Variables
// Data
double desired_pitch = 0, desired_roll = 0, desired_yaw_rate = 0; // Desired values of pitch, roll, and yaw rate sent from the controller
double desired_esc_speed = 1000;
double angle_pitch, angle_roll, yaw_rate; // Actual values of pitch, roll and yaw read from the sensors
double angle_data_array[3];   // Array of angles sent from the control device
uint16_t radio_data_array[9]; // Radio data from the control device

// PID calculation
double error_pitch, error_roll, error_yaw_rate; // Error values (difference between desired and actual) for the pitch, roll, and yaw rate
double prev_error_pitch = 0, prev_error_roll = 0, prev_error_yaw_rate = 0; // Previous values of the errors used in PID calculation (for calculating the D factor)
// Pitch values
double pid_p_pitch, pid_i_pitch,  pid_d_pitch; // Calculated P, I, and D values for the pitch
double kp_pitch = 3.2; // kp, ki, kd values for pitch that specify how much each error is worth in the PID error calculation
double ki_pitch = 0;
double kd_pitch = 24;
// Roll values
double pid_p_roll, pid_i_roll,  pid_d_roll; // Calculated P, I, and D values for the roll
double kp_roll = 3.2; // kp, ki, kd values for roll that specify how much each error is worth in the PID error calculation
double ki_roll = 0;
double kd_roll = 24;
// Yaw values
double pid_p_yaw, pid_i_yaw,  pid_d_yaw; // Calculated P, I, and D values for the yaw rate
double kp_yaw = 0; // kp, ki, kd values for yaw rate that specify how much each error is worth in the PID error calculation
double ki_yaw = 0;
double kd_yaw = 0;

// Total PID values
double PID_pitch, PID_roll, PID_yaw_rate; // PID values calculated for pitch, roll, and the yaw rate
double esc1_pwm = 1000, esc2_pwm = 1000, esc3_pwm = 1000, esc4_pwm = 1000; // PWM values that set the speed of each motors

// Time variables
unsigned long current_time_PID = 0; // Current time in the loop in microseconds
unsigned long previous_time_PID; // Previous time variables used to run independent conditions
double time_difference_PID;      // Time difference between PID function runs used for PID calculation

// This function calculates pid values and in accordence to the calculated values sets given motor speeds to each drone motor
void calculate_pid_set_values(Servo esc_1_module, Servo esc_2_module, Servo esc_3_module, Servo esc_4_module, int* data_array) {
  previous_time_PID = current_time_PID; // First set the previous time to the time the previous function run (this will give me the difference between that time and the current time)
  current_time_PID = micros(); // Measure the current time, this will stay the same until next time this function is called, so it will be saved to previous_time_PID in the next iteration
  time_difference_PID = ((double)(current_time_PID - previous_time_PID)) / 1000000; // Calculate the time difference in s between previous and this function call

  // Get the current data
  convert_data(data_array, radio_data_array, angle_data_array);
  // Save the angle values
  angle_pitch = angle_data_array[0];
  angle_roll = angle_data_array[1];
  yaw_rate = angle_data_array[2];
  
  desired_pitch = map(radio_data_array[4], 5, 1014, -10, 10); // Map right joystick y to correspond to angles from -7 to 7 degrees
  desired_roll = map(radio_data_array[5], 5, 1014, -10, 10); // Map right joystick x to correspond to angles from -7 to 7 degrees
  desired_yaw_rate = map(radio_data_array[3], 0, 1020, -3, 3); // Map left joystick x to correspond to angle rates from -3 to 3 degrees/s
  desired_esc_speed = map(radio_data_array[2], 0, 1023, 1000, 2000); // Map left joystick y to correspond to ESC speed (PWM signal), goes from 1200 to 1800

  //kp_pitch = ((double)radio_data_array[1] / 100);
  //kp_roll = kp_pitch;
  
  // PID pitch calculation
  error_pitch = angle_pitch - desired_pitch; // Calculate error for pitch as difference between actual and desired values
  pid_p_pitch = kp_pitch * error_pitch;  // Calculate the proportional error (P value) for pitch as kp (some scaling factor) times the error
  // If the error is small enough calculate the integral error (I value)
  // Calculate the error by incrementing it by some factor times current error
  if(error_pitch > I_ERROR_CALC_TH * -1 && error_pitch < I_ERROR_CALC_TH) {
    pid_i_pitch += ki_pitch * error_pitch;
  }
  // Calculate the derivative error (D value) by taking the difference between current and previous errors (multiplied by factor kd)
  pid_d_pitch = kd_pitch * (error_pitch - prev_error_pitch);
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
  error_roll = angle_roll - desired_roll; // Calculate error for roll as difference between actual and desired values
  pid_p_roll = kp_roll * error_roll;  // Calculate the proportional error (P value) for roll as kp (some scaling factor) times the error
  // If the error is small enough calculate the integral error (I value)
  // Calculate the error by incrementing it by some factor times current error
  if(error_roll > I_ERROR_CALC_TH * -1 && error_roll < I_ERROR_CALC_TH) {
    pid_i_roll += ki_roll * error_roll;
  }
  // Calculate the derivative error (D value) by taking the difference between current and previous errors (multiplied by factor kd)
  pid_d_roll = kd_roll * (error_roll - prev_error_roll);
  prev_error_roll = error_roll; // Save the current error as the previous error for roll
  // Calculate the PID for roll as the sum of P I and D parts
  PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  // Also limit the PID values to a set min/max value so that it doesn't spiral out of control
  if(PID_roll < -1 * MAX_PID_VALUE) {
    PID_roll = -1 * MAX_PID_VALUE;
  }
  if(PID_roll > MAX_PID_VALUE) {
    PID_roll = MAX_PID_VALUE;
  }

  // PID yaw rate calculation
  error_yaw_rate = yaw_rate - desired_yaw_rate; // Calculate error for yaw rate as difference between actual and desired values
  pid_p_yaw = kp_yaw * error_yaw_rate;  // Calculate the proportional error (P value) for yaw rate as kp (some scaling factor) times the error
  // If the error is small enough calculate the integral error (I value)
  // Calculate the error by incrementing it by some factor times current error
  if(error_yaw_rate > I_ERROR_CALC_TH * -1 && error_yaw_rate < I_ERROR_CALC_TH) {
    pid_i_yaw += ki_yaw * error_yaw_rate;
  }
  // Calculate the derivative error (D value) by taking the difference between current and previous errors (multiplied by factor kd)
  pid_d_yaw = kd_yaw * (error_yaw_rate - prev_error_yaw_rate);
  prev_error_yaw_rate = error_yaw_rate; // Save the current error as the previous error for yaw rate
  // Calculate the PID for yaw rate as the sum of P I and D parts
  PID_yaw_rate = pid_p_yaw + pid_i_yaw + pid_d_yaw;
  // Also limit the PID values to a set min/max value so that it doesn't spiral out of control
  if(PID_yaw_rate < -1 * MAX_PID_VALUE) {
    PID_yaw_rate = -1 * MAX_PID_VALUE;
  }
  if(PID_yaw_rate > MAX_PID_VALUE) {
    PID_yaw_rate = MAX_PID_VALUE;
  }
  
  // Motor speed calculation TODO: change set offset
  esc1_pwm = 1650 + PID_pitch - PID_roll + PID_yaw_rate;
  esc2_pwm = 1650 - PID_pitch + PID_roll + PID_yaw_rate;
  esc3_pwm = 1650 - PID_pitch - PID_roll - PID_yaw_rate;
  esc4_pwm = 1650 + PID_pitch + PID_roll - PID_yaw_rate;

  // Limit max and min PWM values
  if(esc1_pwm < MIN_PWM_VALUE){esc1_pwm = MIN_PWM_VALUE;}
  if(esc2_pwm < MIN_PWM_VALUE){esc2_pwm = MIN_PWM_VALUE;}
  if(esc3_pwm < MIN_PWM_VALUE){esc3_pwm = MIN_PWM_VALUE;}
  if(esc4_pwm < MIN_PWM_VALUE){esc4_pwm = MIN_PWM_VALUE;}

  if(esc1_pwm > MAX_PWM_VALUE){esc1_pwm = MAX_PWM_VALUE;}
  if(esc2_pwm > MAX_PWM_VALUE){esc2_pwm = MAX_PWM_VALUE;}
  if(esc3_pwm > MAX_PWM_VALUE){esc3_pwm = MAX_PWM_VALUE;}
  if(esc4_pwm > MAX_PWM_VALUE){esc4_pwm = MAX_PWM_VALUE;}
  
  // TODO find all values and finish the project
  if(radio_data_array[6]) {
    /*esc_1_module.writeMicroseconds(esc1_pwm);
    esc_2_module.writeMicroseconds(esc2_pwm);
    esc_3_module.writeMicroseconds(esc3_pwm);
    esc_4_module.writeMicroseconds(esc4_pwm);*/
    Serial.print(angle_pitch);
    Serial.print('\t');
    Serial.println(angle_roll);
    /*Serial.print(esc1_pwm);
    Serial.print(" ");
    Serial.print(esc2_pwm);
    Serial.print(" ");
    Serial.print(esc3_pwm);
    Serial.print(" ");
    Serial.print(esc4_pwm);
    Serial.println();
    */
    // TODO UPLOAD THE LATEST receiver_and_sensors code version
    esc_1_module.writeMicroseconds(MIN_PWM_VALUE);
    esc_2_module.writeMicroseconds(MIN_PWM_VALUE);
    esc_3_module.writeMicroseconds(MIN_PWM_VALUE);
    esc_4_module.writeMicroseconds(MIN_PWM_VALUE);
  }
  else {
    esc_1_module.writeMicroseconds(MIN_PWM_VALUE);
    esc_2_module.writeMicroseconds(MIN_PWM_VALUE);
    esc_3_module.writeMicroseconds(MIN_PWM_VALUE);
    esc_4_module.writeMicroseconds(MIN_PWM_VALUE);
  }


}
