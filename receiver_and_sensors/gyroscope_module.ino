/*
 * This tab contains all the functions and variables used for calculation of the gyroscope values 
 * that result in returning the angle of pitch, roll, and the angular velocity change in yaw
 * Gyro used: MPU6050 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */
#define LOOP_FREQ 250 // Frequency at which the angles will be calculated in the main loop
#define LSB_SENSITIVITY 65.5 // Value used for scaling when calculating angle values

// This macro writes a specified message msg to a reg register of the MPU6050
// It first begins transmission with the IMU and then it writes a register
// Then it writes a message to that register and ends transmission
#define write_msg_register(reg, msg) do { \
  Wire.beginTransmission(GYRO_ADDR);      \
  Wire.write(reg);                        \
  Wire.write(msg);                        \
  Wire.endTransmission();                 \
} while(false)

// This macro requests a specified number of bytes from a register
// It begins the transmission, writes to the register and ends transmission
// and then requests no_bytes from that register
#define request_bytes_register(reg, no_bytes) do {  \
  Wire.beginTransmission(GYRO_ADDR);                \
  Wire.write(reg);                                  \
  Wire.endTransmission();                           \
  Wire.requestFrom(GYRO_ADDR, no_bytes);            \
} while(false) 

// Variables
long gyro_calibration_error[3] = {};  // Array of calibration errors for the gyroscope
long accel_calibration_error[3] = {}; // Array of calibration errors for the accelerometer
long gyro_values_arr[3] = {};         // Array of the measurements from the gyro (x, y, z axes respectively)
long accel_values_arr[3] = {};        // Array of acceleration values (x, y, z axes respectively)
long accel_mag;                       // Magnitude of the total acceleration
double yaw_rate_callibration_error = 0;   // calibration error for the calculated yaw rate value
double yaw_rate_gyro = 0;                 // calculated value of the yaw rate
double pitch_accel, roll_accel;           // Acceleration value for pitch and roll angles
double real_pitch = 0;                    // calculated pitch angle
double real_roll = 0;                     // calculated roll angle
double angle_pitch = 0;                   // real pitch angle output
double angle_roll = 0;                    // real roll angle output
bool first_measurement = true;            // Stores if a measurement is a first measurement in the program
// This function initializes the gyroscpe and the accelerometer and sets 
// the desired configuration parameters
void gyro_init_config() {
  // 0x6B is power management register
  // Sending 0 to it will reset the device
  write_msg_register(0x6B, 0x00);
    
  // Configure gyroscope sensitivity - send msg to register 0x1B
  // Write message 0x08 = b00001000, which will set the gyro range to +-500 deg/s
  write_msg_register(0x1B, 0x08);  
   
  // Configure accelerometer sensitivity - send msg to register 0x1C
  // Write message 0x10 = b00010000, which will set the accelerometer range to +-8g
  write_msg_register(0x1C, 0x10);  
}

// This function calibrates the gyro by making a number of measurements and taking their average
void calibrate_gyro() {
  // Take 2000 measurements and add them
  for(int i = 0; i < 2000; ++i) {
    // Request 6 bytes from register 0x43 which will be used to calculate the calibration values
    request_bytes_register(0x43, 6);
    // Calculate the values and add them to the total sum
    // To calculate each axis gyro measurement I read the first measurement (ex here GYRO_XOUT[15:8]), shift it 8 bits left, and add second measurement (ex here GYRO_XOUT[7:0])
    // This way I get the whole measurement for each axis (ex GYRO_XOUT[15:0])
    // All measurements give me rates, I just need to multiply them later by time
    gyro_calibration_error[0] += Wire.read() << 8 | Wire.read(); // Calculate x value
    gyro_calibration_error[1] += Wire.read() << 8 | Wire.read(); // Calculate y value
    gyro_calibration_error[2] += Wire.read() << 8 | Wire.read(); // Calculate z value
  }
  // Divide by 2000 to get the average
  gyro_calibration_error[0] /= 2000;
  gyro_calibration_error[1] /= 2000;
  gyro_calibration_error[2] /= 2000;
  

}
/* // NOT USED
// This function calibrates the accelerometer by making a number of measurements and taking their average
void calibrate_accel() {
  // Take 2000 measurements and add them
  for(int i = 0; i < 2000; ++i) {
    // Request 4 bytes from register 0x3B which will be used to calculate the calibration values
    request_bytes_register(0x3B, 4);
    // Calculate the values and add them to the total sum
    accel_calibration_error[0] += Wire.read() << 8 | Wire.read(); // Calculate x acceleration
    accel_calibration_error[1] += Wire.read() << 8 | Wire.read(); // Calculate y acceleration
  }
  // Divide by 2000 to get the average
  accel_calibration_error[0] /= 2000;
  accel_calibration_error[1] /= 2000;
}
*/
// This function calculates the calibration error for the yaw rate
void calibrate_gyro_yaw() {
  // Get 100 values
  for(int i = 0; i < 100; ++i) {
    // request 2 bytes from 0x47
    request_bytes_register(0x47, 2);
    // calculate the z value of the gyro used for yaw calculation
    gyro_values_arr[2] = Wire.read() << 8 | Wire.read() - gyro_calibration_error[2]; // yaw
    // calculate yaw rate with smoothing - .7 of the value is the last yaw rate value and 0.3 is based on the measurement
    // I set the range to +-500deg/s, so the calculated measurement needs to be scaled by 65.5 (as in datasheet LSB sensitivity for 500dps is 65.5)
    yaw_rate_gyro = 0.7 * yaw_rate_gyro + ((double) gyro_values_arr[2] / LSB_SENSITIVITY) * 0.3;
    // Add the calculated value to the calibration error
    yaw_rate_callibration_error += yaw_rate_gyro;
  }
  // Divide the calibration error by the number of measurements to get the average
  yaw_rate_callibration_error /= 100;
}

// This function calculates pitch and roll angles, as well as yaw change rate
void calculate_angles(double* angle_array) {
  // Request 6 bytes from register 0x43 used to calculate the angle
  request_bytes_register(0x43, 6);
  // calculate gyro values for 3 axes
  gyro_values_arr[0] = Wire.read() << 8 | Wire.read() - gyro_calibration_error[0]; // x
  gyro_values_arr[1] = Wire.read() << 8 | Wire.read() - gyro_calibration_error[1]; // y
  gyro_values_arr[2] = Wire.read() << 8 | Wire.read() - gyro_calibration_error[2]; // z

  // get and calculate acceleration values
  request_bytes_register(0x3B, 6);
  accel_values_arr[0] = Wire.read() << 8 | Wire.read(); // x
  accel_values_arr[1] = Wire.read() << 8 | Wire.read(); // y
  accel_values_arr[2] = Wire.read() << 8 | Wire.read(); // z

  // Calculate the yaw rate with smoothing and reduced by calibration
  yaw_rate_gyro = 0.8 * yaw_rate_callibration_error + ((double) gyro_values_arr[2] / LSB_SENSITIVITY) * 0.2 - yaw_rate_callibration_error;
  
  // Gyro angle calculations
  // First add how much the angle changed over time
  // Also, I put the IMU sideways so the x and y axes are switched when calculating the angles
  real_pitch += gyro_values_arr[1] / (LOOP_FREQ * LSB_SENSITIVITY);
  real_roll += gyro_values_arr[0] / (LOOP_FREQ * LSB_SENSITIVITY);

  // If the IMU has rotated over time, adjust for that, also convert the inner angle to radians
  real_pitch += real_roll * sin(gyro_values_arr[2] / (LOOP_FREQ * LSB_SENSITIVITY) * PI / 180);
  real_roll -= real_pitch * sin(gyro_values_arr[2] / (LOOP_FREQ * LSB_SENSITIVITY) * PI / 180);

  // Accelerometer angle calculations
  // First calculate total acceleration magnitude = root of all the axes values squared
  accel_mag = sqrt(accel_values_arr[0] * accel_values_arr[0] + accel_values_arr[1] * accel_values_arr[1] + accel_values_arr[2] * accel_values_arr[2]);
  // Calculate pitch and roll acceleration in degrees
  pitch_accel = asin((double)accel_values_arr[0] / accel_mag) * 180 / PI;
  roll_accel = asin((double)accel_values_arr[1] / accel_mag) * -180 / PI;
  
  if(first_measurement) {
    // If it's a first measurement, set pitch and roll angles to be equal to acceleration
    real_pitch = pitch_accel;
    real_roll = roll_accel;
    first_measurement = false;
  }
  else {
    // Otherwise just correct the drift with the acceleration
    real_pitch = real_pitch * 0.94 + pitch_accel * 0.06;
    real_roll = real_roll * 0.94 + roll_accel * 0.06;
  }
  // Finally, dampen pitch and roll angles
  angle_pitch = angle_pitch * 0.9 + real_pitch * 0.1;
  angle_roll = angle_roll * 0.9 + real_roll * 0.1;

  // Finally write calculated values into the array
  angle_array[0] = angle_pitch; 
  angle_array[1] = angle_roll;
  angle_array[2] = yaw_rate_gyro;
}
