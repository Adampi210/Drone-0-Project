/*
 * This module contains functions for communicating with the peripheral device.
 */
#define ARRAY_SEND_SIZE 28 // Size of the array of data that will be sent

void send_data_to_peri(uint16_t* radio_data, double* angle_data, double pressure_data) {
  uint8_t array_to_send[ARRAY_SEND_SIZE]; // Array of values to send. The values must be Bytes so I need to convert them into a useful form

  // Radio data conversion and saving
  // Since the first 6 values are from 0 to 1023 then I can just split them
  // into 2 parts (ex 10 and 23) and send each as a distict byte
  // I use this loop to do that
  for(int i = 0; i < 12; i += 2) {
    array_to_send[i] = radio_data[i / 2] / 100;     // Set ith element to first part
    array_to_send[i + 1] = radio_data[i / 2] % 100; // Set i+1th element to second part
  }
  // The last radio values are represented as bits (either 0 or 1), so I can just set them to the array
  for(int i = 12; i < 15; ++i) {
    array_to_send[i] = radio_data[i - 6];
  }

  // Angle data conversion and saving
  // Angles are type double, but I will need only 2 digits after the comma
  // The angles are expected to go from -90 to 90 so 2 digits before comma should be sufficient

  for(int i = 0; i < 3; ++i) {
    array_to_send[3 * i + 15] = angle_data[i] < 0 ? 0 : 1; // This saves whether an angle is positive or negative (if positivie it is 1)
    array_to_send[3 * i + 16] = abs((int)angle_data[i]);               // First part of the angle (two first digits)
    array_to_send[3 * i + 17] = abs((int)(angle_data[i] * 100)) % 100; // Second part of the angle
  }

  // Pressure data conversion and saving
  // I use 4 digits after the comma, and the rest of the pressure digits before it
  array_to_send[24] = (int)(pressure_data * 10000) % 100; // Last 2 digits after the comma  XXXX.XXAA
  array_to_send[25] = (int)(pressure_data * 100) % 100;        // First 2 digits after the comma XXXX.AAXX
  array_to_send[26] = ((int)pressure_data) % 100;         // Last 2 digits before the comma XXAA.XXXX
  array_to_send[27] = ((int)pressure_data) / 100;         // First 2 digits before the comma AAXX.XXXX

  // Send all the data to the peripheral device using I2C
  Wire.beginTransmission(MOTOR_CONTROL_ADDR);
  Wire.write(array_to_send, ARRAY_SEND_SIZE);
  Wire.endTransmission(true);
}
