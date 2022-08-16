/*
 * This module contains functions for communicating with the controller device.
 */

#define ARRAY_RECEIV_SIZE 22 // Size of the array of received data (differs from the sent data because the values are converted)

// This function reads the data from the controller device and saves it into an array
void receive_data_from_ctrl(int* data_array) {
  // Read radio data
  // First convert the parts that were split into 2 back into values from 0 to 1023
  for(int i = 0; i < 6; ++i) {
    data_array[i] = Wire.read() * 100 + Wire.read();
  }
  // Then read the rest of the radio values normally
  for(int i = 6; i < 9; ++i) {
    data_array[i] = Wire.read();
  }

  // Read angle data
  // Since I am reading into integer array I can't convert these back to double yet, so I read them normally
  // I could have done all that in one for loop but this way it is clearer which values correspond to which calculation
  for(int i = 9; i < 18; ++i) {
    data_array[i] = Wire.read();
  }

  // Read pressure data
  for(int i = 18; i < 22; ++i) {
    data_array[i] = Wire.read();
  }
}

// TODO
void convert_data(int* data_array, uint16_t* radio_data, double* angle_data, double* pressure_data) {
  
}
