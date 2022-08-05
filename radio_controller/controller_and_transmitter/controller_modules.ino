/*
 * This module reads the data from controller devices
 * As well as communicates with the LCD controller to read rest of the data
 * and send some data to be displayed on the LCD.
 */

// This function reads the data from the controller devices
void read_data_devices(uint16_t* all_data, int peripheral_device_addr) {
  all_data[0] = 1023 - analogRead(R_POT); // Right potentiometer reading
  all_data[1] = analogRead(L_POT);        // Left potentiometer reading

  // The next two readings must be received via I2C communication from the peripheral device
  Wire.requestFrom(peripheral_device_addr, 4); // request 4 bytes from the peripheral device
  // Write the read values to all_data array
  for(int i = 2; i < 4; ++i) {
    // I can only send bytes but the reading is from 0 to 1023 so I broke it up into 2
    // This loop patches it up together and saves the data to the array
    // First it reads two bytes sent from the peripheral device
    uint16_t temp_val_left_side = Wire.read();
    uint16_t temp_val_right_side = Wire.read();
    all_data[i] = temp_val_left_side * 100 + temp_val_right_side; // And then saves the right value to the array
  }

  // Then read the rest of the controller devices' values
  all_data[4] = 1023 - analogRead(R_JOYSTICK_Y);     // Right joystick y axis reading
  all_data[5] = 1023 - analogRead(R_JOYSTICK_X);     // Right joystick x axis reading
  all_data[6] = digitalRead(SWITCH);                 // Switch reading
  all_data[7] = 1 - digitalRead(R_JOYSTICK_BUTTON);  // Right joystick button reading
  all_data[8] = 1 - digitalRead(L_JOYSTICK_BUTTON);  // Left joystick button reading
}

// This function sends the data to be displayed on the LCD to the peripheral device
void send_LCD_data(uint8_t if_drone_on, int motor_analog_read, int peripheral_dev_addr) {
  uint8_t motor_pwr_percent = map(motor_analog_read, 0, 1023, 0, 99); // percentage of the max power the motors are running at
  // Send the values via I2C
  Wire.beginTransmission(peripheral_dev_addr);
  Wire.write(if_drone_on);
  Wire.write(motor_pwr_percent);
  Wire.endTransmission();
}
