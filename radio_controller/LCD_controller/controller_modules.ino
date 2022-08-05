// This function reads the values of the joystick and
// converts them to be sent to the controller device
void read_and_convert(uint8_t* array_to_send) {
  // Initialize and read the joystick values
  int left_joystick_reading_y = 1023 - analogRead(L_JOYSTICK_Y);
  int left_joystick_reading_x = 1023 - analogRead(L_JOYSTICK_X);

  // convert values and save them in the array
  array_to_send[0] = left_joystick_reading_y / 100;
  array_to_send[1] = left_joystick_reading_y % 100;
  array_to_send[2] = left_joystick_reading_x / 100;
  array_to_send[3] = left_joystick_reading_x % 100;
}
