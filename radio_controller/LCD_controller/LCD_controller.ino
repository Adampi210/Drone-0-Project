/*
 * This program reads the remaining values of the controller, and shows other values on an LCD
 */
// Include necessary libraries
#include <Wire.h>     // Two wire (I2C) library for communication with the second arduino
#include <LiquidCrystal.h> // LCD controlling library

// Definitions of the controlling elements
#define L_JOYSTICK_Y A6 // Left joystick y axis
#define L_JOYSTICK_X A3 // Left joystick x axis

// Definitions for the LCD
#define LCD_X 16
#define LCD_Y 2
#define BRIGHTNESS_ADJUST_PIN 9

// Definitions for the I2C
#define DEV_ADDR 0x09

// Variables
// LCD variables
LiquidCrystal lcd_module(2, 3, 4, 5, 6, 7); // lcd object with specified pins
int if_drone_on;          // Value storing if a drone is on
int motors_pwr_percent;   // Value storing the percentage of max power the motors are running at

// Messages to be printed on the LCD
String first_line = "Drone:";
String second_line = "% motors:";

// I2C variables
uint8_t data_to_send_controller[] = {0, 0, 0, 0}; // data to be sent to the controller device

void setup() {
  // Setup the LCD
  pinMode(BRIGHTNESS_ADJUST_PIN, OUTPUT);
  analogWrite(BRIGHTNESS_ADJUST_PIN, 150); // set moderate brightness
  lcd_module.begin(LCD_X, LCD_Y); // initiate the lcd
  lcd_module.clear();             // clear anything that was on the LCD
  // Print the message on the lcd
  lcd_module.setCursor(0, 0);
  lcd_module.print(first_line);
  lcd_module.setCursor(0, 1);
  lcd_module.print(second_line);

  // Set the modes of the control devices
  pinMode(L_JOYSTICK_Y, INPUT);
  pinMode(L_JOYSTICK_X, INPUT);

  // I2C communication setup
  Wire.begin(DEV_ADDR);
  Wire.onRequest(request_event); // If I get a request from the device run the request event function
  Wire.onReceive(receive_event); // On receiving data run the receive_event function
}

void loop() {
  read_and_convert(data_to_send_controller); // read the left joystick values and save them to an array

  // lcd communication
  // print the value of if_drone_on
  lcd_module.setCursor(first_line.length(), 0);
  lcd_module.print(if_drone_on);
  // print the value of motors_pwr_percent
  lcd_module.setCursor(second_line.length(), 1);
  if(motors_pwr_percent < 10) {
    lcd_module.print("0");
  }
  lcd_module.print(motors_pwr_percent);
  lcd_module.print(" ");
}

// I2C functions
// This function sends the data in the local array when a request is received
void request_event() {
  uint8_t local_send_array[4]; // local array that will be sent
  // save the values to the local array
  for(int i = 0; i < 4; ++i) {
    local_send_array[i] = data_to_send_controller[i];
  }
  // Send the data with I2C
  Wire.write(local_send_array, sizeof(local_send_array)/sizeof(local_send_array[0]));
}

// This function reads the values sent from the controller via I2C
void receive_event() {
  if_drone_on = Wire.read();
  motors_pwr_percent = Wire.read();
}
