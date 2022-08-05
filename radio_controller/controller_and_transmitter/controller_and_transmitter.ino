/*
 * This program reads the values from the pilot controlls and sends them to the receiver module
 * Also, I didn't have enough pins on the Arduino Uno I used, so some of the 
 * controller values are read in the second Arduino (Nano Every) and sent to this
 * Arduino via I2C communication
 */
// Include necessary libraries
#include <SPI.h>      // Used for communication with SPI modules
#include <nRF24L01.h> // Constants radio module library
#include <RF24.h>     // Radio communication library
#include <Wire.h>     // Two wire (I2C) library for communication with the second arduino

// Definitions of the controlling elements
#define R_POT A3            // Right potentiometer
#define L_POT A0            // Left potentiometer
#define R_JOYSTICK_Y A1     // Right joystick y axis
#define R_JOYSTICK_X A2     // Right joystick x axis
#define SWITCH 4            // Switch
#define R_JOYSTICK_BUTTON 5 // Right joystick button
#define L_JOYSTICK_BUTTON 3 // Left joystick button

#define LCD_CONTROL_ADDR 0x09 // LCD controller address (second microcontroller)

// Define pins used to communicate with the radio module
#define CE_PIN 9
#define CSN_PIN 10

#define DATA_SIZE 9
// Variables
// Radio module variables
RF24 radio_transmitter(CE_PIN, CSN_PIN); // Radio transmitter module object

// Data varables
uint16_t data_to_transmit[9]; // Data values to be transmitted to the receiver

// Time variables
unsigned long current_time;
unsigned long previous_time; 
unsigned long time_interval = 4;

void setup() {
  // Set the modes of the controller pins
  pinMode(R_POT, INPUT);
  pinMode(L_POT, INPUT);
  pinMode(R_JOYSTICK_Y, INPUT);
  pinMode(R_JOYSTICK_X, INPUT);
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(L_JOYSTICK_BUTTON, INPUT_PULLUP);
  pinMode(R_JOYSTICK_BUTTON, INPUT_PULLUP);
  
  Wire.begin(); // initiate the I2C communication
    
  init_transmitter(radio_transmitter); // Initiate the transmitter

  Serial.begin(9600);
}

void loop() {
  current_time = millis();
  if(current_time - previous_time >= time_interval) {
    previous_time = current_time;
    
    // Read data from all devices
    read_data_devices(data_to_transmit, LCD_CONTROL_ADDR);

    // Send data to the peripheral device
    send_LCD_data(data_to_transmit[6], data_to_transmit[1], LCD_CONTROL_ADDR);
    // Transmit the data to the receiver
    transmit_data(radio_transmitter, data_to_transmit);
    Serial.println(data_to_transmit[0]);
  }
}
