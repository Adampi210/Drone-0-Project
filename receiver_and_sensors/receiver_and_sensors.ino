/*
 * This program receives the data from the radio controller, reads the sensor values
 * and calculates necessary data values to be sent to the motor controller
 */
// Include the necessary libraries
#include <SPI.h>      // Used for communication with SPI modules
#include <nRF24L01.h> // Constants radio module library
#include <RF24.h>     // Radio communication library
#include <Wire.h>     // Two wire (I2C) library for communication with the second arduino

// Definitions
#define MOTOR_CONTROL_ADDR 0x09 // Address of the motor controller module (second microcontroller)
#define GYRO_ADDR 0x68 // Address of the gyroscope

// Define pins used to communicate with the radio module
#define CE_PIN 9
#define CSN_PIN 10

#define DATA_RECEV_SIZE 9 // Define size of the data sent by the controller
// Variables
// Radio module variables
RF24 radio_receiver(CE_PIN, CSN_PIN); // Radio receiver module object

// Gyroscope variables
double calculated_angles[3];   // Array of angles calculated from the IMU

// Data variables
uint16_t radio_data[DATA_RECEV_SIZE]; // data received from the radio controller module

// Time variables
unsigned long current_time; // Current time in the loop in microseconds
unsigned long previous_time_angle_calc, previous_time_baro; // previous time variables used to run independent conditions
unsigned long time_interval_angle_calc = 2000; // Interval that the angle calculation runs on (2ms)
unsigned long time_interval_baro_meas = 10000; // Interval that the pressure measurement runs on (10ms)

void setup() {
  // Initiate I2C communication
  Wire.begin();

  // Initialize the receiver
  init_receiver(radio_receiver);

  // Initialize the IMU
  gyro_init_config();
  calibrate_gyro();
  calibrate_gyro_yaw();
  
}

void loop() {
  current_time = micros(); // get current time since starting the program
  
  receive_data(radio_data, radio_receiver); // receive the radio data

  // Every 2ms calculate the angles and send that data to the second device
  if(current_time - previous_time_angle_calc >= time_interval_angle_calc) {
    previous_time_angle_calc = current_time;
    calculate_angles(calculated_angles); // Calculate all the angles
    send_data_to_peri(radio_data, calculated_angles); // Send the data to the peripheral device
  }
}
