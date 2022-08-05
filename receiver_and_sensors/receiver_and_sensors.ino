/*
 * This program receives the data from the radio controller, reads the sensor values
 * and calculates necessary data values to be sent to the motor controller
 */
// Include the necessary libraries
#include <SPI.h>      // Used for communication with SPI modules
#include <nRF24L01.h> // Constants radio module library
#include <RF24.h>     // Radio communication library
#include <Wire.h>     // Two wire (I2C) library for communication with the second arduino
#include <MS5611.h>   // Barometer library

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

// Barometer variables
MS5611 barometer_sensor; // Barometer sensor object
uint32_t raw_temp;       // temperature reading from the barometer
uint32_t raw_press;      // pressure reading from the barometer

// Gyroscope variables

// Data variables
uint16_t all_data[21]; // All data read from sensors, controller, etc.
uint8_t data_to_motor_ctr[21]; // Data to be sent to the motor controller (as byte chunks)
double calculated_angles[3];   // Array of angles calculated from the IMU
// Time variables
unsigned long current_time;
unsigned long previous_time, previous_time_angle_calc; 
unsigned long time_interval_angle_calc = 4000;

void setup() {
  // Initiate I2C communication
  Wire.begin();

  Serial.begin(115200);

  // Initialize the receiver
  init_receiver(radio_receiver);
  Serial.println("init_receiver done");
  // Initialize the IMU
  gyro_init_config();
  Serial.println("gyro_init_config done");
  calibrate_gyro();
  Serial.println("calibrate_gyro done");
  while(1);
  calibrate_accel();
  Serial.println("calibrate_accel done");
  calibrate_gyro_yaw();
  Serial.println("calibrate_gyro_yaw done");

  
}

void loop() {
  current_time = micros();
  
  receive_data(all_data, radio_receiver);
  
  if(current_time - previous_time_angle_calc >= time_interval_angle_calc) {
    previous_time_angle_calc = current_time;
    calculate_angles(calculated_angles);
    Serial.print("Pitch: ");
    Serial.print(calculated_angles[0]);
    Serial.print(" Roll: ");
    Serial.print(calculated_angles[1]);
    Serial.print(" Yaw rate: ");
    Serial.print(calculated_angles[2]);
  }

}
