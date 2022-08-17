/*
 * 
 */

#include <Wire.h>   // Two wire (I2C) library for communication with the first arduino
#include <Servo.h>  // Servo library used for controlling the motors

#define DEV_ADDR 0x09 // Address of this device used for I2C communication

#define ESC_1 9   // Pin used to control the first ESC (layout seen on the PID_module tab)
#define ESC_2 10  // Pin used to control the second ESC
#define ESC_3 5   // Pin used to control the third ESC
#define ESC_4 6   // Pin used to control the fourth ESC
#define SERVO 3   // Pin used to control the camera servo

#define DATA_SIZE 22 // Size of the data sent by the controller device

// Variables
// Sensor and controller data
int all_data[DATA_SIZE] = {};

// Motors and servo
Servo esc_1, esc_2, esc_3, esc_4; // Each esc_x variable corresponds to each motor 
Servo servo_cam;                  // Servo setting the camera inclination
float esc_1_pwm, esc_2_pwm, esc_3_pwm, esc_4_pwm; // PWM signal send to each esc modeling their speeds
float servo_position;                             // PWM signal storing position of the servo

// Time variables
unsigned long current_time; // Current time in the loop in microseconds
unsigned long previous_time_motors; // previous time variable used to run motors
unsigned long time_interval_motors = 1000; // Interval that the motors loop runs on (4ms)


void setup() {
  delay(100); // First delay to let everything else start running

  Serial.begin(115200);
  
  Wire.begin(DEV_ADDR); // Begin I2C communication as a peripheral device with address DEV_ADDR

  // Initiate connection with each esc and the servo by calling attach
  esc_1.attach(ESC_1);
  esc_2.attach(ESC_2);
  esc_3.attach(ESC_3);
  esc_4.attach(ESC_4);
  servo_cam.attach(SERVO);

  // Write initial positions and PWM to escs
  // value of 1000 indicates null PWM signal
  esc_1.writeMicroseconds(1000);
  esc_2.writeMicroseconds(1000);
  esc_3.writeMicroseconds(1000);
  esc_4.writeMicroseconds(1000);

  // Write initial camera position
  servo_cam.write(75);

  delay(7000); // Delay for 7s to allow the motors finish their set up procedure

  Wire.onReceive(receive_event); // When the device receives data via I2C run receive_event function
  delay(200); // Delay for 200ms to receive some data
}

void loop() {
  current_time = micros(); // get current time since starting the program

  if(current_time - previous_time_motors >= time_interval_motors) {
    previous_time_motors = current_time;
    calculate_pid_set_values(esc_1, esc_2, esc_3, esc_4, all_data);
  }

}

// This function receives the data from the controller device and calls another function that writes it into an array
// It is run every time a receive event is detected (data has been received from the controller device)
void receive_event() {
  receive_data_from_ctrl(all_data); // call the function to write values into an array
}
