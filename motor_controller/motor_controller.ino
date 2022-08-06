/*
 * 
 */

#include <Wire.h>
#include <Servo.h>

#define ESC_1 9
#define ESC_2 10
#define ESC_3 5
#define ESC_4 6
#define SERVO 3

// Variables
// Motors and servo
Servo esc_1, esc_2, esc_3, esc_4; // Each esc_x variable corresponds to each motor 
Servo servo_cam;                  // Servo setting the camera inclination
float esc_1_pwm, esc_2_pwm, esc_3_pwm, esc_4_pwm; // PWM signal send to each esc modeling their speeds
float servo_position;                             // PWM signal storing position of the servo
void setup() {
  delay(100); // First delay to let everything else start running

  Serial.begin(115200);

  // Wire.begin(9); // Begin I2C communication as a peripheral device with address 0x09

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
}

void loop() {
  // put your main code here, to run repeatedly:

}
