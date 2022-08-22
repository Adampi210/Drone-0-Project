/*
 * This tab contains functions that control the camera servo setting the angle
 * of the front camera.
 */

#define MAX_CAM_POS 80 // Maximum position of the camera
#define MIN_CAM_POS 10  // Minimum position of the camera

// This function moves the camera to the desired position as specified by the user
void move_camera(Servo camera_pin, int desired_camera_position) {
  // Map the position to be between max and min possible positions
  int camera_position = map(desired_camera_position, 0, 1023, MIN_CAM_POS, MAX_CAM_POS);
  camera_pin.write(camera_position);
}
