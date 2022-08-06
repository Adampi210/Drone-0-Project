/* 
 * This tab contains function used for radio communication. The module I used
 * was nRF24L01+.
 */
// Variables
uint16_t data_received[DATA_RECEV_SIZE];       // Array of received values from the controller
const byte receiver_address[14] = {'a','X','6','L','$','C','Y','7','9','1','h','D','*','%'}; // address of the receiver (transmitter finds a module with that address and sends the data there)

// This function initializes the receiver to start listening to the data sent
void init_receiver(RF24 radio_module) {
  radio_module.begin(); // begin operation of the module
  radio_module.openReadingPipe(0, receiver_address); // open the first pipe for reading the data (can have more pipes) (pipe 0 used for writing)
  radio_module.setPALevel(RF24_PA_MAX); // set the power level of the module to max (this way the range of receiving is as high as it gets)
  radio_module.startListening();        // start listening for the data sent on the opened pipe
}

// This function receives the data from the controller and writes it to an array
void receive_data(uint16_t* data_array, RF24 radio_module) {
  // If the data is available to read, read it
  if(radio_module.available()) {
    radio_module.read(data_received, sizeof(data_received)); // Read the data into data_received array
    // Write the values into an array passed as a parameter
    // This could have been done with a for loop, but I wanted to put descriptions of what each value represents
    
    data_array[0] = data_received[0]; // Right potentiometer
    data_array[1] = data_received[1]; // Left potentiometer
    data_array[2] = data_received[2]; // Left joystick y
    data_array[3] = data_received[3]; // Left joystick x
    data_array[4] = data_received[4]; // Right joystick y
    data_array[5] = data_received[5]; // Right joystick x
    data_array[6] = data_received[6]; // Controller Switch
    data_array[7] = data_received[7]; // Right joystick button
    data_array[8] = data_received[8]; // Left joystick button
  }
}
