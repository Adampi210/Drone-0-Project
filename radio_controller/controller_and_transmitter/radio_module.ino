// This tab contains function used for radio communication
// Variables
const byte receiver_address[14] = {'a','X','6','L','$','C','Y','7','9','1','h','D','*','%'}; // address of the receiver

// This function initializes the transmitter to start sending data
void init_transmitter(RF24 radio_module) {
  radio_module.begin(); // begin operation of the module
  radio_module.openWritingPipe(receiver_address); // open the writing pipe for sending the data
  radio_module.stopListening(); // stop listening to the incoming messages (necessery if I want to write)
  radio_module.setPALevel(RF24_PA_MAX); // set the power level of the module to max (this way the range of transmitting is as high as it gets)
  radio_module.setRetries(3, 5);        // set the number of retries if failed to submit (5 retries each 750us apart(3 * 250us))
  Serial.println("Init Success");
}

// This function sends the data to the receiver
void transmit_data(RF24 radio_module, uint16_t* all_data) {
  radio_module.write(all_data, DATA_SIZE * sizeof(all_data));
}
