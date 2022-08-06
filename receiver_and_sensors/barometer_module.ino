/*
 * This tab contains barometer functions. The sensor I used was MS5611,  
 * For its operation, I was using a designated library: https://github.com/RobTillaart/MS5611.git
 * Note: The sensor is light sensitive so I added a black casing around it to prevent any
 * interference with the measurement.
 */

// This function initializes the barometer and sets its operation parameters
void baro_init(MS5611 barometer_module) {
  barometer_module.begin(); // Initiate the barometer operation
  barometer_module.setOversampling(OSR_ULTRA_HIGH); // Set the oversampling to ultra high - this way the measurement will be most accurate
  // This poses a problem as the measurement time will be 8.22ms, so the measurement has to be independent of the angle measurement (otherwise everything would slow down)
  // To counter this the pressure will remain constant for 10ms and then change after its measurement is taken, with the angle measurement going on independently in the background
  delay(20);
}

// This function takes the measurement of the pressure and returns it
double baro_get_pressure(MS5611 barometer_module) {
  barometer_module.read(); // Read data from the sensor
  return barometer_module.getPressure(); // Return the pressure
}

// This function gets average pressure of 100 measurements
// I use it initially to find the starting point pressure
double get_average_pressure(MS5611 barometer_module) {
  double pressure_measurement = 0; // average pressure measurement

  // Add 100 measurements together
  for(int i = 0; i < 100; ++i) {
    pressure_measurement += baro_get_pressure(barometer_module);
  }
  pressure_measurement /= 100; // divide the result by 100
  return pressure_measurement; // return the average pressure
}
