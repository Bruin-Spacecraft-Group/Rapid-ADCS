struct DataSample {
  const double timeSec;
  const double rpm;
};

void parseTestConfig(int& test_config) {
  if (test_config < 1 || test_config > 6) {
    cout << "Please configure a valid test." << endl;
    cout << "  1. Speed vs torque" << endl;
    cout << "  2. Speed control precision" << endl;
    cout << "  3. Max speed" << endl;
    cout << "  4. Current vs continuous speed" << endl;
    cout << "  5. Frequency response" << endl;
    cout << "  6. Custom" << endl;
    cin >> test_config;
    parseTestConfig(test_config);
  }
}

long micros() {
  timespec t;
  clock_gettime(clock_id, &t);
  return t.tv_nsec*1000;
}

void delay(int milli) {
  usleep(milli*1000);
}

// Set voltage on DAC
void setVoltage(double VOLTAGE){
  uint16_t voltageBinary = 0;
  voltageBinary = (uint16_t) ((VOLTAGE / 5.0) * 16383.0);
  voltageBinary = voltageBinary << 2;
  sendCommand(CODE_LOAD, voltageBinary);
}

// Send command to DAC
void sendCommand(uint8_t COMMAND, uint16_t DATA){
  uint8_t data[3];
  data[0] = COMMAND;
  data[1] = (DATA >> 8) & 0xff;
  data[2] = (DATA << 0) & 0xff;
  // Wire.beginTransmission(ADDR);
  // Wire.write(data, 3);
  // Wire.endTransmission();

  // i2cWriteBlockData(i2c_handle, COMMAND, reinterpret_cast<char*>(data + 1), 3);
  if (write(i2c_handle, data, 3) != 3) {
    /* ERROR HANDLING: i2c transaction failed */
    cerr << "Failed to write to the i2c bus." << endl;
    cerr << "Error: " << errno << endl;
  }
}

// Set motor speed
void setMotorSpeed(double rpm, bool forward){
  if(!forward){
    // digitalWrite(DIR, LOW);
    gpioWrite(DIR, PI_LOW);
  }
  else{
    // digitalWrite(DIR, HIGH);
    gpioWrite(DIR, PI_HIGH);
  }
  double voltageSet = (rpm / 15000.0) * 5.0;
  setVoltage(voltageSet);
}

void setMotorSpeed(double rpm) {
  if (rpm >= 0) { setMotorSpeed(rpm, true); }
  else { setMotorSpeed(-rpm, false); }
}

void falling(int gpio, int level, uint32_t tick) {
  cycleIndex++;
  if(cycleIndex >= delayCycles){
    cycleIndex = 0;
    microSec = micros();
    pwm_value = microSec - prev_time;
    timeSec = (((microSec + prev_time / 2.0)) / 1000000.0);
    prev_time = microSec;
    rpm = delayCycles * 10.0 / (((double) pwm_value) / 1000000.0);

    if (initialTime == -1) { initialTime = timeSec; }
    const DataSample sample = {timeSec - initialTime, rpm}; // {timeSec, rpm}
    writer->try_write(sample);
    std::size_t failedWrites = writer->getFailedWrites();
    if (failedWrites % 10 == 0 && failedWrites > 0 && !suppress_lost_messages) {
      cerr << "Lost: "<< failedWrites << " messages" << endl;
    }
  }
}

