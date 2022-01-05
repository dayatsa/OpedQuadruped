void imuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}


void readPitchRoll() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  filterData();
  
  // Calculate Pitch & Roll
  pitch = -(atan2((float)filtered_accelerometer_x, sqrt((float)filtered_accelerometer_y * (float)filtered_accelerometer_y + (float)filtered_accelerometer_z * (float)filtered_accelerometer_z)) * 180.0) / M_PI;
  roll = (atan2((float)filtered_accelerometer_y, (float)filtered_accelerometer_z) * 180.0) / M_PI;

  pitch = pitch * 1.28082915164085 + 1.94686031049409;
  roll = roll * 1.214071024 - 5.05862926724484;
}


void filterData() {
  filtered_accelerometer_x = (float)accelerometer_x * ALPHA + (filtered_accelerometer_x * (1.0 - ALPHA));
  filtered_accelerometer_y = (float)accelerometer_y * ALPHA + (filtered_accelerometer_y * (1.0 - ALPHA));
  filtered_accelerometer_z = (float)accelerometer_z * ALPHA + (filtered_accelerometer_z * (1.0 - ALPHA));
}
