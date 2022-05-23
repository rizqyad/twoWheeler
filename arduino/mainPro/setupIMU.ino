void setupMPU(){
  // initialize devices
  Serial.println("Initializing I2C devices...");
  // initialize mpu6050
  accelgyro.initialize();
  Serial.println((accelgyro.getDeviceID() == 0x34) ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // set MPU6050 offsets
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);
  
}
void setupHMC(){
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L
  // initialize hmc5883l
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}

double calc_pitch(float xGf, float yGf, float zGf){
  return atan(-xGf / sqrt(yGf * yGf + zGf * zGf)) * RAD_TO_DEG;
}
