#include "stdint.h"
#include "MPU9250.h"
#include "Wire.h"
#include "Arduino.h"

MPU9250::MPU9250() {
  Wire.begin();
  _address = MPU9250_SLAVE_ADDRESS;
  _mag_address = MPU9250_MAGNET_ADDRESS;
}


uint8_t MPU9250::readRegister(const uint8_t register_addr) {
  uint8_t data = 0;

  Wire.beginTransmission(_address);
  Wire.write(register_addr);
  Wire.endTransmission();

  Wire.requestFrom((int)_address, 1);

  while(Wire.available()) {
    data = Wire.read();
  }

  return data;
}


int16_t MPU9250::readRegisters(const uint8_t msb_register, const uint8_t lsb_register) {
  uint8_t msb = readRegister(msb_register);
  uint8_t lsb = readRegister(lsb_register);
  return (((int16_t)msb) << 8) | lsb;
}


void MPU9250::writeRegister(const uint8_t register_addr, const uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(register_addr);
  Wire.write(value);
  Wire.endTransmission();
}


uint8_t MPU9250::readMagRegister(const uint8_t register_addr) {
  uint8_t data = 0;
  
  Wire.beginTransmission(_mag_address);
  Wire.write(register_addr);
  Wire.endTransmission();

  Wire.requestFrom((int)_mag_address, 1);

  while(Wire.available()) {
    data = Wire.read();
  }

  return data;
}


void MPU9250::writeMagRegister(const uint8_t register_addr, const uint8_t value) {
  Wire.beginTransmission(_mag_address);
  Wire.write(register_addr);
  Wire.write(value);
  Wire.endTransmission();
}


void MPU9250::initialize(void) {
  _who_am_i = readRegister(MPU9250_WAI_ADDRESS);
  if (_who_am_i != 0x71) exit(0);

  // Acceleration, Gyroscope Sensing
  writeRegister(MPU9250_PWR_MGMT_1, 0x00);
  writeRegister(MPU9250_INT_PIN_CFG, 0x02);
  writeMagRegister(MPU9250_MAG_CNTL, 0x06);

  // get Acceleration, Gyroscope Config
  uint8_t gyroConfig = readRegister(MPU9250_GYRO_CONFIG);
  uint8_t accelConfig = readRegister(MPU9250_ACCEL_CONFIG);
  uint8_t magnetConfig = readMagRegister(MPU9250_MAG_CNTL);

  gyroConfig = gyroConfig << 3;
  gc = gyroConfig >> 6;
  accelConfig = accelConfig << 3;
  ac = accelConfig >> 6;
  mc = magnetConfig >> 4;

  switch(gc) {
    case 0b00:  // gyro_scale ±250(deg/sec)
      _gyro_scale = 250.0;
      break;
    case 0b01:  // gyro_scale ±500(deg/sec)
      _gyro_scale = 500.0;
      break;
    case 0b10:  // gyro_scale ±1000(deg/sec)
      _gyro_scale = 1000.0;
      break;
    case 0b11:  // gyro_scale ±2000(deg/sec)
      _gyro_scale = 2000.0;
      break;
    default:
      break;
  }

  switch(ac) {
    case 0b00:  // accel_scale ±2(g)
      _accel_scale = 2.0;
      break;
    case 0b01:  // accel_scale ±4(g)
      _accel_scale = 4.0;
      break;
    case 0b10:  // accel_scale ±8(g)
      _accel_scale = 8.0;
      break;
    case 0b11:  // accel_scale ±16(g)
      _accel_scale = 16.0;
      break;
    default:
      break;
  }

  switch(mc) {
    case 0b0000:  // magnet_scale 14bit
      _mag_scale = 8190.0;
      break;
    case 0b0001:  // magnet_scale 16bit
      _mag_scale = 32760.0;
      break;
    default:
      break;
  }
}


void MPU9250::getSensor(void) {
  getAcceleration();
  getTemperatureDegC();
  getGyroscope();
  getMagnet();
  getRoll();
  getPitch();
  getCompass();
}


void MPU9250::getAcceleration(void) {
  ax = getAccelerationX();
  ay = getAccelerationY();
  az = getAccelerationZ();
  accelX = (double)ax * _accel_scale / 32768.0;
  accelY = (double)ay * _accel_scale / 32768.0;
  accelZ = (double)az * _accel_scale / 32768.0;
}


void MPU9250::getTemperatureDegC(void) {
  temp = getTemperature();
  temperature = (double)temp / 333.87 + 21.0;
}


void MPU9250::getGyroscope(void) {
  gx = getGyroscopeX();
  gy = getGyroscopeY();
  gz = getGyroscopeZ();
  gyroX = (double)gx * _gyro_scale / 32768.0;
  gyroY = (double)gy * _gyro_scale / 32768.0;
  gyroZ = (double)gz * _gyro_scale / 32768.0;
}


void MPU9250::getMagnet(void) {
  drdy = readMagRegister(MPU9250_MAG_ST1);
  mx = getMagnetX();
  my = getMagnetY();
  mz = getMagnetZ();
  magX = (double)mx / _mag_scale * 4912.0 - offsetMagX;
  magY = (double)my / _mag_scale * 4912.0 - offsetMagY;
  magZ = (double)mz / _mag_scale * 4912.0 - offsetMagZ;
  hofl = readMagRegister(MPU9250_MAG_ST2);
}


void MPU9250::getCompass(void) {
  //compass = atan2(magX, magY) * RAD_TO_DEG;
  double hx = magX * cos(roll * DEG_TO_RAD) + magZ * sin(roll * DEG_TO_RAD);
  double hy = magY * cos(pitch * DEG_TO_RAD) + sin(pitch * DEG_TO_RAD) * (magX * sin(roll * DEG_TO_RAD) - magZ * cos(roll * DEG_TO_RAD));
  compass = atan2(hx, hy) * RAD_TO_DEG;
  if (compass < 0) compass += 360;
}


void MPU9250::getRoll(void) {
  roll = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
}


void MPU9250::getPitch(void) {
  pitch = atan(-accelX / accelZ) * RAD_TO_DEG;
}


double MPU9250::compass2mathAngle(void) {
  double ma = compass;
  if (compass >= 180) ma -= 360;
  return ma;
}


int16_t MPU9250::getAccelerationX(void) {
  return readRegisters(MPU9250_ACCEL_XOUT_H, MPU9250_ACCEL_XOUT_L);
}


int16_t MPU9250::getAccelerationY(void) {
  return readRegisters(MPU9250_ACCEL_YOUT_H, MPU9250_ACCEL_YOUT_L);
}


int16_t MPU9250::getAccelerationZ(void) {
  return readRegisters(MPU9250_ACCEL_ZOUT_H, MPU9250_ACCEL_ZOUT_L);
}


int16_t MPU9250::getTemperature(void) {
  // TEMP_degC = ((TEMP_OUT- TEMP_Offset)/TEMP_Sensitivity)+21degC
  // TEMP_Sensitivity = 340 or 333.87
  return readRegisters(MPU9250_TEMP_OUT_H, MPU9250_TEMP_OUT_L);
}


int16_t MPU9250::getGyroscopeX(void) {
  return readRegisters(MPU9250_GYRO_XOUT_H, MPU9250_GYRO_XOUT_L);
}


int16_t MPU9250::getGyroscopeY(void) {
  return readRegisters(MPU9250_GYRO_XOUT_H, MPU9250_GYRO_XOUT_L);
}


int16_t MPU9250::getGyroscopeZ(void) {
  return readRegisters(MPU9250_GYRO_XOUT_H, MPU9250_GYRO_XOUT_L);
}


uint8_t MPU9250::gyroConfig(void) {
  return readRegister(MPU9250_GYRO_CONFIG);
}


uint8_t MPU9250::accelConfig(void) {
  return readRegister(MPU9250_ACCEL_CONFIG);
}


int16_t MPU9250::getMagnetX(void) {
  uint8_t low = readMagRegister(MPU9250_MAG_XOUT_L);
  uint8_t high = readMagRegister(MPU9250_MAG_XOUT_H);
  return (((int16_t)high) << 8) | low;
}


int16_t MPU9250::getMagnetY(void) {
  uint8_t low = readMagRegister(MPU9250_MAG_YOUT_L);
  uint8_t high = readMagRegister(MPU9250_MAG_YOUT_H);
  return (((int16_t)high) << 8) | low;
}


int16_t MPU9250::getMagnetZ(void) {
  uint8_t low = readMagRegister(MPU9250_MAG_ZOUT_L);
  uint8_t high = readMagRegister(MPU9250_MAG_ZOUT_H);
  return (((int16_t)high) << 8) | low;
}


void MPU9250::calibrateMagnet(void) {
  getMagnet();
  if (mmMagX[0] == -999999) mmMagX[0] = magX;
  if (mmMagX[1] == -999999) mmMagX[1] = magX;
  if (mmMagY[0] == -999999) mmMagY[0] = magY;
  if (mmMagY[1] == -999999) mmMagY[1] = magY;
  if (mmMagZ[0] == -999999) mmMagZ[0] = magZ;
  if (mmMagZ[1] == -999999) mmMagZ[1] = magZ;

  if (mmMagX[0] > magX) mmMagX[0] = magX;
  if (mmMagX[1] < magX) mmMagX[1] = magX;
  if (mmMagY[0] > magY) mmMagY[0] = magY;
  if (mmMagY[1] < magY) mmMagY[1] = magY;
  if (mmMagZ[0] > magZ) mmMagZ[0] = magZ;
  if (mmMagZ[1] < magZ) mmMagZ[1] = magZ;
  
  offsetMagX = mmMagX[0] + (mmMagX[1] - mmMagX[0]) / 2.0;
  offsetMagY = mmMagY[0] + (mmMagY[1] - mmMagY[0]) / 2.0;
  offsetMagZ = mmMagZ[0] + (mmMagZ[1] - mmMagZ[0]) / 2.0;
}


void MPU9250::setOffsetMagnet(double offsetX, double offsetY, double offsetZ) {
  offsetMagX = offsetX;
  offsetMagY = offsetY;
  offsetMagZ = offsetZ;
}






Kalman::Kalman() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

