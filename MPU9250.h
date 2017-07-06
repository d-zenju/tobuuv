#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <math.h>

// MPU9250 Acceleration, Gyroscope Slave Address (AD0=H: 0x69)
#define MPU9250_SLAVE_ADDRESS           0x68

// MPU9250 Magnet Slave Address
#define MPU9250_MAGNET_ADDRESS          0x0C

// MPU9250 Acceleration, Gyroscope Who Am I
#define MPU9250_WAI_ADDRESS             0x75

// MPU9250 Acceleration, Gyroscope initialize
#define MPU9250_PWR_MGMT_1              0x6B
#define MPU9250_INT_PIN_CFG             0x37

// MPU9250 Acceleration Address
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40

// MPU9250 Temperature Address
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42

// MPU9250 Gyroscope Address
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48

// MPU9250 Magnet Address
#define MPU9250_MAG_ST1                 0x02
#define MPU9250_MAG_XOUT_L              0x03
#define MPU9250_MAG_XOUT_H              0x04
#define MPU9250_MAG_YOUT_L              0x05
#define MPU9250_MAG_YOUT_H              0x06
#define MPU9250_MAG_ZOUT_L              0x07
#define MPU9250_MAG_ZOUT_H              0x08
#define MPU9250_MAG_ST2                 0x09
#define MPU9250_MAG_CNTL                0x0A

// MPU9250 Config
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_ACCEL_CONFIG            0x1C


class MPU9250 {
  public:
    MPU9250();
    void initialize(void);
    void getSensor(void);
    void getAcceleration(void);
    void getTemperatureDegC(void);
    void getGyroscope(void);
    void getMagnet(void);
    void getCompass(void);
    void getRoll(void);
    void getPitch(void);

    void getSensors(void);
    void calcSensors(void);

    double compass2mathAngle(void);

    uint8_t gyroConfig(void);
    uint8_t accelConfig(void);

    void calibrateMagnet(void);
    void setOffsetMagnet(double offsetX, double offsetY, double offsetZ);
    void calibrateAcceleration(void);
    void setOffsetAcceleration(double offsetX, double offsetY, double offsetZ);

    int16_t getAccelerationX(void);
    int16_t getAccelerationY(void);
    int16_t getAccelerationZ(void);
    
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t mx;
    int16_t my;
    int16_t mz;

    uint8_t gc;
    uint8_t ac;
    
    double accelX;
    double accelY;
    double accelZ;
    double temperature;
    double gyroX;
    double gyroY;
    double gyroZ;
    double magX;
    double magY;
    double magZ;

    double compass;
    double roll;
    double pitch;

    double offsetMagX = 0.0;
    double offsetMagY = 0.0;
    double offsetMagZ = 0.0;
    double scaleMagX = 1.0;
    double scaleMagY = 1.0;
    double scaleMagZ = 1.0;
    
    // Calibration Magnet(min, max)
    double mmMagX[2] = {-999999, -999999};
    double mmMagY[2] = {-999999, -999999};
    double mmMagZ[2] = {-999999, -999999};
    

  private:
    uint8_t readRegister(const uint8_t register_addr);
    int16_t readRegisters(const uint8_t msb_register, const uint8_t lsb_register);
    uint8_t readMagRegister(const uint8_t register_addr);
    void writeRegister(const uint8_t register_addr, const uint8_t value);
    void writeMagRegister(const uint8_t register_addr, const uint8_t value);

    int16_t getTemperature(void);
    int16_t getGyroscopeX(void);
    int16_t getGyroscopeY(void);
    int16_t getGyroscopeZ(void);
    int16_t getMagnetX(void);
    int16_t getMagnetY(void);
    int16_t getMagnetZ(void);
    
    uint8_t _address;
    uint8_t _mag_address;
    uint8_t _who_am_i;

    // Acceleration, Gyroscope Scale Select
    double _gyro_scale;
    double _accel_scale;
    double _mag_scale;

    // Magnet Status
    uint8_t drdy;
    uint8_t hofl;
    uint8_t mc;

    double root;

    // Accel, Gyro, Temp
    uint8_t agtm[22];
    
    //double GyroMeasError = M_PI * (40.0 / 180.0);   // gyroscope measurement error in rads/s (start at 40 deg/s)
    //double beta = sqrt(3.0 / 4.0) * GyroMeasError;  // compute beta
    //double deltat = 0.0;  // integration interval for both filter schemes
};


class Kalman {
public:
    Kalman();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // Used to set angle, this should be set as the starting angle
    float getRate(); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(float Q_angle);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};


#endif /* _MPU9250_H_ */
