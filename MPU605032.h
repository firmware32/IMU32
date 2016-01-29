// MPU605032.h

#ifndef _MPU605032_h
#define _MPU605032_h

 

#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_REG_USER_CTRL         (0x6A) // User Control
#define MPU6050_REG_INT_PIN_CFG       (0x37)

#define ACC_SMOOTH_HISTORY_COUNT   256

#include "arduino.h"
#include <Wire.h>
#include "SPI2C.h"
#include "I2C32.h"

#include "Sensor32.h"
class MPU605032 : public I2C32
{
 protected:
	byte  buffer[14];
	byte AccScaleID;
	byte GyroScaleID;
	void setAccNormalize(sensor_int_vect * a);
	void setGyroNormalize(sensor_int_vect * a);
 public:
	 MPU605032(sensor_vect * gyroNormalized, sensor_vect * axisDegrees);
	 void setConfig();
	 sensor_orientation_t AccOrientation;
	 sensor_orientation_t GyroOrientation;
	 sensor_int_vect AccRaw;
	 sensor_vect	AccNormalized;
	 sensor_vect AccSmooth;
	 sensor_vect AccRadians;
	 sensor_vect AccDegrees;
	 sensor_int_vect AccOffset;
	 sensor_int_vect GyroRaw;
	 sensor_int32_vect GyroOffset;
	 sensor_vect * GyroNormalized;
	 sensor_vect GyroNormalizedLastSample;
	 sensor_vect GyroDegrees;
	 sensor_vect * AxisDegrees;
	 bool GyroZero;
	 bool AccCalibrate ;
	void init();
	void setClock(byte v);
	void setAccRange(byte v);
	void setGyroRange(byte v);
	void setSleepEnabled(bool v);
	void setI2CBypassEnabled(bool state);
	void setI2CMasterModeEnabled(bool state);
	sensor_int_vect * getAcceleration();
	sensor_int_vect * getGyro();
	void setAccSmooth();
	void setAccCalibrate();
	void setGyroZero();
	void setGyroDegrees();
	void setAxisDegrees();
	void updateAccAndGyro();
};

 

#endif

