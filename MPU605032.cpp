 
#include "Arduino.h"
#include <Wire.h>
#include "MPU605032.h"
#include "Sensor32.h"
#include "SPI2C.h"
#include "I2C32.h"
SPI2C_config mpu6xxx32Config = { SPI2C_type_e::SPI2C_I2C,0x68,SPI2C_I2C_CLOCK_400, SPI2C_NOT_USED,SPI2C_NOT_USED };

void MPU605032::init()
{
	setConfig();
	sensor_orientation_t defOrient = { { SENSOR_X ,SENSOR_Y, SENSOR_Z,},{1,1,1} };
	AccOrientation = defOrient;
	GyroOrientation = defOrient;

	setSleepEnabled(true);
	delay(10);
	setSleepEnabled(false);
	delay(10);
	setClock(MPU6050_CLOCK_PLL_XGYRO);
	setGyroRange(MPU6050_GYRO_FS_2000);
	setAccRange(MPU6050_ACCEL_FS_2);
	setSleepEnabled(false);
	delay(10);
	setI2CBypassEnabled(true);
	setI2CMasterModeEnabled(false);
}
//#define MPU6050_CLOCK_PLL_XGYRO         0x01
void MPU605032::setClock(byte source)
{
	writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

//							MPU6050_RANGE_2G	MPU6050_RANGE_4G	MPU6050_RANGE_8G	MPU6050_RANGE_16G
uint32_t AccRangeG[4] = { 16384, 8192, 4096, 2048 };
float AccScale[4] = {		0.00006103515625 , 0.0001220703125f,	0.000244140625f,	0.00048828125f };
float * accScale = AccScale;
void MPU605032::setAccRange(byte range)
{
	AccScaleID = range;
	writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
inline void MPU605032::setAccNormalize(sensor_int_vect * a)
{
	AccNormalized.x = (float)(a->x - AccOffset.x) * AccScale[AccScaleID];
	AccNormalized.y= (float)(a->y - AccOffset.y) * AccScale[AccScaleID];
	AccNormalized.z = (float)(a->z - AccOffset.z) * AccScale[AccScaleID];
}
	 //					MPU6050_SCALE_250DPS	MPU6050_SCALE_500DPS	MPU6050_SCALE_1000DPS	MPU6050_SCALE_2000DPS:
float GyroScale[4] = { 0.0076219512195121951219512195122f, 0.01524390243902439024390243902439f,0.03048780487804878048780487804878f, 0.06097560975609756097560975609756f };
void MPU605032::setGyroRange(byte range)
{
	GyroScaleID = range;
	writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
inline void MPU605032::setGyroNormalize(sensor_int_vect * a)
{
	GyroNormalized->x = (float)(a->x - GyroOffset.x) * GyroScale[GyroScaleID];
	GyroNormalized->y = (float)(a->y - GyroOffset.y) * GyroScale[GyroScaleID];
	GyroNormalized->z = (float)(a->z - GyroOffset.z) * GyroScale[GyroScaleID];
}
MPU605032::MPU605032(sensor_vect * gyroNormalized, sensor_vect * axisDegrees)
{
	AxisDegrees  = axisDegrees;
	GyroNormalized = gyroNormalized;
}
void MPU605032::setConfig()
{
	setSPI2CConfig(&mpu6xxx32Config);
}
void MPU605032::setSleepEnabled(bool enabled)
{
	writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

sensor_int_vect * MPU605032::getGyro()
{
	sensor_int_vect * v = &GyroRaw;
	read(MPU6050_RA_GYRO_XOUT_H, buffer, 6);
	v->data[GyroOrientation.orientation[0]] = ((((int16_t)buffer[0]) << 8) | buffer[1])  *  GyroOrientation.direction[0];
	v->data[GyroOrientation.orientation[1]] = ((((int16_t)buffer[2]) << 8) | buffer[3]) * GyroOrientation.direction[1];
	v->data[GyroOrientation.orientation[2]] = ((((int16_t)buffer[4]) << 8) | buffer[5]) * GyroOrientation.direction[2];
	return v;
}
float accSmoothHistory[ACC_SMOOTH_HISTORY_COUNT][3];
uint8_t accSmoothHistoryID = 0;
 
inline void MPU605032::setAccSmooth()
{
	static float AccPer = (float)ACC_SMOOTH_HISTORY_COUNT /1000000.0;
	uint8_t i = 0;
	for (  i = 0; i <  3; i++)
	{
		AccSmooth.data[i] -= accSmoothHistory[accSmoothHistoryID][i];
		accSmoothHistory[accSmoothHistoryID][i] = (AccNormalized.data[i] * AccPer);
		AccSmooth.data[i] += (accSmoothHistory[accSmoothHistoryID][i]);
	}
	accSmoothHistoryID++;
	if (accSmoothHistoryID == ACC_SMOOTH_HISTORY_COUNT)
		accSmoothHistoryID = 0;
}
void MPU605032::setAccCalibrate()
{
	static int32_t AccTemp[3];
		static int16_t AccCalibateSamples;

		if (AccCalibrate)
		{
			AccCalibateSamples = 512;
			AccCalibrate = false;
		}
	if (AccCalibateSamples>0) {
		AccCalibateSamples--;
		for (uint8_t axis = 0; axis < 3; axis++) {
			if (AccCalibateSamples == 511) AccTemp[axis] = 0;   // Reset a[axis] at start of calibration
			AccTemp[axis] += AccRaw.data[axis];           // Sum up 512 readings
			AccOffset.data[axis]= AccTemp[axis]/512; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (AccCalibateSamples == 0) {

			AccOffset.data[2] -= AccRangeG[AccScaleID];
			//global_conf.accZero[YAW] -= ACC_1G;   // shift Z down by ACC_1G and store values in EEPROM at end of calibration
			//conf.angleTrim[ROLL] = 0;
			//conf.angleTrim[PITCH] = 0;
			 
		}
	}
}
static uint16_t gyroSamples = 0;
void MPU605032::setGyroZero()
{
	
	if (GyroZero){
		gyroSamples = 512;
		GyroZero = false;

	}
	if (gyroSamples == 0)
		return;
	else
	{
		for (int i = 0; i < 3; i++)
		{
			if (gyroSamples == 512)
				GyroOffset.data[i] = 0;
			GyroOffset.data[i]+= GyroRaw.data[i];
		}
		gyroSamples--;
		if (gyroSamples == 0)
		{
			GyroOffset.x = GyroOffset.x / 512;
			GyroOffset.y = GyroOffset.y / 512;
			GyroOffset.z = GyroOffset.z / 512;
			for (int i = 0; i < 3; i++)
			{ 
				GyroDegrees.data[i] = 0;
				AxisDegrees->data[i] = 0;

			}
		}
	}

}

inline void MPU605032::setGyroDegrees()
{
	static float lastGyro = 0;
	float currentGyro = micros();
	float per = (currentGyro  - lastGyro) / 1000000.0f;
	for (int i = 0; i < 3; i++)
	{
		GyroNormalizedLastSample.data[i] = (GyroNormalized->data[i] * per);
		GyroDegrees.data[i] += GyroNormalizedLastSample.data[i];

	}
	lastGyro = currentGyro;
}
inline void MPU605032::setAxisDegrees()
{
	if (gyroSamples)
		return;
	for (int i = 0; i < 3; i++)
	{
		if (i == 0)
			AccRadians.data[i] = atan2(AccNormalized.x, sqrt(AccNormalized.y * AccNormalized.y + AccNormalized.z * AccNormalized.z));  //atan((AccNormalized.y / sqrtf((AccNormalized.x * AccNormalized.x) + (AccNormalized.z * AccNormalized.z))));
		else if (i == 1)
			AccRadians.data[i] = atan2(-AccNormalized.y, AccNormalized.z);
		

		 
		if (i < 2)
		{ 
			AccDegrees.data[i] = AccRadians.data[i] * RAD_TO_DEG;
			AxisDegrees->data[i] += (GyroNormalizedLastSample.data[i]);
			AxisDegrees->data[i]= (AxisDegrees->data[i] * .95) + (AccDegrees.data[i] * .05);
		}
		else
			AxisDegrees->data[i] += (GyroNormalizedLastSample.data[i]);
	}
	/*float froll =  * RAD_TO_DEG;
	float fpitch = atan((fay / sqrtf((fax * fax) + (faz * faz)))) * RAD_TO_DEG;*/
}
void MPU605032::setI2CMasterModeEnabled(bool state)
{
	writeBit(MPU6050_REG_USER_CTRL, 5, state);
}

void MPU605032::setI2CBypassEnabled(bool state)
{
	  writeBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}
void MPU605032::updateAccAndGyro()
{
	setConfig();
	sensor_int_vect * v = &GyroRaw;
	read(MPU6050_RA_ACCEL_XOUT_H, buffer, 14);
	
	v->data[GyroOrientation.orientation[0]] = ((((int16_t)buffer[8]) << 8) | buffer[9])  *  GyroOrientation.direction[0];
	v->data[GyroOrientation.orientation[1]] = ((((int16_t)buffer[10]) << 8) | buffer[11]) * GyroOrientation.direction[1];
	v->data[GyroOrientation.orientation[2]] = ((((int16_t)buffer[12]) << 8) | buffer[13]) * GyroOrientation.direction[2];
	setGyroZero();
	setGyroNormalize(v);
	setGyroDegrees();
	setAxisDegrees();
	v = &AccRaw;
	v->data[AccOrientation.orientation[0]] = ((((int16_t)buffer[0]) << 8) | buffer[1])  *  AccOrientation.direction[0];
	v->data[AccOrientation.orientation[1]] = ((((int16_t)buffer[2]) << 8) | buffer[3]) * AccOrientation.direction[1];
	v->data[AccOrientation.orientation[2]] = ((((int16_t)buffer[4]) << 8) | buffer[5]) * AccOrientation.direction[2];
	setAccCalibrate();
	setAccNormalize(&AccRaw);
	setAccSmooth();
	

}

sensor_int_vect * MPU605032::getAcceleration()
{
	sensor_int_vect * v = &AccRaw;
	read(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
	v->data[AccOrientation.orientation[0]] = ((((int16_t)buffer[0]) << 8) | buffer[1])  *  AccOrientation.direction[0];
	v->data[AccOrientation.orientation[1]] = ((((int16_t)buffer[2]) << 8) | buffer[3]) * AccOrientation.direction[1];
	v->data[AccOrientation.orientation[2]] = ((((int16_t)buffer[4]) << 8) | buffer[5]) * AccOrientation.direction[2];
	return v;
}
