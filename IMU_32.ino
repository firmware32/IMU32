

#include <SPI.h>
#include <Wire.h>
#include <SPI2C.h>
#include <I2C32.h>
#include "Sensor32.h"
#include "MPU605032.h"
#include "HMC5883L32.h"
#include "MW_Protocol32.h"
#define SER SerialUSB
sensor_vect acc;
sensor_vect gyro;
float mag_normalized;
float mag_compensated;
MPU605032 mpu(&gyro, &acc);
HMC5883L32 mag(  &mpu.AccNormalized);
void   serialCom();
void setup()
{
	
	Wire.begin();
	//SER.begin(115200);
	//while (!SER);
	mwp.init();
	/*while (1)
		SER.println("hi");*/
	mpu.init();
	mag.init();
	 
  /* add setup code here */
	
}
uint32_t _del = 0;
byte once = 0;
void loop()
{
	uint32_t tim = micros();
	mpu.updateAccAndGyro();
	mag.updateMag();
	uint32_t tim2 = micros() - tim;
	sensor_int_vect * acc = &mpu.AccRaw;
	sensor_int_vect * gyro = &mpu.GyroRaw;


	//SER.print("acc_raw"); SER.print(" ");
	//SER.print( acc->x); SER.print(" ");
	//SER.print(acc->y); SER.print(" ");
	//SER.print(acc->z); SER.print(" ");SER.print(" ");

	sensor_vect * vectf = &mpu.AccNormalized;
	//SER.print("acc_norm"); SER.print(" ");
	if(millis() >_del)
	{ 
		if (once <= 5)
			once++;

		if(once==5)
			mpu.GyroZero = mpu.AccCalibrate = true;
	//SER.print(atan2(vectf->x , vectf->z)* RAD_TO_DEG); SER.print(" ,");
	//SER.print(atan2(vectf->y, vectf->z)* RAD_TO_DEG); SER.print(", ");
	/////SER.print(vectf->z); SER.print(" ");SER.print(" ");
		SER.print("time"); SER.print(" ");
		SER.print(tim2); SER.print(" ");
	//vectf = &mpu.AccSmooth;
	////SER.print("acc_smooth"); SER.print(" ");
	//SER.print(atan2(vectf->x, vectf->z)* RAD_TO_DEG); SER.print(" ,");
	//SER.print(atan2(vectf->y, vectf->z) * RAD_TO_DEG);// SER.print(", ");
	/////SER.print(vectf->z); SER.print(" ");
	//SER.println();
	sensor_int_vect * gyro = &mag.MagRaw;
	SER.print("MagRaw"); SER.print(" ");
	SER.print(gyro->x); SER.print(" ");
	SER.print(gyro->y); SER.print(" ");
	SER.print(gyro->z); SER.print(" ");
	sensor_int_vect * gyro2 = &mpu.AccRaw;
	SER.print("accraw"); SER.print(" ");
	SER.print(gyro2->x); SER.print(" ");
	SER.print(gyro2->y); SER.print(" ");
	SER.print(gyro2->z); SER.print(" ");

	vectf =  mpu.GyroNormalized;
	 SER.print("gyro_norm"); SER.print(" ");
	//SER.print(atan2(vectf->x, vectf->z)); SER.print(" ,");
	//SER.print(atan2(vectf->y, vectf->z) );// SER.print(", ");
	SER.print(vectf->x); SER.print(" ");
  SER.print(vectf->y); SER.print(" ");
  SER.print(vectf->z); SER.print(" ");

  vectf = &mpu.AccDegrees;
  SER.print("gyro_deg"); SER.print(" ");
  //SER.print(atan2(vectf->x, vectf->z)); SER.print(" ,");
  //SER.print(atan2(vectf->y, vectf->z) );// SER.print(", ");
  SER.print(vectf->x); SER.print(" ");
  SER.print(vectf->y); SER.print(" ");
  SER.print(vectf->z); SER.print(" ");

  vectf =  mpu.AxisDegrees;
  SER.print("axis"); SER.print(" ");
  //SER.print(atan2(vectf->x, vectf->z)); SER.print(" ,");
  //SER.print(atan2(vectf->y, vectf->z) );// SER.print(", ");
  SER.print(vectf->x); SER.print(" ");
  SER.print(vectf->y); SER.print(" ");
  SER.print(vectf->z); SER.print(" ");
  SER.print(mag.MagHeading * 180 / 3.14159265358979323846); SER.print(" ");
  SER.print(mag.MagHeadingCompensated * 180 / M_PI); SER.print(" ");
	SER.println();
	_del = millis() + 124;
	}
	serialCom();
//	delay(50);
	
  /* add main program code here */

}
