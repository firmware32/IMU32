// 
// 
// 


#include "Arduino.h"
#include <Wire.h>
#include "HMC5883L32.h"
#include "Sensor32.h"
#include "SPI2C.h"
#include "I2C32.h"
SPI2C_config mpuHMC5883LConfig = { SPI2C_type_e::SPI2C_I2C,HMC5883L_ADDRESS,SPI2C_I2C_CLOCK_400, SPI2C_NOT_USED,SPI2C_NOT_USED };


HMC5883L32::HMC5883L32(  sensor_vect * accNormalized)
{
	 
  MagAccNormalized = accNormalized;

}

void HMC5883L32::setConfig()
{
	setSPI2CConfig(&mpuHMC5883LConfig);
}

void HMC5883L32::init()
{
	setConfig();
	sensor_orientation_t defOrient = { { SENSOR_X ,  SENSOR_Y, SENSOR_Z },{ 1,1,1 } };
	 MagOrientation = defOrient;
	write(HMC5883L_RA_CONFIG_A,
		(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
		(HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
		(HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));
 

	// write CONFIG_B register
	setGain(HMC5883L_GAIN_1090);

	// write MODE register
	setMode(HMC5883L_MODE_SINGLE);
}
/** Set number of samples averaged per measurement.
* @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_AVERAGE_BIT
* @see HMC5883L_CRA_AVERAGE_LENGTH
*/
void HMC5883L32::setSampleAveraging(uint8_t averaging)
{
	writeBits(  HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** Set data output rate value.
* @param rate Rate of data output to registers
* @see getDataRate()
* @see HMC5883L_RATE_15
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_RATE_BIT
* @see HMC5883L_CRA_RATE_LENGTH
*/
void HMC5883L32::setDataRate(uint8_t rate)
{
	writeBits(  HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}
void HMC5883L32::setMeasurementBias(uint8_t bias)
{
	 writeBits( HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}
 

void HMC5883L32::setGain(uint8_t gain)
{
	/** Get magnetic field gain value.
	* The table below shows nominal gain settings. Use the "Gain" column to convert
	* counts to Gauss. Choose a lower gain value (higher GN#) when total field
	* strength causes overflow in one of the data output registers (saturation).
	* The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
	*
	* Value | Field Range | Gain (LSB/Gauss)
	* ------+-------------+-----------------
	* 0     | +/- 0.88 Ga | 1370
	* 1     | +/- 1.3 Ga  | 1090 (Default)
	* 2     | +/- 1.9 Ga  | 820
	* 3     | +/- 2.5 Ga  | 660
	* 4     | +/- 4.0 Ga  | 440
	* 5     | +/- 4.7 Ga  | 390
	* 6     | +/- 5.6 Ga  | 330
	* 7     | +/- 8.1 Ga  | 230
	*
	* @return Current magnetic field gain value
	* @see HMC5883L_GAIN_1090
	* @see HMC5883L_RA_CONFIG_B
	* @see HMC5883L_CRB_GAIN_BIT
	* @see HMC5883L_CRB_GAIN_LENGTH
	*/
 
	write(  HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
	 
}

void HMC5883L32::setMode(uint8_t newMode)
{
	write(  HMC5883L_RA_MODE, newMode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mode = newMode; // track to tell if we have to clear bit 7 after a read
}

void HMC5883L32::updateMag()
{

	setConfig();
 

	sensor_int_vect * v =  &MagRaw;
	read(HMC5883L_RA_DATAX_H, buffer, 6);
	if (mode == HMC5883L_MODE_SINGLE) write(HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));

	v->data[0] = ((((int16_t)buffer[0]) << 8) | buffer[1])  ;
	v->data[2] = ((((int16_t)buffer[2]) << 8) | buffer[3])  ;
	v->data[1] = ((((int16_t)buffer[4]) << 8) | buffer[5])  ;
	 

	float fax = MagAccNormalized->x;
	float fay = MagAccNormalized->y;
	float faz = MagAccNormalized->z;

	float froll = atan(-fax / faz) * RAD_TO_DEG;
	float fpitch = atan((fay / sqrtf((fax * fax) + (faz * faz)))) * RAD_TO_DEG;
	float fmx =  v->x / 0.92;
	float fmy =  v->y / 0.92;
	float fmz = v->z / 0.92;

	float ca = (sqrt(1.0f - (fax * fax) - (fay * fay)));
	float cx = (fmx  * (1 - (fax * fax))) - (fmy * fax *  fay) - (fmz * fax * (ca));
	float cy = (fmy *  (ca)) - (fmz * fay);

	//float heading = atan2(my, mx);
	//if (heading < 0)
	//	heading += 2 * M_PI;
	//SerialUSB.print("heading:\t");
	//SerialUSB.println(heading * 180 / M_PI);

	  MagHeading  = atan2(fmy, fmx);
	if (MagHeading  < 0)
		MagHeading += 2 * M_PI;
 	MagHeading *   180 / M_PI;
	MagHeadingCompensated = atan2(cy, cx);
	 if (MagHeadingCompensated  < 0)
	 	MagHeadingCompensated += 2 * M_PI;
	  MagHeadingCompensated *   180 / M_PI;
 
}
