// MW_Protocol32.h

#ifndef _MW_PROTOCOL32_h
#define _MW_PROTOCOL32_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class MW_Protocol32 
{
	
 protected:
	 Serial_ * ps;

 public:
	 MW_Protocol32(Serial_ * s);
	void init();
};

extern MW_Protocol32  mwp;

#endif

