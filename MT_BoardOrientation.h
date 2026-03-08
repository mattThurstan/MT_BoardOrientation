/*
 * MT_BoardOrientation.h - Orientation setup library for long/short/skate boards. Used in longboardLight1 project.
 * Calculates direction and orientation.
 * MTS Standish (mattThurstan), 2026.
 * Copyleft.
 */
 
 /*
  * IMU (expected): 			X=Right/Left, Y=Forward/Backward, Z=Up/Down
  * orientation (byte):	0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
  * direction (byte):	-1=stationary, 0=forward, 1=back, 2=up, 3=down, 4=left, 5=right
  */
  
 /*
  * Rolling average taken from example at http://playground.arduino.cc/Main/RunningAverage
  */

#ifndef __MT_BOARDORIENTATION_H__
#define __MT_BOARDORIENTATION_H__

//#include "Arduino.h"

/*----------------------------main header declerations----------------------------*/
class MT_BoardOrientation
{
  protected:
    uint8_t _diSize;// = 10;							//rolling average of acceleration Y values
    uint8_t _diCnt;
    uint8_t _diIdx;
    float   _diSum;
    float * _diAvr;
	
  private:
	void diInit();
	void diClear();
    void diAddValue(float);
	float diGetAverage();
	const float _diThreshold = 100.00;				//threshold tolerance for 'dead zone' at center of readings
	
	/*----------------------------direction------------------------------*/
	byte _directionCur;                				// 0=forward, 1=back, 2=up, 3=down, 4=left, 5=right, 6=stationary

	/*----------------------------orientation----------------------------*/
	byte _orientation;                  			//0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
	byte orMatrix[3];           					//TEMP x =  0(low) / 1(mid) / 2(hi)       - wanted to use -1, 0, 1 but too convoluted    -- XYZ timed
	byte _orOrientationSave;             			//used to hold the orientation during comparison
	byte _orOrientationTemp;           				//used to hold the orientation (then convert to _orientation)
	boolean orFlag;                     			//flag 0 x
	unsigned long orCounter;            			//TEMP time
	const unsigned long orInterval = 450;			//interval at which to check whether flags have changed - are we still in the same orientation - how long to trigger
	
  public:
	MT_BoardOrientation();
	void Init();
	void InitWithVars(uint8_t accelYAverageSize);	//Y acceleration rolling average loop size
	
	void AddToAverageAccelY(float accelY);	//call this something like '_diSize' times per second (currently 10). this would give a rolling average of 10 per sec.
	byte GetDirection();
	byte GetOrientation(float x, float y, float z);
};

#endif
