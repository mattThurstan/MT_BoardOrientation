/*
 * MT_BoardOrientation.h - Orientation setup library for long/short/skate boards. Used in longboardLight1 project.
 * Calculates direction, orientation and indicators.
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

#include "Arduino.h"

/*----------------------------main header declerations----------------------------*/
class MT_BoardOrientation
{
  protected:
    uint8_t _diSize = 10;							//rolling average of acceleration Y values
    uint8_t _diCnt;
    uint8_t _diIdx;
    float   _diSum;
    float * _diAvr;
	
    uint8_t _indSize = 10;							//rolling average of Roll (Y) values
    uint8_t _indCnt;
    uint8_t _indIdx;
    float   _indSum;
    float * _indAvr;
	
  private:
	/*----------------------------direction------------------------------*/
	void diInit();
	void diClear();
    void diAddValue(float);
	float diGetAverage();
	float _diThreshold;				//threshold tolerance for 'dead zone' at center of readings
	
	byte _directionCur;                				// 0=forward, 1=back, 2=up, 3=down, 4=left, 5=right, 6=stationary

	/*----------------------------orientation----------------------------*/
	byte _orientationCur;                  			//0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
	byte orMatrix[3];           					//TEMP x =  0(low) / 1(mid) / 2(hi)       - wanted to use -1, 0, 1 but too convoluted    -- XYZ timed
	byte _orOrientationSave;             			//used to hold the orientation during comparison
	byte _orOrientationTemp;           				//used to hold the orientation (then convert to _orientationCur)
	boolean orFlag;                     			//flag 0 x
	unsigned long orCounter;            			//TEMP time
	const unsigned long orInterval = 250;//450;			//interval at which to check whether flags have changed - are we still in the same orientation - how long to trigger
	
	/*----------------------------indicators-----------------------------*/
	void indInit();
	void indClear();
    void indAddValue(float);
	float indGetAverage();
	float _indThreshold;					//pitch deadzone. when to turn on left/right leaning/turning indicators.
	
	byte _indicatorCur;								//0=null, 1=Left, 2=Right
	
  public:
	MT_BoardOrientation();
	void InitWithVars(uint8_t accelYAverageSize, float accelThreshold, uint8_t indYAverageSize, float indThreshold);	//Y acceleration rolling average loop size, Roll (Y)
	
	void AddToAverage(float accelY, float pitch);	//call this something like '_diSize' times per second (currently 10). this would give a rolling average of 10 per sec.
	void DoUpdateDirection();
	void DoUpdateOrientation(float x, float y, float z);
	void DoUpdateIndicator();
	
	byte GetDirection() { return _directionCur; }
	byte GetOrientation() { return _orientationCur; }
	byte GetIndicator() { return _indicatorCur; }	//roll. used with deadzone for Left/Right orange indicators. 0=null, 1=Left, 2=Right
};

#endif
