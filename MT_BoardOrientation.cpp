/*
 * MT_BoardOrientation.cpp - Orientation setup library for long/short/skate boards. Used in longboardLight1 project.
 * MTS Standish (mattThurstan), 2026.
 * Copyleft.
 */
 
/*
 * Code contained within this file 'MT_BoardOrientation.cpp' was initially built with reference code from the following sources:
 * 
 * 'MPU-6050 Accelerometer + Gyro' by "Krodal" (arduino.cc user ) - June 2012 - Open Source / Public Domain.
 * 
 * https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_raw/MPU6050_raw.ino
 * 
 * http://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/ 
 * - Done by Luis Ródenas <luisrodenaslorda@gmail.com>
 * - Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
 * 
 * https://vvvv.org/documentation/arduino02
 * 
============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

 /*
  * MPU6050: 			X=Right/Left, Y=Forward/Backward, Z=Up/Down
  * orientation (byte):	0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
  * direction (byte):	0=forward, 1=back, 2=up, 3=down, 4=left, 5=right, 6=stationary
  */
    
 /*
  * Rolling average taken from example at http://playground.arduino.cc/Main/RunningAverage
  */

//#include "Arduino.h"
#include "MT_BoardOrientation.h"

//#define DEBUG	//comment/un-comment
#ifdef DEBUG
  #define DEBUG_PRINT_HEADER(x); Serial.print(F("MT_BoardOrientation - "))
  #define DEBUG_PRINT(x);    Serial.print(x)
  #define DEBUG_PRINTF(x);    Serial.print(F(x))
  #define DEBUG_PRINTLN(x);  Serial.println(x)
  #define DEBUG_PRINTLNF(x); Serial.println(F(x))
#else
  #define DEBUG_PRINT_HEADER(x)
  #define DEBUG_PRINT(x)     //blank line
  #define DEBUG_PRINTF(x)    //blank line
  #define DEBUG_PRINTLN(x)   //blank line
  #define DEBUG_PRINTLNF(x)  //blank line
#endif

/*
 * Declare at start of Arduino project.
 * eg. MT_BoardOrientation _orient;
 */
MT_BoardOrientation::MT_BoardOrientation() {
	_diSize = 10;
	
	/*----------------------------direction------------------------------*/
	_directionCur = 0;                 			// 0=forward, 1=back, 2=up, 3=down, 4=left, 5=right, 6=stationary

	/*----------------------------orientation----------------------------*/
	_orientation = 0;        					//0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
	orMatrix[0] = 0;             	    		//TEMP x =  0(low) / 1(mid) / 2(hi) - wanted to use -1, 0, 1 but too convoluted -- XYZ timed
	orMatrix[1] = 0;
	orMatrix[2] = 0;
	_orOrientationSave = 0;                		//used to hold the orientation during comparison
	_orOrientationTemp = 0;                  	//used to hold the orientation (then convert to _orientation)
	orFlag = false;                           	//flag 0 x
	orCounter = 0;                      		//TEMP time
}

/*
 * Call from setup to initialise.
 */
void MT_BoardOrientation::Init() {
	diInit();
}

void MT_BoardOrientation::InitWithVars(uint8_t accelYAverageSize) { 
	_diSize = accelYAverageSize
	diInit();
}

/*
 * Initialise direction/acceleration Y rolling average array.
 */
void MT_BoardOrientation::diInit() {
    _diAvr = (float*) malloc(_diSize * sizeof(float));
    if (_diAvr == NULL) _diSize = 0;
    diClear();
}

/*
* Clear direction rolling average array.
*/
void MT_BoardOrientation::diClear() {
	_diCnt = 0;
    _diIdx = 0;
    _diSum = 0.0;
    for (int i = 0; i< _diSize; i++) _diAvr[i] = 0.0;  // needed to keep addValue simple
}

void MT_BoardOrientation::AddToAverageAccelY(float accelY) {
	diAddValue(accelY);
}

/*
* Add value to direction rolling average array.
*/
void MT_BoardOrientation::diAddValue(float f) {
	if (_diAvr == NULL) return;
    _diSum -= _diAvr[_diIdx];
    _diAvr[_diIdx] = f;
    _diSum += _diAvr[_diIdx];
    _diIdx++;
    if (_diIdx == _diSize) _diIdx = 0;  // faster than %
    if (_diCnt < _diSize) _diCnt++;
}

/*
* Get average (result) from direction rolling average array.
*/
float MT_BoardOrientation::diGetAverage() {
	if (_diCnt == 0) return NAN;
    return _diSum / _diCnt;
}

/*
* Called from a timed loop within the main loop.
*/
byte MT_BoardOrientation::GetDirection() {
	//only call if tracking sub-mode is actually running, otherwise waste of processing..
    if (diGetAverage() > _diThreshold) {	//100
	  _directionCur = 0;  //going forwards
    } else if (diGetAverage() < -_diThreshold) {
	  _directionCur = 1;  //going backwards
	} else {
	  _directionCur = 6;  //stationary
	}
	diClear();
	
	DEBUG_PRINT_HEADER();
	DEBUG_PRINTF("_directionCur ");
	DEBUG_PRINT(_directionCur);
	DEBUG_PRINTLNF(".");
	
	return _directionCur;
}

/*
* Called from a timed loop within the main loop.
*/
byte MT_BoardOrientation::GetOrientation(float x, float y, float z) {
	float xyz[3]; //this is a smooth gyro and accel combined to give angle
	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;
    float cutoff = 45;  //starting at zero calibration, we need to know 90deg either way, so 45 is halfway point anywhere from 0
    
	for (int i = 0; i < 3; i++) {
      if ( xyz[i] < -cutoff ) { orMatrix[i] = 0; }
      else if ( xyz[i] < cutoff && xyz[i] > -cutoff ) { orMatrix[i] = 1; }
      else if ( xyz[i] > cutoff ) { orMatrix[i] = 2; }
    }

    //compare 3-matrix, set orientation temp
    if (orMatrix[0] == 1 && orMatrix[1] == 1 ) {
            _orOrientationTemp = 0;
            if (orFlag == false) { _orOrientationSave = 0; orCounter = millis(); orFlag = true; }
          }  //1, 1, 1 - flat
//upside-down - have to work on this one..
//    else if (orMatrix[0] == 1
//          && (orMatrix[1] == 0 || orMatrix[1] == 2)
//          && orMatrix[2] == 0) {
//            _orOrientationTemp = 1;
//            if (orFlag == false) { _orOrientationSave = 1; orCounter = millis(); orFlag = true; }
//          }  //1, 0 or 2, 1 - upside-down - have to work on this one..
    else if (orMatrix[0] == 2 && orMatrix[1] == 1 ) { 
            _orOrientationTemp = 2;
            if (orFlag == false) { _orOrientationSave = 2; orCounter = millis(); orFlag = true; } 
          }  //2, 1, 1 - up
    else if (orMatrix[0] == 0 && orMatrix[1] == 1 ) { 
            _orOrientationTemp = 3;
            if (orFlag == false) { _orOrientationSave = 3; orCounter = millis(); orFlag = true; }
          }  //0, 1, 1 - down
    else if (orMatrix[0] == 1 && orMatrix[1] == 0 ) { 
            _orOrientationTemp = 4;
            if (orFlag == false) { _orOrientationSave = 4; orCounter = millis(); orFlag = true; }
          }  //1, 0, 1 - left  
    else if (orMatrix[0] == 1 && orMatrix[1] == 2 ) { 
            _orOrientationTemp = 5;
            if (orFlag == false) { _orOrientationSave = 5; orCounter = millis(); orFlag = true; }
          }  //1, 2, 1 - right

    unsigned long orGetMillis = millis();
    if (orFlag == true) {
      if ( (unsigned long) (orGetMillis - orCounter) >= orInterval) {
        if (_orOrientationSave == _orOrientationTemp) {
          //is the orientation still the same as when we took a sample and set the timer?
          //if so, set the actual orientation
		  //this is all probably overkill
          _orientation = _orOrientationTemp;
        }
        orFlag = false; //either way, reset
      }
    }

	DEBUG_PRINT_HEADER();
	DEBUG_PRINTF("_orientation ");
	DEBUG_PRINT(_orientation);
	DEBUG_PRINTLNF(".");
	
	return _orientation;
}
