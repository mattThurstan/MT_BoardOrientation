/*
 * MT_BoardOrientation_simple.ino - Simple example for Orientation setup library for long/short/skate boards. Used in longboardLight1 project.
 * Currently using MPU6050 6-axis motion sensor.
 * MTS Standish (mattThurstan), 2018.
 * Copyleft.
 */

#include <EEPROM.h>                               //ESP8266 style EEPROM (using 512 bytes) - WeMos D1 (R2 &) mini, 80 MHz, 115200 baud, 4M, (3M SPIFFS)
#include <FastLED.h>                              //WS2812B LED strip control and effects
#include <MT_BoardOrientation.h>                  //attempt to move all board mpu6050 sensor related items to a single library
 
#define SERIAL_SPEED 115200                       //Serial coms baud speed
boolean DEBUG = true;                             //realtime serial debugging output - general

MT_BoardOrientation o;                //declaration - sets defaults

void setup() 
{
  o.Init(); //need to pass values if any available from memory
  //o.InitWithVars(a[3], b[3]);
  
  if (DEBUG) {
    Serial.print(F("MPU6050 Stored Accel Offset XYZ = "));
    Serial.print(o.GetMPU6050AccelOffsetX());
    Serial.print(F(", "));
    Serial.print(o.GetMPU6050AccelOffsetY());
    Serial.print(F(", "));
    Serial.print(o.GetMPU6050AccelOffsetZ());
    Serial.println();

    Serial.print(F("MPU6050 Stored Gyro Offset XYZ = "));
    Serial.print(o.GetMPU6050GyroOffsetX());
    Serial.print(F(", "));
    Serial.print(o.GetMPU6050GyroOffsetY());
    Serial.print(F(", "));
    Serial.print(o.GetMPU6050GyroOffsetZ());
    Serial.println();
  }
}

const unsigned long _mpu6050ReadInterval = 20;    //read loop interval in milliseconds
const unsigned long _orientationInterval = 100;   //main orientation read loop interval in milliseconds

void loop() 
{
  EVERY_N_MILLISECONDS(_mpu6050ReadInterval) { o.ReadFiltered(); }    //FastLED non-blocking timed loop
  EVERY_N_MILLISECONDS(_orientationInterval) { 
    o.ReadOrientation(); 
    //if (o.GetOrientation() == 0 &&_mainLightsSubMode == 3) { o.ReadDirection(); }   /* main lights sub-mode 3 will always be kept 3 cos of this void */
    if (DEBUG) { 
      Serial.print(F("Orientation = "));
      Serial.print(o.GetOrientation());
      Serial.println();
    }
  }
}