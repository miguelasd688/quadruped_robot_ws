#include "IMUSensor.h"

IMUSensor::IMUSensor() 
{
  bno = Adafruit_BNO055(55, 0x28);
  
  x = 1000000;
  y = 1000000;
  z = 1000000;
  yaw = 10000;
  pitch = 10000;
  roll = 10000;
  Xacc = 10000;
  Yacc = 10000;
  Zacc = 10000;
  Xrot = 10000;
  Yrot = 10000;
  Zrot = 10000;
  boardTemp = 10000;
}

void IMUSensor::Initialize() {
    if (!bno.begin()) {
      yaw = 333.0;
      roll = 333.0;
      pitch = 333.0;
    }
    bno.setExtCrystalUse(true);
}

void IMUSensor::ReadData() {
  uint8_t sys, gyr, accel, mg = 0;
  bno.getCalibration(&sys, &gyr, &accel, &mg);
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
}

float IMUSensor::GetYaw() {
  return orientationData.orientation.x;; 
}
float IMUSensor::GetPitch() {
   return orientationData.orientation.y;
}
float IMUSensor::GetRoll() {
  return orientationData.orientation.z;
}









