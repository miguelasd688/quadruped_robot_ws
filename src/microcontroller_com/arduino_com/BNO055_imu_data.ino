#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem



void IMUSetup(void){
//  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    
  }

  delay(1000);
}

void readIMU(void){
    uint8_t sys, gyr, accel, mg = 0;
    bno.getCalibration(&sys, &gyr, &accel, &mg);
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    //bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    
//    Serial.print("Accl:");
//    Xacc = accelerometerData.acceleration.x;
//    Yacc = accelerometerData.acceleration.y;
//    Zacc = accelerometerData.acceleration.z;

//    Serial.print("angVel:");
    Xrot = angVelocityData.gyro.x;
    Yrot = angVelocityData.gyro.y;
    Zrot = angVelocityData.gyro.z;
    
//    Serial.print("Orient:");
    yaw = orientationData.orientation.x;
    roll = orientationData.orientation.y;
    pitch = orientationData.orientation.z;

    
//    Serial.print("Mag:");
    x = magnetometerData.magnetic.x;
    y = magnetometerData.magnetic.y;
    z = magnetometerData.magnetic.z;
    
//    Serial.print("Gyro:");
    x = angVelocityData.gyro.x;
    y = angVelocityData.gyro.y;
    z = angVelocityData.gyro.z;
    
//    Serial.print("Rot:");
//    x = event->gyro.x;
//    y = event->gyro.y;
//    z = event->gyro.z;
    
//    Serial.print("Linear:");
    Xacc = linearAccelData.acceleration.x;
    Yacc = linearAccelData.acceleration.y;
    Zacc = linearAccelData.acceleration.z;
    
//    Serial.print("Gravity:");
    x = gravityData.acceleration.x;
    y = gravityData.acceleration.y;
    z = gravityData.acceleration.z;

    boardTemp = bno.getTemp();
    
//    Serial.print("  ");
//    Serial.print(F("temperature: "));
//    Serial.print(boardTemp);
//    Serial.println("  ");

//    uint8_t system, gyro, accel, mag = 0;
//    bno.getCalibration(&system, &gyro, &accel, &mag);
//    Serial.println();
//    Serial.print("Calibration: Sys=");
//    Serial.print(system);
//    Serial.print(" Gyro=");
//    Serial.print(gyro);
//    Serial.print(" Accel=");
//    Serial.print(accel);
//    Serial.print(" Mag=");
//    Serial.println(mag);
//
//    Serial.println("--");
}
