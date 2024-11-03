#ifndef IMUSENSOR_H
#define IMUSENSOR_H

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

class IMUSensor {
public:
    IMUSensor();
    void Initialize();
    void ReadData();
    float GetYaw();
    float GetPitch();
    float GetRoll();

private:
    Adafruit_BNO055 bno;
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    float x, y, z;
    float yaw, pitch, roll;
    float Xacc, Yacc, Zacc;
    float Xrot, Yrot, Zrot;
    float boardTemp;
};

#endif
