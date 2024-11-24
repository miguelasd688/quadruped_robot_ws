#ifndef ACTUATORSENCODERS_H
#define ACTUATORSENCODERS_H

#include <ResponsiveAnalogRead.h>
#include <SimpleKalmanFilter.h>
#include "utils.h"

class ActuatorsEncoders {
private:
  uint32_t encoder_pin[12] = {15, 16, 17,//FR
                                39, 38, 26,//FL
                                20, 21, 22,//BR
                                14, 41, 40//BL
                                };
  ResponsiveAnalogRead analog1;
  ResponsiveAnalogRead analog2;
  ResponsiveAnalogRead analog3;
  ResponsiveAnalogRead analog4;
  ResponsiveAnalogRead analog5;
  ResponsiveAnalogRead analog6;
  ResponsiveAnalogRead analog7;
  ResponsiveAnalogRead analog8;
  ResponsiveAnalogRead analog9;
  ResponsiveAnalogRead analog10;
  ResponsiveAnalogRead analog11;
  ResponsiveAnalogRead analog12;

  uint32_t encoder_data[12] = {500000, 500000, 500000,
                               500000, 500000, 500000,
                               500000, 500000, 500000,
                               500000, 500000, 500000
                              };
  //Regression data for encoders
  //               coxa      femur    tibia
  float ae[12] = {0.20292, 0.20317, 0.19904 ,
                  0.21256, -0.22492, -0.21321,
                  -0.21047, -0.20355, -0.20095,
                  -0.20265, 0.19904, 0.20337
                 };
  
  float be[12] = {-18.59717, -5.70512, -2.51697,
                  -5.75856, 197.29411, 202.72169,
                  185.96931, 204.11902, 199.38663,
                  197.89534, -5.33768, -32.23424
                 };

  struct LegsAngle anglesEncoders;

  float eang1 , eang2 , eang3 , eang4 , eang5 , eang6 , eang7 , eang8 , eang9 , eang10 , eang11 , eang12;
  float enc1 , enc2 , enc3 , enc4 , enc5 , enc6 , enc7 , enc8 , enc9 , enc10 , enc11 , enc12;


  //  SimpleKalmanFilter(e_mea, e_est, q);
  //  e_mea: Measurement Uncertainty
  //  e_est: Estimation Uncertainty
  //  q: Process Noise

  float e_mea = 100;
  float q = 1;
  SimpleKalmanFilter KFilter1;
  SimpleKalmanFilter KFilter2;
  SimpleKalmanFilter KFilter3;
  SimpleKalmanFilter KFilter4;
  SimpleKalmanFilter KFilter5;
  SimpleKalmanFilter KFilter6;
  SimpleKalmanFilter KFilter7;
  SimpleKalmanFilter KFilter8;
  SimpleKalmanFilter KFilter9;
  SimpleKalmanFilter KFilter10;
  SimpleKalmanFilter KFilter11;
  SimpleKalmanFilter KFilter12;
public:
  ActuatorsEncoders();
  void ReadRawEncoders();
  void ReadEncoders();  
  void ReadEncoderAngles();
  float GetEncoderRawValue(int i);
};

#endif