#ifndef ACTUATORS_H
#define ACTUATORS_H

#define MAX_PULSE 2500
#define MIN_PULSE 560

#include "InverseKinematicsSolver.h"

class Actuators {
private:
  InverseKinematicsSolver IK;

  //---------------SERVO PIN DEFINITION------------------------
  uint32_t freq = 125; // PWM frecuency can be choosen here.
  
  uint32_t actuators_pin[12] = { 6, 5, 4,//FR
                                 28, 29, 36,//FL
                                 3, 2, 1,//BR
                                 7, 24, 25//BL
                               };
  
  //----------------- CALIBRATION PARAMETERS OF EACH SERVO -----------------
  //Regression data for motor
  //                 coxa      femur     tibia
  float a[12] = { -1.08333, -1.06667, -1.07778, //FR
                   -1.03333,  0.97778,  1.01111, //FL
                   1.03333,  1.05556,  1.07778,   //BR
                   1.07500, -1.07778, -1.00000 //BL
                 };
  
  float b[12] = {183.0, 197.0, 165.5, //FR
                193.0,   5.5,  -2.5,   //FL
                  5.0, -20.0, -11.0,     //BR
                  -13.5, 191.5, 182.0 //BL
                 };
  //Regression data for encoders
  //               coxa      femur    tibia
  float ae[12] = {0.20292, 0.20317, 0.19904 ,
                   0.21256, -0.22492, -0.21321,
                   -0.21047, -0.20355, -0.20095,
                   -0.20265, 0.19904, 0.20337
                  };
  
  float be[12] = { -18.59717, -5.70512, -2.51697,
                    -5.75856, 197.29411, 202.72169,
                    185.96931, 204.11902, 199.38663,
                    197.89534, -5.33768, -32.23424
                  };
                  
  float lowLim[12] = {50, 30, 30, 50, 30, 30, 50, 30, 30, 50, 30, 30};
  float highLim[12] = {130, 150, 150, 130, 150, 150, 130, 150, 150, 130, 150, 150};
  
  struct LegsAngle anglesServo;
  float fineAngle;
  
  
public:  
  void ConnectServos();
  void SetNewServo(uint32_t pin);
  void ServoWrite(uint32_t pin , float angle);
  float CheckLimits(float angle , float lowLim , float highLim);
  void MoveServos();
  bool StepMotors(bool RUN, bool SAFE, LegsAngle targetAngles);
};

#endif