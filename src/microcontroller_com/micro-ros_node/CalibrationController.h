#ifndef CALIBRATION_CONTROLLER_H
#define CALIBRATION_CONTROLLER_H

#include "utils.h"

class CalibrationController {
private:

  float desiredAngles1[12] = {183.0, 197.0, 165.5, //FR
                             193.0,   5.5,  -2.5,   //FL
                             5.0, -20.0, -11.0,     //BR
                             -13.5, 191.5, 182.0 //BL
                            };
  float desiredAngles2[12] = {183.0, 197.0, 165.5, //FR
                             193.0,   5.5,  -2.5,   //FL
                             5.0, -20.0, -11.0,     //BR
                             -13.5, 191.5, 182.0 //BL
                            };
  float desiredAngles3[12] = {183.0, 197.0, 165.5, //FR
                             193.0,   5.5,  -2.5,   //FL
                             5.0, -20.0, -11.0,     //BR
                             -13.5, 191.5, 182.0 //BL
                            };
                            
  int lastAction = 0;
  float xi;
  float yi;
  float xf;
  float yf;

public:  
  bool CheckStatus(int calAction, int calLeg, LegsAngle targetAngles);
  void SetInitialPoint(float x, float y);
  void SetFinalPoint(float x, float y);
  float ComputeInterpolationA();
  float ComputeInterpolationB();
};

#endif