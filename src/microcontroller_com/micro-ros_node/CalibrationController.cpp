#include "CalibrationController.h"

bool CalibrationController::CheckStatus(int calAction, int calLeg, LegsAngle targetAngles)
{

  if (calAction != this->lastAction)
  {
    switch (calAction)
    {
    case 0: // Nominal action, no calibration active
      /* code */
      break;
    case 1: // Set first reference point, calibration started
      for (int i = 0; i < 3; i++)
      {
        Debugger::Log("Calibration set point 1");
        SetInitialPoint(desiredAngles1[i + 3 * calLeg], targetAngles.asArray[i + 3 * calLeg]);
      }
      break;
    case 2: // Set second reference point, calibration in progress
      for (int i = 0; i < 3; i++)
      {
        Debugger::Log("Calibration set point 2");
        SetFinalPoint(desiredAngles2[i + 3 * calLeg], targetAngles.asArray[i + 3 * calLeg]);
      }
      break; 
    default: // Calibration terminated, return true to compute calibration
      Debugger::Log("Calibration terminate"); 
      this->lastAction = calAction;
      return true;
    }
    this->lastAction = calAction;
    return false;
  }
  return false;
}

float CalibrationController::ComputeInterpolationA()
{
  return (yf - yi) / (xf - xi);
}

float CalibrationController::ComputeInterpolationB()
{
  return yi - (yf - yi) * xi / (xf - xi);
}

void CalibrationController::SetInitialPoint(float x, float y)
{
  this->yi = x;
  this->xi = y;
}

void CalibrationController::SetFinalPoint(float x, float y)
{
  this->yf = x;
  this->xf = y;
}