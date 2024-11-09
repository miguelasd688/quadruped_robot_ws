#include "InverseKinematicsSolver.h"


struct LegsAngle InverseKinematicsSolver::CalculateRobotAngles(struct LegsAngle targetAngles)
{
  targetAngles.FR = CalculateLegAngles(targetAngles.FR);
  targetAngles.FL = CalculateLegAngles(targetAngles.FL);
  targetAngles.BR = CalculateLegAngles(targetAngles.BR);
  targetAngles.BL = CalculateLegAngles(targetAngles.BL);

  return targetAngles;
}


struct AngleVector InverseKinematicsSolver::CalculateLegAngles(struct AngleVector anglesIK) {

  servoOffset.tetta = 90;
  servoOffset.alpha = 45;
  servoOffset.gamma = 45;
  
  anglesIKtmp.alpha = abs(anglesIK.alpha);
  anglesIKtmp.gamma = abs(anglesIK.gamma);
  
  anglesServo.tetta = anglesIK.tetta + servoOffset.tetta;
    
  l = sqrt(sq(legLenght) + sq(femurTendon) - 2.0*femurTendon*legLenght*cos(deg2rad(anglesIKtmp.gamma)));
  c1 = rad2deg(asin(femurTendon*sin(deg2rad(anglesIKtmp.gamma))/l));
  c2 = rad2deg(acos((sq(femurLever) + sq(l) - sq(legLenght))/(2.0*femurLever*l)));
  anglesServo.gamma = 180.0 - anglesIKtmp.alpha - c1 -c2 + servoOffset.gamma;
  anglesServo.gamma = - anglesServo.gamma + 180.0; //change servo sign
  
  l = sqrt(sq(axisOffset) + sq(femurTendon) - 2*femurTendon*axisOffset*cos(deg2rad(anglesIKtmp.alpha + 45)));
  c1 = rad2deg(asin(femurTendon*sin(deg2rad(anglesIKtmp.alpha + 45))/l));
  c2 = rad2deg(acos((sq(femurLever) + sq(l) - sq(axisOffset))/(2*femurLever*l)));
  anglesServo.alpha = 180.0 - 45.0 - c1 - c2 + servoOffset.alpha;
  
  return anglesServo;
}
