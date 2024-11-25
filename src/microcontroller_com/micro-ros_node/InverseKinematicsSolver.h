
#include "utils.h"

class InverseKinematicsSolver {
private:
  struct AngleVector anglesIKtmp;
  struct AngleVector servoOffset;
  struct AngleVector anglesServo;

  float l , c1 , c2;
  float legLenght = 90.0;
  float axisOffset = 29.0;
  
  float tibiaLever = 24.0;//20.5; //lenght of servo horn has two positions
  float tibiaTendon = 24.0; 
  
  float femurLever = 24.0; 
  float femurTendon = 24.0;

public:
  LegsAngle CalculateRobotAngles(LegsAngle targetAngles);
  struct AngleVector CalculateLegAngles(struct AngleVector anglesIK);
};