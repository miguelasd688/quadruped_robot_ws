#include "Actuators.h"

void Actuators::ConnectServos() {
  analogWriteResolution(12);
  for (int i = 0; i < sizeof(actuators_pin); i++) {
    SetNewServo(actuators_pin[i]);
  }
}


void Actuators::SetNewServo(uint32_t pin) {
  analogWriteFrequency(pin, freq);
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void Actuators::ServoWrite(uint32_t pin, float angle) {

  float T = 1000000.0f / freq;
  float usec = float(MAX_PULSE - MIN_PULSE) * (angle / 180.0) + (float)MIN_PULSE;
  uint32_t duty = int(usec / T * 4096.0f);
  analogWrite(pin, duty);
}



float Actuators::CheckLimits(float angle, float lowLim, float highLim) {

  if (angle >= highLim) {
    angle = highLim;
    return angle;
  }
  if (angle <= lowLim) {
    angle = lowLim;
    return angle;
  }
  return angle;
}

void Actuators::MoveServos() {
  for (int i = 0; i < sizeof(actuators_pin); i++) {
    anglesServo[i] = CheckLimits(anglesServo[i], lowLim[i], highLim[i]);
    fineAngle = a[i] * anglesServo[i] + b[i];
    ServoWrite(actuators_pin[i], fineAngle);
  }
}


bool Actuators::StepMotors(bool RUN, bool SAFE, LegsAngle targetAngles) {
  //----------------if safe is already False, mantein servos in last position--------
  if (RUN == true) {
    targetAngles = IK.CalculateRobotAngles(targetAngles);

    anglesServo[0] = targetAngles.FR.tetta;
    anglesServo[1] = targetAngles.FR.alpha;
    anglesServo[2] = targetAngles.FR.gamma;
    anglesServo[3] = targetAngles.FL.tetta;
    anglesServo[4] = targetAngles.FL.alpha;
    anglesServo[5] = targetAngles.FL.gamma;
    anglesServo[6] = targetAngles.BR.tetta;
    anglesServo[7] = targetAngles.BR.alpha;
    anglesServo[8] = targetAngles.BR.gamma;
    anglesServo[9] = targetAngles.BL.tetta;
    anglesServo[10] = targetAngles.BL.alpha;
    anglesServo[11] = targetAngles.BL.gamma;
    /*for (int i = 0; i < sizeof(anglesServo); i++)
    {
      oAnglesServo[i] = anglesServo[i];
    } */
    MoveServos();
    return false;
  } else if (RUN == false and SAFE == false) {
    /*for (int i = 0; i < sizeof(anglesServo); i++)
    {
      anglesServo[i] = oAnglesServo[i];
    } */
    //MoveServos();
    return false;
  }
  return true;
}