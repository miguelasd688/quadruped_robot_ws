#include "Actuators.h"

void Actuators::ConnectServos() {
  analogWriteResolution(12);
  for (uint32_t i = 0; i < 12; i++) {
    SetNewServo(actuators_pin[i]);
  }
}

void Actuators::SetNewServo(uint32_t pin) {
  analogWriteFrequency(pin, freq);
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

bool Actuators::StepMotors(bool RUN, bool SAFE, struct LegsAngle targetAngles) {
  //----------------if safe is already False, mantein servos in last position--------
  if (RUN == true) {
    targetAngles = IK.CalculateRobotAngles(targetAngles);
    for (int i = 0; i < 12; i++) {
      anglesServo.asArray[i] = targetAngles.asArray[i];
    }
    Actuators::MoveServos();
    return false;
  } else if (RUN == false and SAFE == false) {
    return false;
  }
  return true;
}

void Actuators::MoveServos() {
  float angle;
  float ll;
  float hl;
  for (int i = 0; i < 12; i++) {
    anglesServo.asArray[i] = CheckLimits(anglesServo.asArray[i], lowLim[i], highLim[i]);
    fineAngle = a[i] * anglesServo.asArray[i] + b[i];
    ServoWrite(actuators_pin[i], fineAngle);
  }
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