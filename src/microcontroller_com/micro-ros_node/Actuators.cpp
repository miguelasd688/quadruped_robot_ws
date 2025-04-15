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

bool Actuators::StepMotors(bool ARMED, bool SAFE, int calAction, int calLeg, LegsAngle targetAngles) {
  //----------------if safe is already False, mantein servos in last position--------
  if (ARMED == true) {
    targetAngles = IK.CalculateRobotAngles(targetAngles);
    MoveServos(targetAngles);

    char buffer[200];
    snprintf(buffer, 200, "C: %.2f | F: %.2f | T: %.2f", targetAngles.asArray[0], targetAngles.asArray[1], targetAngles.asArray[2]);
    Debugger::Log(buffer);

    if (calibration.CheckStatus(calAction, calLeg, targetAngles)) {
      for (int i = 0; i < 3; i++) {
        a[i + 3 * calLeg] = calibration.ComputeInterpolationA();
        b[i + 3 * calLeg] = calibration.ComputeInterpolationB();
      }
    }
    return false;
  } else {
    return false;
  }
  return true;
}

void Actuators::MoveServos(LegsAngle targetAngles) {
  float angle;
  float ll;
  float hl;
  for (int i = 0; i < 12; i++) {
    anglesServo.asArray[i] = CheckLimits(targetAngles.asArray[i], lowLim[i], highLim[i]);
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