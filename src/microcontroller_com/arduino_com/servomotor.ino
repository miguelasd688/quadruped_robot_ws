



void connectServos() {
  analogWriteFrequency(m1, freq); //FR coxa
  digitalWrite(m1, LOW);
  pinMode(m1, OUTPUT);
  analogWriteFrequency(m2, freq); //femur
  digitalWrite(m2, LOW);
  pinMode(m2, OUTPUT);
  analogWriteFrequency(m3, freq); //tibia
  digitalWrite(m3, LOW);
  pinMode(m3, OUTPUT);

  analogWriteFrequency(m4, freq); //FL coxa
  digitalWrite(m4, LOW);
  pinMode(m4, OUTPUT);
  analogWriteFrequency(m5, freq); //femur
  digitalWrite(m5, LOW);
  pinMode(m5, OUTPUT);
  analogWriteFrequency(m6, freq); //tibia
  digitalWrite(m6, LOW);
  pinMode(m6, OUTPUT);

  analogWriteFrequency(m7, freq); //FR coxa
  digitalWrite(m7, LOW);
  pinMode(m7, OUTPUT);
  analogWriteFrequency(m8, freq); //femur
  digitalWrite(m8, LOW);
  pinMode(m8, OUTPUT);
  analogWriteFrequency(m9, freq); //tibia
  digitalWrite(m9, LOW);
  pinMode(m9, OUTPUT);

  analogWriteFrequency(m10, freq); //FR coxa
  digitalWrite(m10, LOW);
  pinMode(m10, OUTPUT);
  analogWriteFrequency(m11, freq); //femur
  digitalWrite(m11, LOW);
  pinMode(m11, OUTPUT);
  analogWriteFrequency(m12, freq); //tibia
  digitalWrite(m12, LOW);
  pinMode(m12, OUTPUT);

  analogWriteFrequency(m13, freq); //body
  digitalWrite(m13, LOW);
  pinMode(m13, OUTPUT);
}



void servoWrite(int pin , float angle) {

  float T = 1000000.0f / freq;
  float usec = float(MAX_PULSE - MIN_PULSE) * (angle / 180.0) + (float)MIN_PULSE;
  uint32_t duty = int(usec / T * 4096.0f);

  analogWrite(pin , duty);
}



float checkLimits(float angle , float lowLim , float highLim) {

  if ( angle >= highLim ) {
    angle = highLim;
  }
  if ( angle <= lowLim ) {
    angle = lowLim;
  }
  return angle;
}

void moveServos() {
  //FR
  anglesServoFR.tetta = checkLimits(anglesServoFR.tetta , lowLim[0] , highLim[0]);
  fineAngle = a[0] * anglesServoFR.tetta + b[0];
  servoWrite(m1 , fineAngle);

  anglesServoFR.alpha = checkLimits(anglesServoFR.alpha , lowLim[1] , highLim[1]);
  fineAngle = a[1] * anglesServoFR.alpha + b[1];
  servoWrite(m2 , fineAngle);

  anglesServoFR.gamma = checkLimits(anglesServoFR.gamma , lowLim[2] , highLim[2]);
  fineAngle = a[2] * anglesServoFR.gamma + b[2];
  servoWrite(m3 , fineAngle);

  //FL
  anglesServoFL.tetta = checkLimits(anglesServoFL.tetta , lowLim[3] , highLim[3]);
  fineAngle = a[3] * anglesServoFL.tetta + b[3];
  servoWrite(m4 , fineAngle);

  anglesServoFL.alpha = checkLimits(anglesServoFL.alpha , lowLim[4] , highLim[4]);
  fineAngle = a[4] * anglesServoFL.alpha + b[4];
  servoWrite(m5 , fineAngle);

  anglesServoFL.gamma = checkLimits(anglesServoFL.gamma , lowLim[5] , highLim[5]);
  fineAngle = a[5] * anglesServoFL.gamma + b[5];
  servoWrite(m6 , fineAngle);

  //BR
  anglesServoBR.tetta = checkLimits(anglesServoBR.tetta , lowLim[6] , highLim[6]);
  fineAngle = a[6] * anglesServoBR.tetta + b[6];
  servoWrite(m7 , fineAngle);

  anglesServoBR.alpha = checkLimits(anglesServoBR.alpha , lowLim[7] , highLim[7]);
  fineAngle = a[7] * anglesServoBR.alpha + b[7];
  servoWrite(m8 , fineAngle);

  anglesServoBR.gamma = checkLimits(anglesServoBR.gamma , lowLim[8] , highLim[8]);
  fineAngle = a[8] * anglesServoBR.gamma + b[8];
  servoWrite(m9 , fineAngle);

  //BL
  anglesServoBL.tetta = checkLimits(anglesServoBL.tetta , lowLim[9] , highLim[9]);
  fineAngle = a[9] * anglesServoBL.tetta + b[9];
  servoWrite(m10 , fineAngle);

  anglesServoBL.alpha = checkLimits(anglesServoBL.alpha , lowLim[10] , highLim[10]);
  fineAngle = a[10] * anglesServoBL.alpha + b[10];
  servoWrite(m11 , fineAngle);

  anglesServoBL.gamma = checkLimits(anglesServoBL.gamma , lowLim[11] , highLim[11]);
  fineAngle = a[11] * anglesServoBL.gamma + b[11];
  servoWrite(m12 , fineAngle);

  //BODY
  angleBody = checkLimits(angleBody , lowLim[12] , highLim[12]);
  fineAngle = a[12] * angleBody + b[12];
  servoWrite(m13 , fineAngle);
}


void stepMotors() {
  /*----------------if safe is already False, mantein servos in last position--------*/
  if (RUN == true) {
    SAFE = false;
    anglesServoFR = calculateIKangles(anglesIKFR);
    anglesServoFL = calculateIKangles(anglesIKFL);
    anglesServoBR = calculateIKangles(anglesIKBR);
    anglesServoBL = calculateIKangles(anglesIKBL);

    moveServos();
  }
  else if (RUN == false and SAFE == false) {
    SAFE = false;
    angleBody = oangleBody;

    anglesServoFR = oanglesServoFR;
    anglesServoFL = oanglesServoFL;
    anglesServoBR = oanglesServoBR;
    anglesServoBL = oanglesServoBL;

    moveServos();
  }
}
