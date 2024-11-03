#define MAX_PULSE 2500
#define MIN_PULSE 560

/*---------------SERVO PIN DEFINITION------------------------*/
int freq = 125; // PWM frecuency can be choosen here.

int actuators_pin[12] = { 6, 5, 4,//FR
                          28, 29, 36,//FL
                          3, 2, 1,//BR
                          7, 24, 25//BL
                        };


int motorPin;
int motor = 0;

/*----------------- CALIBRATION PARAMETERS OF EACH SERVO -----------------*/
//Regression data for motor
//                 coxa      femur     tibia
float a[13] = { -1.08333, -1.06667, -1.07778, //FR
                 -1.03333,  0.97778,  1.01111, //FL
                 1.03333,  1.05556,  1.07778,   //BR
                 1.07500, -1.07778, -1.00000, //BL
               };

float b[13] = {183.0, 197.0, 165.5, //FR
              193.0,   5.5,  -2.5,   //FL
                5.0, -20.0, -11.0,     //BR
                -13.5, 191.5, 182.0, //BL
               };
//Regression data for encoders
//               coxa      femur    tibia
float ae[13] = {0.20292, 0.20317, 0.19904 ,
                 0.21256, -0.22492, -0.21321,
                 -0.21047, -0.20355, -0.20095,
                 -0.20265, 0.19904, 0.20337,
                };

float be[13] = { -18.59717, -5.70512, -2.51697,
                  -5.75856, 197.29411, 202.72169,
                  185.96931, 204.11902, 199.38663,
                  197.89534, -5.33768, -32.23424,
                };
                
float lowLim[13] = {50, 30, 30, 50, 30, 30, 50, 30, 30, 50, 30, 30};
float highLim[13] = {130, 150, 150, 130, 150, 150, 130, 150, 150, 130, 150, 150};

float fineAngle;

void connectServos() {
  for (int i = 0; i < sizeof(actuators_pin); i++)
  {
    setNewServo(actuators_pin[i], freq);
  }
}


void setNewServo(int pin, int freq)
{
  analogWriteFrequency(pin, freq); //tibia
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
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

  for (int i = 0; i < sizeof(actuators_pin); i++)
  {
    anglesServo[i] = checkLimits(anglesServo[i] , lowLim[i] , highLim[i]);
    fineAngle = a[i] * anglesServoFR.tetta + b[i];
    servoWrite(actuators_pin[i] , fineAngle);
  }
}


void stepMotors() {
  /*----------------if safe is already False, mantein servos in last position--------*/
  if (RUN == true) {
    SAFE = false;

    anglesServoFR = calculateIKangles(anglesIKFR);
    anglesServoFL = calculateIKangles(anglesIKFL);
    anglesServoBR = calculateIKangles(anglesIKBR);
    anglesServoBL = calculateIKangles(anglesIKBL);
    
    anglesServo[0] = anglesServoFR.tetta;
    anglesServo[1] = anglesServoFR.alpha;
    anglesServo[2] = anglesServoFR.gamma;
    anglesServo[3] = anglesServoFR.tetta;
    anglesServo[4] = anglesServoFR.alpha;
    anglesServo[5] = anglesServoFR.gamma;
    anglesServo[6] = anglesServoFR.tetta;
    anglesServo[7] = anglesServoFR.alpha;
    anglesServo[8] = anglesServoFR.gamma;
    anglesServo[9] = anglesServoFR.tetta;
    anglesServo[10] = anglesServoFR.alpha;
    anglesServo[11] = anglesServoFR.gamma;
    for (int i = 0; i < sizeof(anglesServo); i++)
    {
      oAnglesServo[i] = anglesServo[i];
    }  
    moveServos();
  }
  else if (RUN == false and SAFE == false) {
    SAFE = false;
    for (int i = 0; i < sizeof(anglesServo); i++)
    {
      anglesServo[i] = oAnglesServo[i];
    }  
    moveServos();
  }
}
