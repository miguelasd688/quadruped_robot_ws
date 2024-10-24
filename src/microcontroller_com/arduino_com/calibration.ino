#define MAX_PULSE 2500
#define MIN_PULSE 560

/*---------------SERVO PIN DEFINITION------------------------*/
bool interp = false;
bool question = true;
bool swing = false;
int i;

int freq = 125; // PWM frecuency can be choosen here.
int m1 = 6;//FR
int m2 = 5;
int m3 = 4;

int m4 = 28;//FL
int m5 = 29;
int m6 = 36;

int m7 = 3;//BR
int m8 = 2;
int m9 = 1;

int m10 = 7;//BL
int m11 = 24;
int m12 = 25;

int m13 = 0;//BODY

int motorPin;
int motor = 0;

/*----------------- CALIBRATION PARAMETERS OF EACH SERVO -----------------*/
//Regression data for motor
//                 coxa      femur     tibia
float a[13] = { -1.08333, -1.06667, -1.07778, //FR
                 -1.03333,  0.97778,  1.01111, //FL
                 1.03333,  1.05556,  1.07778,   //BR
                 1.07500, -1.07778, -1.00000, //BL
                 1.06250
               };

float b[13] = {183.0, 197.0, 165.5, //FR
                193.0,   5.5,   -2.5,   //FL
                5.0, -20.0, -11.0,     //BR
                -13.5, 191.5, 182.0, //BL
                -0.875
               };
//Regression data for encoders
//               coxa      femur    tibia
float ae[13] = {0.20292, 0.20317, 0.19904 ,
                 0.21256, -0.22492, -0.21321,
                 -0.21047, -0.20355, -0.20095,
                 -0.20265, 0.19904, 0.20337,
                 -0.20226
                };

float be[13] = { -18.59717, -5.70512, -2.51697,
                  -5.75856, 197.29411, 202.72169,
                  185.96931, 204.11902, 199.38663,
                  197.89534, -5.33768, -32.23424,
                  187.48058
                };
                
float lowLim[13] = {50, 30, 30, 50, 30, 30, 50, 30, 30, 50, 30, 30, 70};
float highLim[13] = {130, 150, 150, 130, 150, 150, 130, 150, 150, 130, 150, 150, 110};

/*--------Corresponding angles you want to meassure at in your system-----------*/
float x1[13] = {120, 135,  90,  60, 135 , 90, 120, 135,  90,  60, 135,  90, 110}; //this will be the first angle you will meassure
float x2[13] = {60,   90, 135, 120,  90, 135,  60,  90, 135, 120,  90, 135,  70};//this will be the second angle you will meassure for calibration

float ang1[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float ang2[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float xi[500];
float yi[500];

float fineAngle;
float fineL;
float fineH;

float calibrationAngle;
float res = 1.0;
float ares = 0.5;
float bres = 1.0;
float cres = 4.0;

float rawAngle;
float orawAngle;
float eang;
char cm;
char answer;
/*--------You can define a motor tag for each servo--------*/
String motorTag[13] = {"FR coxa", "FR femur", "FR tibia",
                       "FL coxa", "FL femur", "FL tibia",
                       "BR coxa", "BR femur", "BR tibia",
                       "BL coxa", "BL femur", "BL tibia", "Body angle"
                      };



int motorInfo(int i) {
  readEncoders();
  if (i == 0) {
    rawAngle = enc1;
    motorPin = m1;
  }
  else if (i == 1) {
    rawAngle = enc2;
    motorPin = m2;
  }
  else if (i == 2) {
    rawAngle = enc3;
    motorPin = m3;
  }
  else if (i == 3) {
    rawAngle = enc4;
    motorPin = m4;
  }
  else if (i == 4) {
    rawAngle = enc5;
    motorPin = m5;
  }
  else if (i == 5) {
    rawAngle = enc6;
    motorPin = m6;
  }
  else if (i == 6) {
    rawAngle = enc7;
    motorPin = m7;
  }
  else if (i == 7) {
    rawAngle = enc8;
    motorPin = m8;
  }
  else if (i == 8) {
    rawAngle = enc9;
    motorPin = m9;
  }
  else if (i == 9) {
    rawAngle = enc10;
    motorPin = m10;
  }
  else if (i == 10) {
    rawAngle = enc11;
    motorPin = m11;
  }
  else if (i == 11) {
    rawAngle = enc12;
    motorPin = m12;
  }
  else if (i == 12) {
    rawAngle = enc13;
    motorPin = m13;
  }
  return rawAngle , motorPin;
}



void calibrationSecuence( ) {
  //set servos at their middle position at firstt
  for (int i = 0; i <= 12; i++) {
    rawAngle , motorPin = motorInfo(i);
    servoWrite(motorPin , 90);
  }
  //    sensorOffset0 = calibrateContacts();
  Serial.println(" ");
  Serial.println("_________________________________SERVO CALIBRATION ROUTINE_________________________________");
  Serial.println("___________________________________________________________________________________________");
  Serial.println("(*) Don't send several caracter at the same time.");
  delay(500);
  Serial.println(" ");
  Serial.println("Keyboard: 'x'-> EXIT CALIBRATION.   'c'-> ENTER CALIBRATION.");
  Serial.println("          'i'-> PRINT INFORMATION.   ");
  Serial.println(" ");
  Serial.println("          'n'-> CHANGE MOTOR (+).   'b' -> CHANGE MOTOR (-).");
  Serial.println("          'r'-> CHANGE RESOLUTION.");
  Serial.println("          'm'-> START CALIBRATION.");
  Serial.println("          'q'-> STOP CALIBRATION.");
  Serial.println(" ");
  Serial.println("     · Once servo calibration started:");
  Serial.println("          'p'-> ADD ANGLE.          'o'-> SUBTRACT ANGLE.   ");
  Serial.println("          's'-> SAVE ANGLE.");
  delay(500);
  Serial.println(" ");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.print("SELECTED MOTOR: ");
  Serial.print(motorTag[motor]);
  Serial.print(".     SELECTED RESOLUTION: ");
  Serial.println(res);


  while (CAL == true) {
    if (Serial.available() > 0) {
      cm = Serial.read();
      if (cm == 'x') {
        Serial.println("Closing CALIBRATION program...");
        CAL = false;
        secuence = false;
        startDisplay(PAGE);
        angleBody = 90;
        anglesIKFR.tetta = 0.0;
        anglesIKFR.alpha = -45.0;
        anglesIKFR.gamma = 90.0;

        anglesIKFL.tetta = 0.0;
        anglesIKFL.alpha = -45.0;
        anglesIKFL.gamma = 90.0;

        anglesIKBR.tetta = 0.0;
        anglesIKBR.alpha = 45.0;
        anglesIKBR.gamma = -90.0;

        anglesIKBL.tetta = 0.0;
        anglesIKBL.alpha = 45.0;
        anglesIKBL.gamma = -90.0;
      }
      else if (cm == 'i') { // +
        Serial.println(" ");
        Serial.println("---------------------------------------------------------------------------------------------------");
        Serial.println("---------------------------------------------------------------------------------------------------");
        Serial.println("(*) Don't send several caracter at the same time.");
        delay(500);
        Serial.println(" ");
        Serial.println("Keyboard: 'x'-> EXIT CALIBRATION.   'c'-> ENTER CALIBRATION.");
        Serial.println("          'i'-> PRINT INFORMATION.   ");
        Serial.println(" ");
        Serial.println("          'n'-> CHANGE MOTOR (+).   'b' -> CHANGE MOTOR (-).");
        Serial.println("          'r'-> CHANGE RESOLUTION.");
        Serial.println("          'm'-> START CALIBRATION.");
        Serial.println("          'q'-> STOP CALIBRATION.");
        Serial.println(" ");
        Serial.println("     · Once servo calibration started:");
        Serial.println("          'p'-> ADD ANGLE.          'o'-> SUBTRACT ANGLE.   ");
        Serial.println("          's'-> SAVE ANGLE.");
        Serial.println(" ");
        delay(500);
        Serial.println(" ");
        Serial.println("---------------------------------------------------------------------------------------------------");
        Serial.println(" ");
        Serial.print("SELECTED MOTOR: ");
        Serial.print(motorTag[motor]);
        Serial.print(".     SELECTED RESOLUTION: ");
        Serial.println(res);
        Serial.println("Actual parameters of the motor: ");
        Serial.print("High limit: ");
        Serial.print(highLim[motor]);
        Serial.print("   Low limit: ");
        Serial.print(lowLim[motor]);
        Serial.print("   Angle 1: ");
        Serial.print(ang1[motor]);

        Serial.print("   Angle 2: ");
        Serial.println(ang2[motor]);
        Serial.println("---------------------------------------------------------------------------------------------------");



      }
      else if (cm == 'm') { // +
        secuence = true;
      }
      else if (cm == 's') { // +
      }
      else if (cm == 'n') { // +
        motor++;
        if (motor >= 13) {
          motor = 0;
        }
        Serial.print("SELECTED MOTOR: ");
        Serial.println(motorTag[motor]);
      }
      else if (cm == 'b') { // +
        motor--;
        if (motor < 0) {
          motor = 13 - 1;
        }
        Serial.print("SELECTED MOTOR: ");
        Serial.println(motorTag[motor]);
      }
      else if (cm == 'r') { // +
        if (res == ares) {
          res = bres;
        }
        else if (res == bres) {
          res = cres;
        }
        else if (res == cres) {
          res = ares;
        }
        Serial.print("SELECTED RESOLUTION: ");
        Serial.println(res);
      }
    }


    if (secuence == true) {
      Serial.print("Starting secuence for motor: ");
      Serial.println(motorTag[motor]);
      for (int i = 0; i <= 30; i++) {
        delay(20);
        Serial.print(".");
      }
      Serial.println(".");
      Serial.println("Do you want to calibrate internal encoder only? (y/n)");
      while (question == true) {
        unsigned long currentMicros = micros();
        if (currentMicros - previousMicros >= 100000) {
          previousMicros = currentMicros;
          if (Serial.available() > 0) {
            answer = Serial.read();
            if (answer == 'y') {
              question = false;
              interp = true;
              secuence = true;
            }
            else if (answer == 'n') {
              question = false;
              interp = false;
              secuence = true;
            }
            //else {
            //  Serial.println("Please, select Yes(y) or No(n).");
            //}
          }
        }
      }
      answer = 't';
      question = true;

      if (interp == false) {
        Serial.println("___");
        Serial.println("   | Place motor at 1ts position and save angle");
        Serial.println("   | This position can be the higher one");
        rawAngle , motorPin = motorInfo(motor);
        calibrationAngle = 90; //start calibration at aproximate middle position of the servo.

        while (secuence == true) { /*   find first calibration angle    */
          if (Serial.available() > 0) {
            cm = Serial.read();
            if (cm == 'p') { // +
              Serial.print("   | +");
              Serial.print(res);
              Serial.print(" : ");
              calibrationAngle = calibrationAngle + res;
              servoWrite(motorPin , calibrationAngle);
              Serial.println(calibrationAngle);
            }
            else if (cm == 'o') { // -
              Serial.print("   | -");
              Serial.print(res);
              Serial.print(" : ");

              calibrationAngle = calibrationAngle - res;
              servoWrite(motorPin , calibrationAngle);
              Serial.println(calibrationAngle);
            }
            else if (cm == 'r') { // +
              if (res == ares) {
                res = bres;
              }
              else if (res == bres) {
                res = cres;
              }
              else if (res == cres) {
                res = ares;
              }
              Serial.print("SELECTED RESOLUTION: ");
              Serial.println(res);
            }


            else if (cm == 'q') { //    quit secuence
              secuence = false;
              Serial.println("   |  Calibration interrupted!!");
            }
            else if (cm == 's') { //    save angle
              ang1[motor] = calibrationAngle;
              secuence = false;
              Serial.print("   | Angle saved at ");
              Serial.println(calibrationAngle);
            }
          }
        }

        if (cm == 'q') {
          Serial.println("   |");
        }
        else {
          secuence = true;
          Serial.println("___");
          Serial.println("   | Place motor at 2nd position and save angle");
          Serial.println("   | This position can be the lower one");
        }



        while (secuence == true) {     /*   find second calibration angle    */
          if (Serial.available() > 0) {
            cm = Serial.read();
            if (cm == 'p') { // +
              Serial.print("   | +");
              Serial.print(res);
              Serial.print(" : ");
              calibrationAngle = calibrationAngle + res;
              servoWrite(motorPin , calibrationAngle);
              Serial.println(calibrationAngle);
            }
            else if (cm == 'o') { // -
              Serial.print("   | -");
              Serial.print(res);
              Serial.print(" : ");

              calibrationAngle = calibrationAngle - res;
              servoWrite(motorPin , calibrationAngle);
              Serial.println(calibrationAngle);
            }
            else if (cm == 'r') { // +
              if (res == ares) {
                res = bres;
              }
              else if (res == bres) {
                res = cres;
              }
              else if (res == cres) {
                res = ares;
              }
              Serial.print("SELECTED RESOLUTION: ");
              Serial.println(res);
            }

            else if (cm == 'q') { //    quit secuence
              secuence = false;
              Serial.println("   |  Calibration interrupted!!");
            }
            else if (cm == 's') { //     save angle
              ang2[motor] = calibrationAngle;
              secuence = false;
              Serial.print("   | Angle saved at ");
              Serial.println(calibrationAngle);
            }
          }
        }
        /*--------------------start calibration calculations------------------*/
        if (cm == 'q') {
          Serial.println("___|");
          Serial.println("Calibration finished unespected.");

          Serial.println(" Select another motor.");
          Serial.print("SELECTED MOTOR: ");
          Serial.print(motorTag[motor]);
          Serial.print(".     SELECTED RESOLUTION: ");
          Serial.println(res);
        }
        else {
          Serial.println("___");
          Serial.println("   |___");
          Serial.print(  "   |   | Interpolating for motor: ");
          Serial.println(motorTag[motor]);
          secuence = true;
          //real angle is calculated interpolating both angles to a linear relation.
          a[motor] = (ang2[motor] - ang1[motor]) / (x2[motor] - x1[motor]);
          b[motor] = ang1[motor] - x1[motor] * (ang2[motor] - ang1[motor]) / (x2[motor] - x1[motor]);

          Serial.println("   |   |");
        }
        interp = true;
      }



      /*---------------------------make swing movement to interpolate motor encoder-----*/
      if (interp == true and secuence == true) {
        delay(200);
        float x;
        int k = 0;


        int stp = 180;

        swing = true;
        i = 0;
        orawAngle , motorPin = motorInfo(motor);
        previousMicros = 0;

        while (swing == true) { // FIRST
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x2[motor];
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);
            rawAngle , motorPin = motorInfo(motor);

            if ((i % 3) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;

              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }

            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }

        swing = true;
        i = 0;

        while (swing == true) { // moving
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x2[motor] + float(i) * (x1[motor] - x2[motor]) / stp;
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);
            rawAngle , motorPin = motorInfo(motor);

            if ((i % 6) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;

              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }

            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }

        swing = true;
        i = 0;

        while (swing == true) { // SECOND
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x1[motor];
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);
            rawAngle , motorPin = motorInfo(motor);

            if ((i % 3) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;

              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }
            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }

        swing = true;
        i = 0;

        while (swing == true) { // moving
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x1[motor] + float(i) * (x2[motor] - x1[motor]) / stp;
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);
            rawAngle , motorPin = motorInfo(motor);

            if ((i % 6) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;

              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }
            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }

        swing = true;
        i = 0;

        while (swing == true) { // FIRST
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x2[motor];
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);
            rawAngle , motorPin = motorInfo(motor);

            if ((i % 3) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;

              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }

            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }

        swing = true;
        i = 0;

        while (swing == true) { // moving
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x2[motor] + float(i) * (x1[motor] - x2[motor]) / stp;
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);

            rawAngle , motorPin = motorInfo(motor);

            if ((i % 6) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;
              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }


            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }

        swing = true;
        i = 0;

        while (swing == true) { // SECOND
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;


            x = x1[motor];
            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);
            rawAngle , motorPin = motorInfo(motor);

            if ((i % 3) == 0) {
              yi[k + 1] = x;
              xi[k] = rawAngle;

              Serial.print("   |   | Real ang: ");
              Serial.print(x);
              Serial.print(" -> Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print("  Enc: ");
              Serial.println(rawAngle);
              k++;
            }
            if (i >= stp) {
              swing = false;
            }
            i++;
          }
        }


        Serial.println("   |   | Interpolation finished!");


        /*-------Calculate linear interpolation of the encoder from 60 meassures done in swing------*/
        float sx = 0;
        float sy = 0;
        float sx2 = 0;
        float sy2 = 0;
        float sxy = 0;
        float xmean = 0;
        float ymean = 0;
        int n = 300;

        for (int i = 0 ; i < n ; i++) {
          sx += xi[i + 10];
          sy += yi[i + 10];
          sx2 += xi[i + 10] * xi[i + 10];
          sy2 += yi[i + 10] * yi[i + 10];
          sxy += xi[i + 10] * yi[i + 10];
        }
        ae[motor] =  (n * sxy - sx * sy) / (n * sx2 - sx * sx); //sxy / sx2; //
        be[motor] = (sy - ae[motor] * sx) / n; //ymean - ae[motor] * xmean;

        Serial.println("   |   | Moving back to ZERO position."); // turn the motor back to middle position

        swing = true;
        i = 0;

        while (swing == true) {
          unsigned long currentMicros = micros();
          if (currentMicros - previousMicros >= 10000) {
            // save the last time you blinked the LED
            previousMicros = currentMicros;

            x = x1[motor] + float(i) * (90 - x1[motor]) / 60;

            calibrationAngle = a[motor] * x + b[motor];
            servoWrite(motorPin , calibrationAngle);

            rawAngle , motorPin = motorInfo(motor);

            eang = ae[motor] * rawAngle + be[motor];

            if ((i % 4) == 0) {
              Serial.print("   |   | Servo ang: ");
              Serial.print(calibrationAngle);
              Serial.print(" -> Real ang: ");
              Serial.print(x);
              Serial.print(" -> Encoder ang: ");
              Serial.println(eang);
            }
            if (i >= 60) {
              swing = false;
            }
            i++;
          }
        }


        Serial.println("___|___|");
        Serial.println("   | ");
        Serial.println("___");
        Serial.println("   | Calibration finished satisfactory. Results data:");
        Serial.print("   | HIGH lim: "); Serial.print(highLim[motor]);
        Serial.print("   LOW lim: "); Serial.println(lowLim[motor]);
        Serial.print("   | angle 1: "); Serial.print(ang1[motor]);
        Serial.print("   angle 2 "); Serial.println(ang2[motor]);
        Serial.print("   | Regression Motor a: "); Serial.print(a[motor], 5);
        Serial.print("                  b: "); Serial.println(b[motor], 5);
        Serial.print("   | Regression Encoder a: "); Serial.print(ae[motor], 5);
        Serial.print("                    b: "); Serial.println(be[motor], 5);
        Serial.println("   |");
        Serial.println("   |   _____________________________________________________________");
        Serial.println("   |  |                                                             |");
        Serial.println("   |  |   This code won't be able to save the updated parameters    |");
        Serial.println("   |  |   once the robot is shutted down.                           |");
        Serial.println("   |  |                                                             |");
        Serial.println("   |  |   Please, write down the results                            |");
        Serial.println("   |  |   and save them in the definition of each variable.         |");
        Serial.println("   |  |_____________________________________________________________|");
        Serial.println("   |");
        Serial.println("___|");
        Serial.println(" Select another motor.");
        Serial.print("SELECTED MOTOR: ");
        Serial.print(motorTag[motor]);
        Serial.print(".     SELECTED RESOLUTION: ");
        Serial.println(res);
      }
      interp = false;
      secuence = false;
    }
  }
  SAFE = false;
  Serial.println("Calibration killed");
  WRITE_RESTART(0x5FA0004);
}

// END OF CALIBRATION
