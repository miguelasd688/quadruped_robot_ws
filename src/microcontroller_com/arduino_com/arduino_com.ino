#include <math.h>

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))



boolean newData = false;
boolean recvComplete = false;
boolean CAL = false;
boolean secuence = false;
boolean LCDen = true;
boolean CTRL1 = false;
boolean CTRL2 = false;
boolean REST = false;
boolean SAFE = true;
boolean KILL = false;
boolean RUN = false;
boolean Push = false;
boolean oPush = false;

int PAGE = 0;
int maxPages = 3; // MAX PAGES

int pinVin = 27; // pin (teensy 4.1: 21)
int pinLed = 31; // pin status led
int pinPush = 32; // pin push botton

int spaceCounter = 0;

//VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMicros = 0;
const long interval = 10000; //microsec
unsigned long loopTime;
unsigned long previousLooptime;

float pressAt;
float Vin = 0;

float t = 0;
float ot;

struct vector {
  float tetta;
  float alpha;
  float gamma;
};////vector

float angleBody;
float oangleBody;


float fe1 = 0;
float fe2 = 0;
float fe3 = 0;
float fe4 = 0;
float we1;
float we2;
float we3;
float we4;

float yaw = 10000;
float pitch = 10000;
float roll = 10000;
float Xacc = 10000;
float Yacc = 10000;
float Zacc = 10000;
float Xrot = 10000;
float Yrot = 10000;
float Zrot = 10000;
float boardTemp = 10000;

struct vector anglesIKFR;
struct vector anglesIKFL;
struct vector anglesIKBR;
struct vector anglesIKBL;

struct vector anglesRawFR;
struct vector anglesRawFL;
struct vector anglesRawBR;
struct vector anglesRawBL;

struct vector anglesServoFR;
struct vector anglesServoFL;
struct vector anglesServoBR;
struct vector anglesServoBL;

struct vector oanglesServoFR;
struct vector oanglesServoFL;
struct vector oanglesServoBR;
struct vector oanglesServoBL;

struct vector contactForces;
struct vector contactForcesFL;
struct vector contactForcesBR;
struct vector contactForcesBL;

struct vector sensorOffset0;
struct vector sensorOffset1;


String POWER = "ON";
//VARIABLES PARA RECIVIR EL COMANDO
const byte numChars = 32;
char receivedChars[numChars];

//<0#-45#90#0#-45#90#0#45#-90#0#45#-90#90#0#0#0#0>


void setupA() {

  setupDisplay();
  Serial.begin(2000000);

  analogWriteResolution(12);

  pinMode(pinLed, OUTPUT);
  pinMode(pinPush, INPUT);

  //  IMUSetup();

  connectServos();

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

  anglesServoFR = calculateIKangles(anglesIKFR);
  anglesServoFL = calculateIKangles(anglesIKFL);
  anglesServoBR = calculateIKangles(anglesIKBR);
  anglesServoBL = calculateIKangles(anglesIKBL);

  oangleBody = angleBody;
  oanglesServoFR = anglesServoFR;
  oanglesServoFL = anglesServoFL;
  oanglesServoBR = anglesServoBR;
  oanglesServoBL = anglesServoBL;
  /*calibrationDisplay();
    sensorOffset0 = calibrateContacts();
    startDisplay();
    delay(2000);
    calibrationDisplay();
    sensorOffset1 = calibrateContacts();*/


}


void loopA() {


  // put your main code here, to run repeatedly:
  unsigned long currentMicros = micros();
  unsigned long currentMillisIMU = millis();


  //if (recvComplete == true and newData == false) {
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;
    ot = t;
    t = round(float(currentMicros) / 10000.0) / 100.;
    recvComplete = false;
    /////////cuenta el tiempo que tarda el bucle en ejecutarse
    loopTime = currentMicros - previousLooptime;
    previousLooptime = currentMicros;




    /*----------------Send msg to serial--------------
       Modes: DEBUG print to serial prompt
              WRITE send byte package to RPI          */
    SerialDebugANGLES();
    //SerialWriteRPI();
    /*----------------READ ANGLES FROM SERIAL---------------*/
    recvWithStartEndMarkers();
    newData = false;
     /*--------------DEBUG position: bipass angles to directly move servos---------*/
    //anglesDEBUG();
    
    /*----------------Move servos only if RUN flag is ON----------------------------*/
    stepMotors();
   

    /*---------------- reset MC if commanded ---------------*/
    if (KILL == true) {
      WRITE_RESTART(0x5FA0004);
    }

    /*----------------SELECT SCREEN MODE---------------*/
    selectDisplayPage();

    /*----------------READ BATTERY STATUS---------------*/
    Vin = float(analogRead(pinVin));
    //feed battery IN with two reference voltage to relate with analog signal
    Vin = map(Vin, 191, 83, 11.91 , 5.12);
    //Vin = 5.6 + 1.4*(1 + sin(3.14*t/5));
    //Vin = 7;

    /*----------------UPDATE STATUS LED---------------*/
    if (RUN == true) {
      digitalWrite(pinLed, HIGH);
    }
    else {
      digitalWrite(pinLed, LOW);
    }

    /*-----------------------UPDATE SCREEN ------------------------*/
    printDisplayLCD();

    /*--------------READ ENCODER and compare it with last commanded servo angle----------------*/
    readEncoderAngles();
    //readEncoders();

    /*----------------READ IMU---------------*/
    //    uint8_t sys, gyr, accel, mg = 0;
    //    readIMU();

    /*----------------Make all calculations for force stimation----------------*/
    estimateForce();

    
  }
}


void anglesDEBUG() {
  anglesIKBL.alpha = 45.0 + 10 * sin(6.28 * t / 0.8);
  anglesIKBL.gamma = -90.0 - 20 * sin(6.28 * t / 0.8);
  anglesIKBR.alpha = 45.0 + 10 * sin(6.28 * t / 0.8);
  anglesIKBR.gamma = -90.0 - 20 * sin(6.28 * t / 0.8);
  RUN = true;
}
