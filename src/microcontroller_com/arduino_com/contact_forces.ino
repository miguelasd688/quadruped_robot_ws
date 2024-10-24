//int ffs0 = 0; //A0
//int ffs1 = 1; //A1
//int ffs2 = 2; //A2

int ffs0data = 0;
int ffs1data = 0;
int ffs2data = 0;
int n = 500;
int k;

float vout0;
float vout1;
float vout2;

float vout0prev = 0;
float vout1prev = 0;
float vout2prev = 0;

float sum0;
float sum1;
float sum2;

struct vector sensorOffset;

struct vector calibrateContacts() {
  sum0 = 0;
  sum1 = 0;
  sum2 = 0;
  vout0prev = 0;
  vout1prev = 0;
  vout2prev = 0;

  for (k = 0; k <= n ; k++) {
    //    ffs0data = analogRead(ffs0);
    //    ffs1data = analogRead(ffs1);
    //    ffs2data = analogRead(ffs2);
    vout0 = ffs0data * 0.05 + vout0prev * 0.95;
    vout1 = ffs1data * 0.05 + vout1prev * 0.95;
    vout2 = ffs2data * 0.05 + vout2prev * 0.95;

    vout0prev = vout0;
    vout1prev = vout1;
    vout2prev = vout2;
    sum0 = sum0 + vout0;

    sum1 = sum1 + vout1;
    sum2 = sum2 + vout2;
    delay(10);
  }
  sensorOffset.alpha = sum0 / n;
  sensorOffset.tetta = sum1 / n;
  sensorOffset.gamma = sum2 / n;

  return sensorOffset;
}

struct vector readContacts(struct vector sensorOffset0 , struct vector sensorOffset1) {

  //  ffs0data = analogRead(ffs0);
  //  ffs1data = analogRead(ffs1);
  //  ffs2data = analogRead(ffs2);

  vout0 = ffs0data * 0.05 + vout0prev * 0.95;
  vout1 = ffs1data * 0.05 + vout1prev * 0.95;
  vout2 = ffs2data * 0.05 + vout2prev * 0.95;

  vout0prev = vout0;
  vout1prev = vout1;
  vout2prev = vout2;
  /*
    contactForces.tetta = map(vout0 - sensorOffset0.tetta , sensorOffset0.tetta , sensorOffset1.tetta , 0.0 , 1.0);
    contactForces.alpha = map(vout1 - sensorOffset0.alpha , sensorOffset0.alpha , sensorOffset1.alpha , 0.0 , 1.0);
    contactForces.gamma = map(vout2 - sensorOffset0.gamma , sensorOffset0.gamma , sensorOffset1.gamma , 0.0 , 1.0);
  */
  contactForces.tetta = vout0 - sensorOffset0.tetta;
  contactForces.alpha = vout1 - sensorOffset0.alpha;
  contactForces.gamma = vout2 - sensorOffset0.gamma;

  return contactForces;
}

void estimateForce() {
  fe1 = sqrt(pow(oanglesServoFR.alpha - eang2, 2) + pow(oanglesServoFR.gamma - eang3, 2));
  fe2 = sqrt(pow(oanglesServoFL.alpha - eang5, 2) + pow(oanglesServoFL.alpha - eang6, 2));
  fe3 = sqrt(pow(oanglesServoBR.alpha - eang8, 2) + pow(oanglesServoBR.gamma - eang9, 2));
  fe4 = sqrt(pow(oanglesServoBL.alpha - eang11, 2) + pow(oanglesServoBL.gamma - eang12, 2));// - abs(16*we4) ;
  oanglesServoFR = anglesServoFR;
  oanglesServoFL = anglesServoFL;
  oanglesServoBR = anglesServoBR;
  oanglesServoBL = anglesServoBL;

  //    we1 = (anglesServoFR.tetta - oanglesServoFR)/(t-ot);
  //    we2 = (anglesServoFL - oanglesServoFL)/(t-ot);
  //    we3 = (anglesServoBR - oanglesServoBR)/(t-ot);
}
