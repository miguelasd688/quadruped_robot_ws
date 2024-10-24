#include <SimpleKalmanFilter.h>


/*---------------ANALOG PIN DEFINITION------------------------*/
int me1 = 15; //FR coxa
int me2 = 16; // f
int me3 = 17; // t

int me4 = 39;//FL
int me5 = 38;
int me6 = 26;

int me7 = 20;//BR
int me8 = 21;
int me9 = 22;

int me10 = 14;//BL
int me11 = 41;
int me12 = 40;

int me13 = 23;//BODY


int me1data = 500000;
int me2data = 500000;
int me3data = 500000;
int me4data = 500000;
int me5data = 500000;
int me6data = 500000;
int me7data = 500000;
int me8data = 500000;
int me9data = 500000;
int me10data = 500000;
int me11data = 500000;
int me12data = 500000;
int me13data = 500000;


/*
  SimpleKalmanFilter(e_mea, e_est, q);
  e_mea: Measurement Uncertainty
  e_est: Estimation Uncertainty
  q: Process Noise
*/
float e_mea = 100;
float q = 1;

SimpleKalmanFilter KFilter1(e_mea, e_mea, q);
SimpleKalmanFilter KFilter2(e_mea, e_mea, q);
SimpleKalmanFilter KFilter3(e_mea, e_mea, q);
SimpleKalmanFilter KFilter4(e_mea, e_mea, q);
SimpleKalmanFilter KFilter5(e_mea, e_mea, q);
SimpleKalmanFilter KFilter6(e_mea, e_mea, q);
SimpleKalmanFilter KFilter7(e_mea, e_mea, q);
SimpleKalmanFilter KFilter8(e_mea, e_mea, q);
SimpleKalmanFilter KFilter9(e_mea, e_mea, q);
SimpleKalmanFilter KFilter10(e_mea, e_mea, q);
SimpleKalmanFilter KFilter11(e_mea, e_mea, q);
SimpleKalmanFilter KFilter12(e_mea, e_mea, q);
SimpleKalmanFilter KFilter13(e_mea, e_mea, q);



void readRawEncoders() {

  me1data = analogRead(me1);
  me2data = analogRead(me2);
  me3data = analogRead(me3);
  me4data = analogRead(me4);
  me5data = analogRead(me5);
  me6data = analogRead(me6);
  me7data = analogRead(me7);
  me8data = analogRead(me8);
  me9data = analogRead(me9);
  me10data = analogRead(me10);
  me11data = analogRead(me11);
  me12data = analogRead(me12);
  me13data = analogRead(me13);

}
void readEncoders() {
  
  readRawEncoders();
  
  enc1 = KFilter1.updateEstimate(me1data);
  enc2 = KFilter2.updateEstimate(me2data);
  enc3 = KFilter3.updateEstimate(me3data);
  enc4 = KFilter4.updateEstimate(me4data);
  enc5 = KFilter5.updateEstimate(me5data);
  enc6 = KFilter6.updateEstimate(me6data);
  enc7 = KFilter7.updateEstimate(me7data);
  enc8 = KFilter8.updateEstimate(me8data);
  enc9 = KFilter9.updateEstimate(me9data);
  enc10 = KFilter10.updateEstimate(me10data);
  enc11 = KFilter11.updateEstimate(me11data);
  enc12 = KFilter12.updateEstimate(me12data);
  enc13 = KFilter13.updateEstimate(me13data);
}

void readEncoderAngles() {
  readEncoders();

  eang1 = ae[0] * enc1 + be[0];
  eang2 = ae[1] * enc2 + be[1];
  eang3 = ae[2] * enc3 + be[2];
  eang4 = ae[3] * enc4 + be[3];
  eang5 = ae[4] * enc5 + be[4];
  eang6 = ae[5] * enc6 + be[5];
  eang7 = ae[6] * enc7 + be[6];
  eang8 = ae[7] * enc8 + be[7];
  eang9 = ae[8] * enc9 + be[8];
  eang10 = ae[9] * enc10 + be[9];
  eang11 = ae[10] * enc11 + be[10];
  eang12 = ae[11] * enc12 + be[11];
  eang13 = ae[12] * enc13 + be[12];

}
