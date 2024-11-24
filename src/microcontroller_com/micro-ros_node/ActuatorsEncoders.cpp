#include "ActuatorsEncoders.h"

ActuatorsEncoders::ActuatorsEncoders() :  analog1(encoder_pin[0], true),
                                          analog2(encoder_pin[1], true),
                                          analog3(encoder_pin[2], true),
                                          analog4(encoder_pin[3], true),
                                          analog5(encoder_pin[4], true),
                                          analog6(encoder_pin[5], true),
                                          analog7(encoder_pin[6], true),
                                          analog8(encoder_pin[7], true),
                                          analog9(encoder_pin[8], true),
                                          analog10(encoder_pin[9], true),
                                          analog11(encoder_pin[10], true),
                                          analog12(encoder_pin[11], true),
                                          KFilter1(e_mea, e_mea, q),
                                          KFilter2(e_mea, e_mea, q),
                                          KFilter3(e_mea, e_mea, q),
                                          KFilter4(e_mea, e_mea, q),
                                          KFilter5(e_mea, e_mea, q),
                                          KFilter6(e_mea, e_mea, q),
                                          KFilter7(e_mea, e_mea, q),
                                          KFilter8(e_mea, e_mea, q),
                                          KFilter9(e_mea, e_mea, q),
                                          KFilter10(e_mea, e_mea, q),
                                          KFilter11(e_mea, e_mea, q),
                                          KFilter12(e_mea, e_mea, q) {}

void ActuatorsEncoders::ReadRawEncoders() {
  //for (int i = 0; i < 12; i++)
  //{
  //  anglesEncoders.asArray[i] = analogRead(encoder_pin[i]);
  //}
  analog1.update();
  analog2.update();
  analog3.update();
  analog4.update();
  analog5.update();
  analog6.update();
  analog7.update();
  analog8.update();
  analog9.update();
  analog10.update();
  analog11.update();
  analog12.update();

  anglesEncoders.asArray[0] = analog1.getValue();
  anglesEncoders.asArray[1] = analog2.getValue();
  anglesEncoders.asArray[2] = analog3.getValue();
  anglesEncoders.asArray[3] = analog4.getValue();
  anglesEncoders.asArray[4] = analog5.getValue();
  anglesEncoders.asArray[5] = analog6.getValue();
  anglesEncoders.asArray[6] = analog7.getValue();
  anglesEncoders.asArray[7] = analog8.getValue();
  anglesEncoders.asArray[8] = analog9.getValue();
  anglesEncoders.asArray[9] = analog10.getValue();
  anglesEncoders.asArray[10] = analog11.getValue();
  anglesEncoders.asArray[11] = analog12.getValue();
}

void ActuatorsEncoders::ReadEncoders() {
  ActuatorsEncoders::ReadRawEncoders();
  
  enc1 = KFilter1.updateEstimate(anglesEncoders.asArray[0]);
  enc2 = KFilter2.updateEstimate(anglesEncoders.asArray[1]);
  enc3 = KFilter3.updateEstimate(anglesEncoders.asArray[2]);
  enc4 = KFilter4.updateEstimate(anglesEncoders.asArray[3]);
  enc5 = KFilter5.updateEstimate(anglesEncoders.asArray[4]);
  enc6 = KFilter6.updateEstimate(anglesEncoders.asArray[5]);
  enc7 = KFilter7.updateEstimate(anglesEncoders.asArray[6]);
  enc8 = KFilter8.updateEstimate(anglesEncoders.asArray[7]);
  enc9 = KFilter9.updateEstimate(anglesEncoders.asArray[8]);
  enc10 = KFilter10.updateEstimate(anglesEncoders.asArray[9]);
  enc11 = KFilter11.updateEstimate(anglesEncoders.asArray[10]);
  enc12 = KFilter12.updateEstimate(anglesEncoders.asArray[11]);
}

void ActuatorsEncoders::ReadEncoderAngles() {
  ActuatorsEncoders::ReadEncoders();

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
}

float ActuatorsEncoders::GetEncoderRawValue(int i)
{
  return anglesEncoders.asArray[i];
}