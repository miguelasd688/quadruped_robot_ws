
byte bufferSerialSend[] = {164,10,"#",7,"#",11,"#",13};
size_t bytes = sizeof(bufferSerialSend)/sizeof(bufferSerialSend[0]);

void SerialWriteRPI(){
  Serial.write(bufferSerialSend,bytes);
}

void SerialDebugANGLES() {
  /*------------------------SEND ALL DATA OVER SERIAL--------------*/
  //Serial.print("<");
  Serial.print(loopTime / 1000.,1);
  //    Serial.print("# ");
  //    Serial.print(enc1);
  //    Serial.print(" # ");
  //    Serial.print(me1data);
  //    Serial.print(" # ");
  //    Serial.print(enc2);
  //    Serial.print(" # ");
  //    Serial.print(me2data);
  //    Serial.print(" #");
  /*---------------------ACTUATOR INFO--------------------*/
  Serial.print("#");
  //    Serial.print(Vin);
  //    float ploty = 20;
  //    Serial.print("# FR : ");//----------------- leg: FR
  //    Serial.print(" FR: ");
  //    Serial.print(anglesIKFR.tetta);
  //    Serial.print("#");
  //        Serial.print(anglesServoFR.tetta);
  //        Serial.print("#");
  //        Serial.print(enc1);
  //        Serial.print(" # ");
  //
  //    Serial.print(anglesServoFR.tetta - eang1, 1);
  //    Serial.print(" # ");
//  Serial.print( fe1, 1);
//  Serial.print("#");

  //
  //    Serial.print(" f: ");
  //    Serial.print( - anglesServoFR.alpha + eang2 , 1);
  //    Serial.print(" # ");
  //    Serial.print(eang1);
  //    Serial.print(" t: ");
  //    Serial.print(eang2 + ploty);
  //    Serial.print(" # ");
  //    Serial.print(anglesServoFR.gamma + ploty*2);
  //    Serial.print(" # ");
  //    Serial.print(anglesServoFR.gamma - eang3, 1);
  //    Serial.print(" # ");

  //
  //    Serial.print(" FL: ");
  //    Serial.print(anglesIKFL.tetta);//--- leg: FL
  //    Serial.print("#");
  //            Serial.print(anglesServoFL.tetta);
  //            Serial.print("#");
  //        Serial.print(anglesServoFL.tetta - eang4, 1);
  //        Serial.print("#");
  Serial.print( fe2, 1);
  Serial.print("#");
  //
  //    Serial.print(anglesIKFL.alpha);
  //    Serial.print("#");
  //        Serial.print(anglesServoFL.alpha);
  //        Serial.print("#");
  //        Serial.print( anglesServoFL.alpha - eang5, 1);
  //        Serial.print(" # ");
  //
  //    Serial.print(anglesIKFL.gamma);
  //    Serial.print("#");
  //        Serial.print(anglesServoFL.gamma);
  //        Serial.print("#");
  //        Serial.print(anglesServoFL.gamma - eang6, 1);
  //        Serial.print(" # ");
  //    Serial.print(" BR: ");
  //    Serial.print(anglesIKBR.tetta);//--- leg: BR
  //    Serial.print("#");
  //        Serial.print(anglesServoBR.tetta);
  //        Serial.print(" # ");

  //        Serial.print(anglesServoBR.tetta - eang7, 1);
  //        Serial.print("#");
  Serial.print( fe3, 1);
  Serial.print("#");
  //
  //    Serial.print(anglesIKBR.alpha);
  //    Serial.print("#");
  //        Serial.print(anglesServoBR.alpha);
  //        Serial.print(" # ");
  //        Serial.print(anglesServoBR.alpha - eang8, 1);
  //        Serial.print("#");
  //
  //    Serial.print(anglesIKBR.gamma);
  //    Serial.print("#");
  //        Serial.print(anglesServoBR.gamma);
  //        Serial.print(" # ");
  //        Serial.print(anglesServoBR.gamma - eang9, 1);
  //        Serial.print("#");
  //    Serial.print(" BL: ");
  //    Serial.print(anglesIKBL.tetta);//--- leg: BL
  //    Serial.print("#");
  //        Serial.print(anglesServoBL.tetta);
  //        Serial.print(" # ");

  //    Serial.print(oanglesServoBL.tetta - eang10, 1);
  //    Serial.print(" # ");
  //    Serial.print(anglesServoBL.alpha - eang11, 1);
  //    Serial.print(" # ");
  Serial.println(fe4, 1);
  //    Serial.print("#");
  //
  //    Serial.print(anglesIKBL.alpha);
  //    Serial.print("#");
  //        Serial.print(anglesServoBL.alpha);
  //        Serial.print(" # ");
  //        Serial.print(anglesServoBL.alpha - eang11, 1);
  //        Serial.print("#");
  //
  //    Serial.print(anglesIKBL.gamma);
  //    Serial.print("#");
  //        Serial.print(anglesServoBL.gamma);
  //        Serial.print(" # ");
  //        Serial.print(anglesServoBL.gamma - eang12, 1);
  //        Serial.print("#");
  //    Serial.print(angleBody - eang13, 1);

  //    Serial.print(eang13);
  //    Serial.print("#");
  /*----------------IMU------------*/
  //        Serial.print("#");
  //        Serial.print(yaw, 2);
  //        Serial.print("#");
  //        Serial.print(pitch, 2);
  //        Serial.print("#");
  //        Serial.print(roll, 2);
  //        Serial.print("#");
  //        Serial.print(Xacc, 1);
  //        Serial.print("#");
  //        Serial.print(Yacc, 1);
  //        Serial.print("#");
  //        Serial.print(Zacc, 1);
  //        Serial.print("#");
  //        Serial.print(Xrot, 1);
  //        Serial.print("#");
  //        Serial.print(Yrot, 1);
  //        Serial.print("#");
  //        Serial.print(Zrot, 1);
  //    Serial.print("#");
  //    Serial.print(accel,1);
  //    Serial.print(" # ");
  //    Serial.print(gyr,1);
  //    Serial.print(" # ");
  //    Serial.print(mg,1);
  //    Serial.print(" # ");
  //    Serial.print(boardTemp,1);

  // Serial.println(">");
}
