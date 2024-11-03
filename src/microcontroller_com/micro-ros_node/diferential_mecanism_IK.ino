struct vector anglesIKtmp;

struct vector servoOffset;



float deg2rad(float deg){
  float rad;
  rad = deg* 2 * PI / 360.0;
  return rad;
}
float rad2deg(float rad){
  float deg;
  deg = rad* 360.0 / (2 * PI);
  return deg;
}



struct vector calculateIKangles(struct vector anglesIK) {
  float l , c1 , c2;

  float legLenght = 90.0;
  float axisOffset = 29.0;
  
  float tibiaLever = 24.0;//20.5; //lenght of servo horn has two positions
  float tibiaTendon = 24.0; 
  
  float femurLever = 24.0; 
  float femurTendon = 24.0;
  struct vector anglesServo;

  servoOffset.tetta = 90;
  servoOffset.alpha = 45;
  servoOffset.gamma = 45;
  
  anglesIKtmp.alpha = abs(anglesIK.alpha);
  anglesIKtmp.gamma = abs(anglesIK.gamma);
  
  anglesServo.tetta = anglesIK.tetta + servoOffset.tetta;
    
  l = sqrt(sq(legLenght) + sq(femurTendon) - 2.0*femurTendon*legLenght*cos(deg2rad(anglesIKtmp.gamma)));
  c1 = rad2deg(asin(femurTendon*sin(deg2rad(anglesIKtmp.gamma))/l));
  c2 = rad2deg(acos((sq(femurLever) + sq(l) - sq(legLenght))/(2.0*femurLever*l)));

  anglesServo.gamma = 180.0 - anglesIKtmp.alpha - c1 -c2 + servoOffset.gamma;
  anglesServo.gamma = - anglesServo.gamma + 180.0; //change servo sign
  
  l = sqrt(sq(axisOffset) + sq(femurTendon) - 2*femurTendon*axisOffset*cos(deg2rad(anglesIKtmp.alpha + 45)));
  c1 = rad2deg(asin(femurTendon*sin(deg2rad(anglesIKtmp.alpha + 45))/l));
  c2 = rad2deg(acos((sq(femurLever) + sq(l) - sq(axisOffset))/(2*femurLever*l)));

  anglesServo.alpha = 180.0 - 45.0 - c1 - c2 + servoOffset.alpha;
  
  return anglesServo;
}
