
//<0#-45#90#0#-45#90#0#-45#90#0#-45#90#90>

float lastRecv = 0;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char spaceMarker = '#';
    char rc;
    float delta;
    
    
 
    while (Serial.available() > 0 && newData == false) {
        RUN = true;
        lastRecv = millis()/1000.0;
        rc = Serial.read();
        if (rc == 'c') {
          CAL = true;
        }
//        Serial.print("serial In");
        if (recvInProgress == true) {
            if (rc != endMarker && rc != spaceMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else if (rc == spaceMarker ){
              receivedChars[ndx] = '\0';
              if (spaceCounter==0){
                //Serial.println(receivedChars);
                anglesIKFR.tetta = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==1){
                //Serial.println(receivedChars);
                anglesIKFR.alpha = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==2){
                //Serial.println(receivedChars);
                anglesIKFR.gamma = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==3){
                //Serial.println(receivedChars);
                anglesIKFL.tetta = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==4){
                //Serial.println(receivedChars);
                anglesIKFL.alpha = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==5){
                //Serial.println(receivedChars);
                anglesIKFL.gamma = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==6){
                //Seral.println(receivedChars);
                anglesIKBR.tetta = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==7){
                //Serial.println(receivedChars);
                anglesIKBR.alpha = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==8){
                //Serial.println(receivedChars);
                anglesIKBR.gamma = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==9){
                //Serial.println(receivedChars);
                anglesIKBL.tetta = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==10){
                //Serial.println(receivedChars);
                anglesIKBL.alpha = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==11){
                //Serial.println(receivedChars);
                anglesIKBL.gamma = atof(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==12){
                //Serial.println(receivedChars);
                angleBody = atof(receivedChars); // LAST ANGLE
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==13){
                //Serial.println(receivedChars);
                REST = atof(receivedChars); // REST
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==14){
                //Serial.println(receivedChars);
                CTRL1 = atof(receivedChars); // MODE 1
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==15){
                //Serial.println(receivedChars);
                CTRL2 = atof(receivedChars); // MODE 2
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==16){
                //Serial.println(receivedChars);
                KILL = atof(receivedChars); // KILL PROGRAM
                spaceCounter++;
                ndx=0;
              }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                //Serial.println(receivedChars);
                KILL = atof(receivedChars);
                recvInProgress = false;
                recvComplete = true;
                ndx = 0;
                spaceCounter=0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    delta = millis()/1000.0 - lastRecv;
    if (Serial.available() == 0 && delta >= 2){
      RUN = false;
    }
}
