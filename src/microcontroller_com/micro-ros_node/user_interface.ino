/***************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  The 1.54" TFT breakout
    ----> https://www.adafruit.com/product/3787
  The 2.0" TFT breakout
    ----> https://www.adafruit.com/product/4311
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// This Teensy3 and 4 native optimized and extended version
// requires specific pins.
// If you use the short version of the constructor and the DC
// pin is hardware CS pin, then it will be slower.

#define TFT_MISO  12
#define TFT_MOSI  11  //a12 [sda]
#define TFT_SCK   13  //a13 [scl]
#define TFT_DC   9    //    [dc]
#define TFT_CS   10
#define TFT_RST  8    //    [res]

// Note the above pins are for the SPI object.  For those Teensy boards which have
// more than one SPI object, such as T3.5, T3.6, T4 which have at SPI1 and SPI2
// LC with SPI1, look at the cards that come with the teensy or the web page
// https://www.pjrc.com/teensy/pin-1out.html to select the appropriate IO pins.
//#include <Adafruit_GFX.h>    // Core graphics library
#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
#include <SPI.h>



#define LTBLUE    0xB6DF
#define LTTEAL    0xBF5F
#define LTGREEN   0xBFF7
#define LTCYAN    0xC7FF
#define LTRED     0xFD34
#define LTMAGENTA 0xFD5F
#define LTYELLOW  0xFFF8
#define LTORANGE  0xFE73
#define LTPINK    0xFDDF
#define LTPURPLE  0xCCFF
#define LTGREY    0xE71C

#define BLUE      0x001F
#define TEAL      0x0438
#define GREEN     0x07E0
#define CYAN      0x07FF
#define RED       0xF800
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define ORANGE    0xFC00
#define PINK      0xF81F
#define PURPLE    0x8010
#define GREY      0xC618
#define WHITE     0xFFFF
#define BLACK     0x0000

#define DKBLUE    0x000D
#define DKTEAL    0x020C
#define DKGREEN   0x03E0
#define DKCYAN    0x03EF
#define DKRED     0x6000
#define DKMAGENTA 0x8008
#define DKYELLOW  0x8400
#define DKORANGE  0x8200
#define DKPINK    0x9009
#define DKPURPLE  0x4010
#define DKGREY    0x4A49

// For 1.44" and 1.8" TFT with ST7735 use
//ST7735_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);



// For 1.44" and 1.8" TFT with ST7735 use
ST7735_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_RST);



boolean alarm = false;
boolean oalarm = true;
boolean oRUN = true;
boolean oCTRL1 = false;
boolean oCTRL2 = false;
boolean oREST = false;
boolean oSAFE = false;
boolean displayEN = true;
boolean LCDen = true;

int timePlot = 6;
int maxValues = 30;
int values[30];
int ovalues[30];

int xini = 30;
int yini = 20;
int w = 95;
int h = 50;

int count = 0;
int oPAGE = -1;
int blinkInterval = 200;
int blinkInterval1 = 200;
unsigned long previousWarn;
unsigned long previousWarn1;
long blk1 = 0;
long blk = 0;


float oyaw = 0;
float opitch = 0;
float oroll = 0;
float dyaw = 0;
float dpitch = 0;
float droll = 0;
float odyaw = 0;
float odpitch = 0;
float odroll = 0;
float ocoxa = 0;
float ofemur = 0;
float otibia = 0;
float old_publisher_latency = 0;

float ndivX = 7;
float ndivY = 4;
float oVin = 0;
float previousPlot = 0;

float h1 = 83;
float h2 = 58;
float bufferX = 0;

String tittleCtrl1 = "   PID";
String tittleCtrl2 = "Tittle ";

float t = 0.0;
float ot = t;

long lastPrint = 0;

void setupDisplay() {
  // OR use this initializer (uncomment) if using a 240x240 clone
  // that does not have a CS pin2.0" 320x240 TFT:
  // tft.init(240, 240, SPI_MODE2);           // Init ST7789 240x240 no CS
  // Or use this initializer (uncomment) if you're using a 1.44" TFT (128x128)
  tft.initR(INITR_144GREENTAB);

  tft.fillScreen(BLACK);
  tft.setRotation(3);
  oPAGE = PAGE + 1;
}


void selectDisplayPage() {
  /*----------------SELECT SCREEN MODE---------------*/
  if (digitalRead(pinPush) == LOW) {
    Push = true;
    if (Push == true && oPush == false) { // Change page if button is pressed
      PAGE++;
      pressAt = millis();
      if (PAGE > maxPages) { //MAX PAGES
        PAGE = 0;
      }
    }
    oPush = Push;

    if (millis() - pressAt >= 2000) { // If button is pressed for more than 2 sec, turn off LCD flag
      LCDen = false;
    }
  }
  else {
    Push = false;
    oPush = Push;
  }
}

/*-----------------------UPDATE SCREEN ------------------------*/
void printDisplayLCD() {
  if (millis() - lastPrint >= 20)
  {
    lastPrint = millis();
    //sending 'c' to serial console will access to calibration protocol program.
    bool CAL = false;
    if  (CAL == false) {// END OF CALIBRATION
      if (LCDen == true) {
        displayUpdate();
      }
      else {
        pauseDisplay();
      }
    }
    else {
      calibrationDisplay();
      //calibrationSecuence();
    }
  }
}


void calibrationDisplay() {
  tft.setTextWrap(false);
  //tft.fillRect(30, 30, 150, 150 , RED);
  tft.fillScreen(TEAL);
  tft.setCursor(35, 20);
  tft.setTextColor(ST7735_BLACK);
  tft.setTextSize(2);
  tft.println("DOING");
  tft.setCursor(35, 50);
  tft.println("SERVO");
  tft.setCursor(1, 80);
  tft.println("CALIBRATION");
}

void pauseDisplay() {

  if (displayEN == true) {
    displayEN = false;
    tft.setTextWrap(false);
    delay(10);
    //tft.fillRect(30, 30, 150, 150 , RED);
    tft.fillScreen(BLACK);

    tft.fillTriangle(64 , 22 , 20 , 96 , 108 , 96 , YELLOW);
    //    tft.setCursor(70, 50);
    //    tft.setTextColor(ST7735_BLACK);
    //    tft.setTextSize(3);
    //    tft.println("DOING");
    //    tft.setCursor(60, 100);
    //    tft.println("STATIC");
    //    tft.setCursor(20, 150);
    //    tft.println("CALIBRATION");
  }


}

void initAlerts() {
  /*------------init alerts------------*/


  if (Vin > 7) {
    alarm = false;
    oalarm = true;
  }
  else if (Vin <= 7) {
    alarm = true;
    oalarm = false;
  }
  tft.setTextSize(1.5);

  if (RUN == false) {
    tft.fillRoundRect(4, 108, 58, 17, 4 , DKGREEN);
    tft.setTextColor(DKRED);
    tft.setCursor(7, 113);
    tft.println("AGENT OFF"); 
  }
  else {
    tft.fillRoundRect(4, 108, 58, 17, 4 , GREEN);
    tft.setTextColor(RED);
    tft.setCursor(7, 113);
    tft.setTextSize(1.5);
    tft.println("AGENT ON");
  }
  tft.fillRoundRect(66, 108, 58, 17, 4 , RED);
  tft.setTextColor(DKRED);
  tft.setCursor(72, 113);
  tft.println("LOW BAT");

}


/*---------------------- Redraw STATIC page elements -------------------------------*/



void startDisplay(int PAGE) {


  tft.fillScreen(BLACK);

  if (PAGE == 0) { //------->PAGE 0
    tft.setTextSize(1.5);

    tft.setCursor(5, 5);
    tft.setTextColor(LTBLUE);
    tft.print("-Running: ");
    tft.setCursor(100, 5);
    tft.print("s.");

    tft.setCursor(5, 15);
    tft.print("-Pub timer: ");
    tft.setCursor(100, 15);
    tft.print("ms.");

    tft.setCursor(5, 25);
    tft.print("-Bat Volt: ");
    tft.setCursor(100, 25);
    tft.print("V.");

    tft.setCursor(5, 45);
    tft.print("-CtrlMode: ");

    tft.fillRoundRect(4, h2, 58, 17, 4 , DKBLUE);
    tft.setTextColor(TEAL);
    tft.setCursor(7, h2 + 5);
    tft.println(tittleCtrl1);

    tft.fillRoundRect(66, h2, 58, 17, 4 , DKBLUE);
    tft.setTextColor(TEAL);
    tft.setCursor(74, h2 + 5);
    tft.print(tittleCtrl2);

    tft.fillRoundRect(4, h1, 58, 17, 4 , DKBLUE);
    tft.setTextColor(TEAL);
    tft.setCursor(6, h1 + 5);
    tft.print("  REST");

    if ( SAFE == false) {
      tft.fillRoundRect(66, h1, 58, 17, 4 , DKBLUE);
    }
    else if (SAFE == true) {
      tft.fillRoundRect(66, h1, 58, 17, 4 , LTBLUE);
    }
    tft.setTextColor(TEAL);
    tft.setCursor(74, h1 + 5);
    tft.print("  SAFE");
    int l = 4;
    tft.drawLine(64, 128 , 64, h2 - l, WHITE);
    tft.drawLine(1, h2 - l , 127, h2 - l, WHITE);
    tft.drawLine(1, h1 - l , 127, h1 - l, WHITE);
    tft.drawLine(1, h1 - l + 25 , 127, h1 - l + 25, WHITE);
    initAlerts();

  }
  else if (PAGE == 1) { //------->PAGE 1
    tft.setTextSize(1.6);

    tft.setTextColor(LTBLUE);
    tft.setCursor(2, 15);
    tft.print("orn");
    tft.setTextSize(1.5);
    tft.drawLine(24, 8, 24, 28, LTYELLOW);
    tft.drawLine(25, 8, 25, 28, LTYELLOW);
    tft.drawLine(26, 8, 26, 28, LTYELLOW);

    tft.setCursor(26, 5);
    tft.setTextColor(LTYELLOW);
    tft.print("- ");
    tft.setTextColor(BLUE);
    tft.println("yaw:");

    tft.setCursor(26, 15);
    tft.setTextColor(LTYELLOW);
    tft.print("- ");
    tft.setTextColor(BLUE);
    tft.println("pitch:");

    tft.setCursor(26, 25);
    tft.setTextColor(LTYELLOW);
    tft.print("- ");
    tft.setTextColor(BLUE);
    tft.println("roll:");
    /*------------draw circle------------*/

    tft.fillCircle(60, 65, 31, BLUE);
    tft.fillCircle(60, 65, 29, LTYELLOW);
    tft.fillCircle(60, 65, 27, BLUE);
    tft.fillCircle(60, 65, 25, LTBLUE);

    //    tft.fillCircle(120,130,51,BLUE);
    //    tft.drawCircle(120,130,52,BLUE);
    //    tft.drawCircle(120,130,51,BLUE);
    //    tft.drawCircle(120,130,52,LTYELLOW);
    //    tft.drawCircle(120,130,53,LTYELLOW);
    //    tft.drawCircle(120,130,54,LTYELLOW);
    //    tft.drawCircle(120,130,55,LTYELLOW);
    //    tft.drawCircle(120,130,56,LTYELLOW);
    //    tft.drawCircle(120,130,57,BLUE);
    //    tft.drawCircle(120,130,57.5,BLUE);
    //    tft.drawCircle(120,130,58,BLUE);
    initAlerts();

  }
  else if (PAGE == 2) { //------->PAGE 2
    //    tft.setTextSize(2);
    //
    //    tft.setTextColor(LTBLUE);
    //    tft.setCursor(5, 25);
    //    tft.print("orn");
    //    tft.drawLine(45,5,45,60,LTYELLOW);
    //    tft.drawLine(44,5,44,60,LTYELLOW);
    //    tft.drawLine(43,5,43,60,LTYELLOW);
    //    tft.setTextColor(BLUE);
    //    tft.setCursor(50, 5);
    //    tft.println("yaw:");
    //    tft.setCursor(50, 25);
    //    tft.println("pitch:");
    //    tft.setCursor(50, 45);
    //    tft.println("roll:");

    /*------------draw circle------------*/

    tft.fillRect(5, 1,       30, 44, DKCYAN);
    tft.fillRect(5, 1 + 50,  30, 44, DKCYAN);
    tft.fillRect(93, 1,      30, 44, DKCYAN);
    tft.fillRect(93, 1 + 50, 30, 44, DKCYAN);

    tft.fillRect(10, 10,      20, 30, GREEN);
    tft.fillRect(10, 10 + 50, 20, 30, GREEN);
    tft.fillRect(98, 10,      20, 30, GREEN);
    tft.fillRect(98, 10 + 50, 20, 30, GREEN);
    for ( int i = 0 ; i <= 3 ; i++ ) {
      tft.drawLine(30, 40 - i * 10 , 35, 40 - i * 10 , WHITE);
      tft.drawLine(93, 40 - i * 10 , 98, 40 - i * 10 , WHITE);
      tft.drawLine(30, 90 - i * 10 , 35, 90 - i * 10 , WHITE);
      tft.drawLine(93, 90 - i * 10 , 98, 90 - i * 10 , WHITE);

    }
    tft.setTextSize(0.5);
    tft.setTextColor(WHITE);
    tft.setCursor(15, 2);
    tft.print("BR");
    tft.setCursor(15, 52);
    tft.print("FR");
    tft.setCursor(103, 2);
    tft.print("BL");
    tft.setCursor(103, 52);
    tft.print("FL");



    //    tft.fillRect(40, 20, 160, 160, LTBLUE);
    //    tft.fillRect(40, 20, 160, 15, TEAL);
    //    tft.fillRect(40, 100, 160, 80, GREEN);
    //    tft.fillRect(40, 165, 160, 15, DKGREEN);
    //
    //    tft.drawCircle(120, 98, 60, WHITE);
    //    tft.drawCircle(120, 98, 61, WHITE);
    //    tft.drawLine(41, 100, 199, 100, WHITE);
    //    tft.drawLine(100, 100 + 15, 140, 100 + 15, WHITE);
    //    tft.drawLine(90, 100 + 30, 150, 100 + 30, WHITE);
    //    tft.drawLine(100, 100 + 45, 140, 100 + 45, WHITE);
    //    tft.drawLine(100, 100 - 15, 140, 100 - 15, WHITE);
    //    tft.drawLine(90, 100 - 30, 150, 100 - 30, WHITE);
    //    tft.drawLine(100, 100 - 45, 140, 100 - 45, WHITE);
    //
    //    //    tft.fillCircle(120,130,51,BLUE);
    //    //    tft.drawCircle(120,130,52,BLUE);
    //    //    tft.drawCircle(120,130,51,BLUE);
    //    //    tft.drawCircle(120,130,52,LTYELLOW);
    //    //    tft.drawCircle(120,130,53,LTYELLOW);
    //    //    tft.drawCircle(120,130,54,LTYELLOW);
    //    //    tft.drawCircle(120,130,55,LTYELLOW);
    //    //    tft.drawCircle(120,130,56,LTYELLOW);
    //    //    tft.drawCircle(120,130,57,BLUE);
    //    //    tft.drawCircle(120,130,57.5,BLUE);
    //    //    tft.drawCircle(120,130,58,BLUE);
    initAlerts();

  }
  else if (PAGE == 3) { //------->PAGE 3
    tft.fillScreen(BLACK);
    startPlot(xini, yini, w, h, ndivX , ndivY, "Voltage [V]");
    initAlerts();

  }
  else if (PAGE == 4)
  {
    tft.setTextSize(1.6);

    tft.setTextColor(LTBLUE);
    tft.setCursor(2, 15);
    tft.print("FR");
    tft.setTextSize(1.5);
    tft.drawLine(22, 8, 22, 28, LTYELLOW);
    tft.drawLine(23, 8, 23, 28, LTYELLOW);
    tft.drawLine(24, 8, 24, 28, LTYELLOW);

    tft.setCursor(24, 5);
    tft.setTextColor(LTYELLOW);
    tft.print("- ");
    tft.setTextColor(BLUE);
    tft.println("coxa:");

    tft.setCursor(24, 15);
    tft.setTextColor(LTYELLOW);
    tft.print("- ");
    tft.setTextColor(BLUE);
    tft.println("femur:");

    tft.setCursor(24, 25);
    tft.setTextColor(LTYELLOW);
    tft.print("- ");
    tft.setTextColor(BLUE);
    tft.println("tibia:");

    initAlerts();
  }


}



float ofe1 = 0;
float ofe2 = 0;
float ofe3 = 0;
float ofe4 = 0;

void displayUpdate() {


  /*---------parametros debug-------*/
  //  if (count <= 80 ) {
  //    CTRL = true;
  //    DYN = true;
  //    //    REST = true;
  //    //    SAFE = true;
  //  }
  //  else if (count > 80) {
  //    CTRL = false;
  //    DYN = false;
  //    //    REST = false;
  //    //    SAFE = false;
  //  }
  //  if (count >= 160) {
  //    count = 0;
  //  }
  //
  //
  //  count++;
  /*------------------------------------- */
  t = float(millis())/1000.0;
  alerts(); //Check battery and communication

  if (PAGE == 0) {
    if (oPAGE != PAGE) {
      startDisplay(PAGE);
      oPAGE = PAGE;
    }
    infoText0();
  }
  else if (PAGE == 1) {
    if (oPAGE != PAGE) {
      startDisplay(PAGE);
      oPAGE = PAGE;
    }
    InfoTextIMU();

    dyaw = deg2rad(imuSensor.GetYaw());
    dpitch = deg2rad(imuSensor.GetPitch());
    droll = deg2rad(imuSensor.GetRoll());

    if (abs(odyaw / dyaw) >= 0.1) {
      tft.drawLine(60, 65, 60 + 12 * sin(odyaw), 65 + 12 * cos(odyaw), LTBLUE);
    }
    if (abs(odpitch / dpitch) >= 0.1) {
      tft.drawLine(60, 65, 60 + 15 * sin(odroll), 65 + 15 * sin(odpitch), LTBLUE);
    }
    tft.drawLine(60, 65, 60 + 12 * sin(dyaw), 65 + 12 * cos(dyaw), DKGREEN);
    tft.drawLine(60, 65, 60 + 15 * sin(droll), 65 + 15 * sin(dpitch), MAGENTA);


    odyaw = dyaw;
    odpitch = dpitch;
    odroll = droll;
  }
  else if (PAGE == 2) {
    if (oPAGE != PAGE) {
      startDisplay(PAGE);
      oPAGE = PAGE;
    }


    if (fe3 != ofe3) {
      if (fe3 * 4 <= 30) {
        if (fe3 > ofe3) {
          tft.fillRect(10, 40 - fe3 * 4, 20, fe3 * 4, RED);
        }
        if (fe3 < ofe3) {
          tft.fillRect(10, 10, 20, 30 - fe3 * 4, GREEN); //BR
        }
        ofe3 = fe3;
      }
    }
    if (fe1 != ofe1) {
      if (fe1 * 4 <= 30) {
        if (fe1 > ofe1) {
          tft.fillRect(10, 50 + 40 - fe1 * 4, 20, fe1 * 4, RED);
        }
        if (fe1 < ofe1) {
          tft.fillRect(10, 50 + 10, 20, 30 - fe1 * 4, GREEN); //FR
        }
        ofe1 = fe1;
      }
    }
    if (fe4 != ofe4) {
      if (fe4 * 4 <= 30) {
        if (fe4 > ofe4) {
          tft.fillRect(98, 40 - fe4 * 4, 20, fe4 * 4, RED);
        }
        if (fe4 < ofe4) {
          tft.fillRect(98, 10, 20, 30 - fe4 * 4, GREEN); //BL
        }
        ofe4 = fe4;
      }
    }
    if (fe2 != ofe2) {
      if (fe2 * 4 <= 30) {
        if (fe2 > ofe2) {
          tft.fillRect(98, 50 + 40 - fe2 * 4, 20, fe2 * 4, RED);
        }
        if (fe2 < ofe2) {
          tft.fillRect(98, 50 + 10, 20, 30 - fe2 * 4, GREEN); //FL
        }
        ofe2 = fe2;
      }
    }






    //    if (abs(odyaw/dyaw) >= 0.1){
    //      tft.drawLine(120,130,120 + 49*sin(odyaw),130 + 49*cos(odyaw),LTBLUE);
    //    }
    //    if (abs(odpitch/dpitch) >= 0.1){
    //      tft.drawLine(120,130,120 + 60*sin(odroll),130 + 60*sin(odpitch),LTBLUE);
    //    }
    //    tft.drawLine(120,130,120 + 49*sin(dyaw),130 + 49*cos(dyaw),DKGREEN);
    //    tft.drawLine(120,130,120 + 60*sin(droll),130 + 60*sin(dpitch),MAGENTA);
  }
  else if (PAGE == 3) {
    if (oPAGE != PAGE) {
      startDisplay(PAGE);
      oPAGE = PAGE;
    }
    if (float(millis())*1000.0 - previousPlot >= float(timePlot) / float(maxValues) - 0.016) { // every 0.03s seconds, 30 hz
      updatePlot(float(millis())*1000.0, previousPlot , Vin, timePlot , xini, yini, w, h, ndivX , ndivY, YELLOW);
      previousPlot = float(millis())*1000.0;
    }
  }
  else if (PAGE == 4)
  {
    if (oPAGE != PAGE) {
      startDisplay(PAGE);
      oPAGE = PAGE;
    }
    InfoTextAngles();
  }

  ot = t;

}


void alerts() {

  if (Vin <= 5.8) {
    tft.fillRect(103, 60 , 7 , 25 , RED);
    for (int i = 0; i <= 3; i++) {
      tft.drawRect(24 + i, 55 + i, 80 - i * 2, 35 - i * 2 , RED);
    }
    for (int i = -4; i <= 16; i++) {
      tft.drawLine(64 - i, 70 + i / 2.5 , 88, 34,  RED);
    }
    for (int i = -4; i <= 16; i++) {
      tft.drawLine(64 + i, 70 - i / 2.5 , 40, 108,  RED);
    }
  }

  if (Vin <= 6.4) {
    unsigned long currentMillisWarn = millis();
    if (currentMillisWarn - previousWarn >= blinkInterval) {
      // save the last time you blinked the LED
      previousWarn = currentMillisWarn;
      blk++;
      if (blk % 8 == 0) {
        blinkInterval = 400;
      }
      else {
        blinkInterval = 150;
      }
      if (alarm == false) {
        alarm = true;
        oalarm = false;
      }
      else {
        alarm = false;
        oalarm = true;

      }
    }
  }
  else if (Vin > 6.0) {
    alarm = false;
  }

  if (alarm != oalarm) {
    if (alarm == true) {
      tft.fillRoundRect(66, 108, 58, 17, 4 , RED);
      tft.setTextColor(YELLOW);
      tft.setCursor(72, 113);
      tft.setTextSize(1.5);
      tft.println("LOW BAT");
    }
    else {
      tft.fillRoundRect(66, 108, 58, 17, 4 , DKRED);
      tft.setTextColor(RED);
      tft.setCursor(72, 113);
      tft.setTextSize(1.5);
      tft.println("LOW BAT");
    }
    oalarm = alarm;
  }


  if (oRUN != RUN) {
    if (RUN == true) {
      tft.fillRoundRect(4, 108, 58, 17, 4 , GREEN);
      tft.setTextColor(RED);
      tft.setCursor(7, 113);
      tft.setTextSize(1.5);
      tft.println("AGENT ON");
    }
    else {
      tft.fillRoundRect(4, 108, 58, 17, 4 , DKGREEN);
      tft.setTextColor(DKRED);
      tft.setCursor(7, 113);
      tft.setTextSize(1.5);
      tft.println("AGENT OFF");
    }

    oRUN = RUN;
  }
}

void infoText0() {

  tft.setTextSize(1.5);
  tft.setCursor(64, 5);
  tft.setTextColor(BLACK);
  tft.print(ot, 1);
  tft.setCursor(64, 5);
  tft.setTextColor(MAGENTA);
  tft.print(t, 1);
  tft.setTextColor(LTBLUE);

  tft.setCursor(68, 15);
  tft.setTextColor(BLACK);
  tft.print(old_publisher_latency , 1);
  tft.setCursor(68  , 15);
  tft.setTextColor(MAGENTA);
  tft.print(publisher_latency , 1);

  if ( Vin != oVin) {
    tft.setCursor(64, 25);
    tft.setTextColor(BLACK);
    tft.print(oVin, 2);
    tft.setCursor(64, 25);
    tft.setTextColor(MAGENTA);
    tft.print(Vin, 2);
  }
  if ( CTRL1 != oCTRL1 ) {
    tft.setCursor(64, 45);
    tft.setTextColor(BLACK);
    tft.print(oCTRL1);
    tft.setCursor(64, 45);
    tft.setTextColor(MAGENTA);
    tft.print(CTRL1);
  }

  if (REST != oREST) {
    if (REST == true) {

      tft.fillRoundRect(4, h1, 58, 17, 4 , LTBLUE);
      tft.setTextColor(BLUE);
      tft.setCursor(7, h1 + 5);
      tft.print("  REST");
    }
    else {

      tft.fillRoundRect(4, h1, 58, 17, 4 , DKBLUE);
      tft.setTextColor(TEAL);
      tft.setCursor(7, h1 + 5);
      tft.print("  REST");

    }
  }
  if (SAFE != oSAFE) {
    if (SAFE == true) {

      tft.fillRoundRect(66, h1, 58, 17, 4 , LTBLUE);
      tft.setTextColor(BLUE);
      tft.setCursor(74, h1 + 5);
      tft.print("  SAFE");
    }
    else {

      tft.fillRoundRect(66, h1, 58, 17, 4 , DKBLUE);
      tft.setTextColor(TEAL);
      tft.setCursor(74, h1 + 5 );
      tft.print("  SAFE");
    }
  }
  if (CTRL1 != oCTRL1) {
    if (CTRL1 == true) {

      tft.fillRoundRect(4, h2, 58, 17, 4 , LTBLUE);
      tft.setTextColor(BLUE);
      tft.setCursor(7, h2 + 5);
      tft.print(tittleCtrl1);
    }
    else {

      tft.fillRoundRect(4, h2, 58, 17, 4 , DKBLUE);
      tft.setTextColor(TEAL);
      tft.setCursor(7, h2 + 5);
      tft.print(tittleCtrl1);
    }
  }
  if (CTRL2 != oCTRL2) {
    if (CTRL2 == true) {

      tft.fillRoundRect(66, h2, 58, 17, 4 , LTBLUE);
      tft.setTextColor(BLUE);
      tft.setCursor(74, h2 + 5);
      tft.print(tittleCtrl2);
    }
    else {

      tft.fillRoundRect(66, h2, 58, 17, 4 , DKBLUE);
      tft.setTextColor(TEAL);
      tft.setCursor(74, h2 + 5);
      tft.print(tittleCtrl2);
    }
  }

  unsigned long currentMillisWarn = millis();
  if (currentMillisWarn - previousWarn1 >= blinkInterval1) {
    // save the last time you blinked the LED
    previousWarn1 = currentMillisWarn;
    blk1++;
    if (blk1 <= 2) {
      if (SAFE == true) {

        tft.drawRoundRect(66, h1, 58, 17, 4 , LTBLUE);
        tft.setTextColor(BLUE);
        tft.setCursor(74, h1 + 5);
        tft.print("  SAFE");
      }
      else {
        tft.drawRoundRect(66, h1, 58, 17, 4 , DKBLUE);
        tft.setTextColor(TEAL);
        tft.setCursor(74, h1 + 5 );
        tft.print("  SAFE");
      }

    }
    else if (blk1 <= 4) {

      if (SAFE == true) {

        tft.drawRoundRect(66, h1, 58, 17, 4 , GREEN);
        tft.setTextColor(DKGREEN);
        tft.setCursor(74, h1 + 5);
        tft.print("  SAFE");
      }
      else {
        tft.drawRoundRect(66, h1, 58, 17, 4 , RED);
        tft.setTextColor(RED);
        tft.setCursor(74, h1 + 5 );
        tft.print("  SAFE");
      }
    }
    else if (blk1 >= 4) {
      blk1 = 0;
    }
  }

  oVin = Vin;
  old_publisher_latency = publisher_latency;
  oCTRL1 = CTRL1;
  oCTRL2 = CTRL2;
  oREST = REST;
  oSAFE = SAFE;
}

void InfoTextIMU() {

  tft.setTextSize(1.5);
  tft.setTextColor(BLACK);
  tft.setCursor(70, 5);
  tft.print(oyaw);
  tft.setCursor(70, 15);
  tft.print(opitch);
  tft.setCursor(70, 25);
  tft.print(oroll);
  tft.setTextColor(MAGENTA);
  tft.setCursor(70, 5);
  tft.print(imuSensor.GetYaw());
  tft.setCursor(70, 15);
  tft.print(imuSensor.GetPitch());
  tft.setCursor(70, 25);
  tft.print(imuSensor.GetRoll());

  oyaw = imuSensor.GetYaw();
  opitch = imuSensor.GetPitch();
  oroll = imuSensor.GetRoll();
}

void InfoTextAngles() {

  tft.setTextSize(1.5);
  tft.setTextColor(BLACK);
  tft.setCursor(70, 5);
  tft.print(ocoxa);
  tft.setCursor(70, 15);
  tft.print(ofemur);
  tft.setCursor(70, 25);
  tft.print(otibia);
  tft.setTextColor(MAGENTA);
  tft.setCursor(70, 5);
  tft.print(anglesIKFR.tetta);
  tft.setCursor(70, 15);
  tft.print(anglesIKFR.alpha);
  tft.setCursor(70, 25);
  tft.print(anglesIKFR.gamma);

  ocoxa = anglesIKFR.tetta;
  ofemur = anglesIKFR.alpha;
  otibia = anglesIKFR.gamma;
}


void startPlot( int x0, int y0, int w, int h, float ndivX , float ndivY , String voltage) {
  float tempx , tempy;
  float llimY = 5.0;
  float hlimY = 9.0;



  //horizontal lines
  tempy = h / ndivY;//num of Y axis divitions
  tempx = w / ndivX;//num of X axis divitions

  tft.drawLine(x0, y0 + tempy * 0., x0 + w - 1, y0 + tempy * 0., BLUE);
  tft.drawLine(x0, y0 + tempy * 1., x0 + w - 1, y0 + tempy * 1., BLUE);
  tft.drawLine(x0, y0 + tempy * 2., x0 + w - 1, y0 + tempy * 2., BLUE);
  tft.drawLine(x0, y0 + tempy * 3., x0 + w - 1, y0 + tempy * 3., BLUE);
  tft.drawLine(x0, y0 + tempy * 4., x0 + w - 1, y0 + tempy * 4., RED);
  //vertical lines
  //  tft.drawLine(x0-1, y0, x0-1, y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 0., y0, x0 + tempx * 0., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 1., y0, x0 + tempx * 1., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 2., y0, x0 + tempx * 2., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 3., y0, x0 + tempx * 3., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 4., y0, x0 + tempx * 4., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 5., y0, x0 + tempx * 5., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 6., y0, x0 + tempx * 6., y0 + h - 1, BLUE);
  tft.drawLine(x0 + tempx * 7., y0, x0 + tempx * 7., y0 + h - 1, BLUE);

  //Y label
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.setCursor(5, y0 - 15);
  tft.print(voltage);

  //Y axis
  tft.setTextColor(WHITE);
  tft.setCursor(5, y0 - 3);
  tft.print("9.00");
  tft.setCursor(5, y0 - 4 + tempy);
  tft.print("8.00");
  tft.setCursor(5, y0 - 4 + tempy * 2);
  tft.print("7.00");
  tft.setCursor(5, y0 - 4 + tempy * 3);
  tft.print("6.00");
  tft.setCursor(5, y0 - 4 + tempy * 4);
  tft.print("5.00");
  // X label
  tft.setCursor(17, y0 + h + 6);
  tft.print("-7 sec");
  tft.setTextColor(RED);
  tft.print("  ~Time:");
}

void updatePlot(float t, float ot, float Vin, int Xtime , int x0, int y0, int w, int h, float ndivX , float ndivY , unsigned int plotcolor) {
  int delta , odelta;
  int deltaX , odeltaX;
  int loopt , oloopt;
  int tempx;
  int tempy;
  int y, oy;
  int i;
  int intervalX;

  float lhlimY = 5.0;
  float hlimY = 9.0;

  loopt = map(t, 0, Xtime, 0, w);
  oloopt = map(ot, 0, Xtime, 0, w);

  tempx = w / ndivX;
  tempy = h / ndivY;

  delta = loopt % tempx;
  odelta = oloopt % tempx;
  deltaX = loopt % w;
  odeltaX = oloopt % w;

  if (Vin <= lhlimY) {
    Vin = lhlimY;
  }
  y = map(Vin, hlimY, lhlimY, h, 0);
  oy = map(oVin, hlimY, lhlimY, h, 0);

  oVin = Vin;


  //  if (bufferX <= maxValues) {
  //    if (t >= (Xtime / float(maxValues))*bufferX) {
  //      for (i = maxValues; i >= int(bufferX); i--) {
  //        values[i] = y;
  //      }
  //
  //      bufferX++;
  //    }
  //  }
  //  else {
  if (int(t) % (Xtime / maxValues)  >= (Xtime / float(maxValues))) {
    for (i = maxValues; i >= 1; i--) {
      values[i] = values[i - 1];
      ovalues[i] = ovalues[i - 1];
    }
    values[0] = y;
  }
  //  }


  //clean all lines
  tft.fillRect(x0 + 1, y0 + 1, w, h + 1 , BLACK);
  tft.setCursor(x0 + w - 16, y0 + h + 6);
  tft.setTextColor(BLACK);
  tft.print(int(ot));


  //draw next step time
  tft.setCursor(x0 + w - 16, y0 + h + 6);
  tft.setTextColor(WHITE);
  tft.print(int(t));

  tft.drawLine(x0 + w, y0, x0 + w, y0 + h , BLUE);
  tft.drawLine(x0 - delta + tempx * 1., y0, x0 - delta + tempx * 1., y0 + h - 1, BLUE);
  tft.drawLine(x0 - delta + tempx * 2., y0, x0 - delta + tempx * 2., y0 + h - 1, BLUE);
  tft.drawLine(x0 - delta + tempx * 3., y0, x0 - delta + tempx * 3., y0 + h - 1, BLUE);
  tft.drawLine(x0 - delta + tempx * 4., y0, x0 - delta + tempx * 4., y0 + h - 1, BLUE);
  tft.drawLine(x0 - delta + tempx * 5., y0, x0 - delta + tempx * 5., y0 + h - 1, BLUE);
  tft.drawLine(x0 - delta + tempx * 6., y0, x0 - delta + tempx * 6., y0 + h - 1, BLUE);
  tft.drawLine(x0 - delta + tempx * 7., y0, x0 - delta + tempx * 7., y0 + h - 1, BLUE);
  //redraw horizontal lines
  //  tft.drawLine(x0, y0, x0 + w - 1, y0 , BLUE);
  tft.drawLine(x0, y0 + tempy * 1., x0 + w - 1, y0 + tempy * 1. , GREEN);
  tft.drawLine(x0, y0 + tempy * 2., x0 + w - 1, y0 + tempy * 2., BLUE);
  tft.drawLine(x0, y0 + tempy * 3., x0 + w - 1, y0 + tempy * 3., RED);
  tft.drawLine(x0, y0 + tempy * 4. + 2, x0 + w - 1, y0 + tempy * 4. + 2 , BLUE);
  //  tft.drawLine(x0, y0 + h, x0 + w - 1, y0 + h , BLUE);

  for (i = 0; i <= maxValues; i++) {
    intervalX = w / float(maxValues);
    tft.drawCircle(x0 + w - 2 - intervalX * i, y0 + h - values[i], 1 , plotcolor);
  }

}
