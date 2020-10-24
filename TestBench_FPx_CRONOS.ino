//-------------------------------------------------------------------------------------
// testBench.h
// Arduino
// Andreas Urben 2020


// Hardware:
// ARDUINO UNO R3 BOARD SMD
// https://store.arduino.cc/arduino-uno-rev3
//
// Cytron Shield MD10 R2,
// https://www.cytron.io/p-10amp-7v-30v-dc-motor-driver-shield-for-arduino
//
// 4xChannel 4-20mA Current Loop Receiver
// 16Bit ADS1115 I2C Mini Module Rev.A
// http://store.ncd.io
// https://store.ncd.io/product/4-channel-4-20-ma-current-loop-receiver-16-bit-ads1115-i2c-mini-module/
// I2C Adress: 0x48
// 6430  ->  4mA
// 32154 -> 20mA
//
// 1xChannel 4-20mA Current Loop Transmitter
// 12Bit MCP4725 I2C Mini Module Rev.B
// http://store.ncd.io
// https://store.ncd.io/product/1-channel-4-20ma-current-loop-transmitter-i2c-mini-module/
// I2C Adress: 0x60
// 670 ->  4mA
// 3460 -> 20mA
//
// Sparfunk Load Cell Amp HX711 with HBM U9C 20kN
// https://www.sparkfun.com/products/13879
// Set the value "#define SAMPLES" to 2 in the Library HX711_ADC/src/config.h
// On electronics board Open jumper SJ2 (with knife) to set to 80SPS.
//
// Nextion NX8048T070 - Generic 7.0" HMI TFT LCD Touch Display
// NX8048T070_011
// https://nextion.tech/
//
// Arduino Ethernet Shield 2
// white SD Card
// https://store.arduino.cc/arduino-ethernet-shield-2



//-------------------------------------------------------------------------------------
//  Pins ARDUINO UNO R3 BOARD:
//  0 -> RX Serial -> blue   -> TX Nextion Display
//  1 -> TX Serial -> yellow -> RX Nextion Display
//  2 ->                     -> Motor Driver PWM1 (valve 1)  (for Hydraulik Break)
//  3 ->                     -> Motor Driver PWM2 (valve 2)  (for Hydraulik Break)
//  4 ->                     -> SS SPI SD CARD
//  5 -> none                   (Temp 4-20 mA Transmitter)
//  6 ->                     -> CLK Load Cell Amp HX711 Sparfunk
//  7 ->                     -> DAT Load Cell Amp HX711 Sparfunk
//  8 ->                     -> DIR Cytron Shield MD10
//  9 ->                     -> PWM Cytron Shield MD10
// 10 ->                     -> SS   SPI Ethernet Shield 2
// 11 ->                     -> MOSI SPI Ethernet Shield 2
// 12 ->                     -> MISO SPI Ethernet Shield 2
// 13 ->                     -> SCK  SPI Ethernet Shield 2
//
//
// A4 -> SDA I2C Shield      -> SDA 4-20mA Reciver (0x48) -> SDA 4-20mA Transmitter (0x60)
// A5 -> SCL I2C Shield      -> SCL 4-20mA Reciver (0x48) -> SCL 4-20mA Transmitter (0x60)

// #include <ADS1115.h>  //ncd
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1015.h> //adafruit
#include <HX711_ADC.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;    // SD Card SPI SS


// Break Type Selection
// 1 = Electrical Break
// 2 = Hydraulic Break
// const int breakeType = 2;

// int midValue = 2056;
const  int midValue = 2065-35; // -15             //-5 nach link langsamer //-18  //-25 //+250
// int setForceState = 0;


int loadingForceF1;
int loadingForceF2;
int loadedTimeF1;
int loadedTimeF2;
int loadingForceManual=0; 
unsigned long testBenchActualCycle = 0;
unsigned long testBenchTargetCycle = 99;
unsigned long cycleDurationTime;
float cylinderPos = -99;
int testBenchMode = 99; //0=stop; 1=auto; 2=manual; 99=modeSelect
bool logState = 0; // Loging 0=off; 1=on
int logMode = 1; // 1=Full; 2=Move 

int playButtonState = 0; // 0=off; 1=play 
int stopButtonState = 0; // 0=off; 1=on;  
int valveButtonState1 = 0; // 0=off; 1=on
int valveButtonState2 = 0; // 0=off; 1=on

int logForceF1min = 0; 
int logForceF1max = 0;
int logForceF2min = 0;
int logForceF2max = 0;
float logPosF1min = 0;  
float logPosF1max = 0;
float logPosF2min = 0;
float logPosF2max = 0;

int intermediateCycleCounter = 0;

unsigned int brClosingTime;
unsigned int brOpeningTime;
unsigned int brOpenTime;
unsigned int brClosedTime;

enum STATES {
  TestBenchModeSelect = 1,
  TestBenchInit = 2,
  TestBenchInitBrakeClosing = 3,
  TestBenchInitBrakeOpening = 4,
  TestBenchInitLoadcell = 50,
  TestBenchInitCylinderPos1 = 5,
  TestBenchInitCylinderPos2 = 6,
  TestBenchInitCylinderPos3 = 7,
  TestBenchStartCycle = 8,
  BrakeOpening = 9,
  BrakeOpen = 10,
  BrakeClosing = 11,
  BrakeClosed = 12,
  BrakeForceLoadingF1 = 13,
  BrakeForceLoadedF1 = 14,
  BrakeForceLoadingF2 = 15,
  BrakeForceLoadedF2 = 16,
  BrakeForceUnloading = 17,
  BrakeForceUnloaded = 18,
  BrakeOpeningManual = 19,
  BrakeClosingManual = 20,
  BrakeForceLoadingManual = 21,
  BrakeForceUnloadingManual = 22, 
  TestBenchEndCycle = 23,
  TestBenchStop = 99
}

state = TestBenchModeSelect;
// state = TestBenchInit;
//mchar state[21];

// STATES = TestBenchModeSelect;

//---------------- Load Cell ------------------
//HX711 constructor (dout pin, sck pin):
HX711_ADC LoadCell(7, 6);
const unsigned int loadCellInterval = 70; // 125;  //250
unsigned long loadCellPreviousMillis = 0;
int loadCellForce = 0;

//------------------- Motor -------------------
const unsigned int motorInterval = 2000;
unsigned long motorPreviousMillis = 0;
unsigned long motorCurrentMillis = 0;
unsigned int motorPWM = 110;
// int motorStatus = 0;
// bool motorDirection = LOW;

//--------------- 4-20mA Reciver ---------------
//ADS1115 constructor (I2C Adress):
Adafruit_ADS1115 ads(0x48);
const unsigned int reciverInterval = 100;
unsigned long reciverPreviousMillis = 0;
// float posTransmitterMilliAmpere = 0;


//-------------- 0-5V Reciver CRONOS ------------
// imc CRONOS read output
//ADS1115 constructor (I2C Adress):
Adafruit_ADS1115 adsCRONOS(0x49);
const unsigned int reciverIntervalCRONOS = 100;
unsigned long reciverPreviousMillisCRONOS = 0;



//------------- 4-20mA Transmitter -------------
Adafruit_MCP4725 dac;
const unsigned int transmitterInterval = 250;
unsigned long transmitterPreviousMillis = 0;
// uint32_t dac_mA = 1785;


//--------------- SD Card Log -------------------
File dataFile;
File configFile; 

const unsigned int SDCardInterval = 100;
unsigned long SDCardPreviousMillis = 0;
int SDCardState = 0; //0=No Card, 1=Card Detected
// long t_log;
// float mA_A0;
// float loadCellWeight = 0;

//--------------- Nextion Display --------------
const unsigned int displayInterval = 125; //250
unsigned long displayPreviousMillis;
int data = 0;
int hexValue[7];
int n = 0;
int forceGaugePointerAngle;
int forceGaugePointerAnglePrevious;

int nextionActivePage = 0;

struct message {
  int page;
  int id;
  bool touchEvt;
};

struct message nextionMSG;


// ================= setup() ====================


void setup() {

  Serial.begin(115200);
  // Serial.begin(9600);

  //---------------- Load Cell -----------------

  // calValue=(c_nomU9c*V_AVDD_LOADCELL*GAIN_HX711/LOADCELL_FORCE_MAX)/(V_AVDD_HX711*1000/ADC_RESOLUTION)
  // calValue=(1*5*128/20000)/(5*1000/2^24)=107.374
  const float calValue = 107.374; // calibration value for load cell U9C (Newton)
  const long stabilisingtime = 2000; //(((2+1+1)/80)*1000)+10;
  // calValue = 1053.422*66; //for DMS
  //- calValue = 107.374;   // For Load Cell U9C Newton
  delay(10);
  LoadCell.begin();
  LoadCell.start(stabilisingtime);
  // LoadCell.setTareOffset(8062*calValue); // Offset for Load Cell
  LoadCell.setCalFactor(calValue);


  //---------------- Break Motor  ---------------
  // Cytron 7V-30V DC Motor Shield MD 10 R2 Setup


// Breake config 
  loadingForceF1 = 1500+90+50;
  loadingForceF2 = -1500-90-50;
  loadedTimeF1 = 1000;
  loadedTimeF2 = 1000;
  
  brClosingTime = 100;  
  brOpeningTime = 1000; 
  brOpenTime = 200;
  brClosedTime = 200;


// TestBench config 
  cycleDurationTime = (brOpeningTime+brOpenTime+brClosingTime+brClosedTime+1000+loadedTimeF1+1000+loadedTimeF2+1000)/1000;

  TCCR1B = TCCR1B & 0b11111000 | 0x01;

  pinMode(8, OUTPUT);   // Direction Pin
  pinMode(9, OUTPUT);   // PWM Pin



  // ---------------- Break Motor Type 2 ---------------
  //  Cytron 7V-30V DC Motor Shield
  // Valve 1 and 2

  pinMode(2, OUTPUT);   // Direction Pin A
  pinMode(3, OUTPUT);   // Direction Pin B




  //---------------- 4-20mA Reciver ------------------

  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV  //Adafruit
  ads.begin();  //Adafruit


  //---------------- 0-5V Reciver ------------------

  adsCRONOS.setGain(GAIN_ONE);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV  //Adafruit
  adsCRONOS.begin();  //Adafruit
  

  // ------------- 4-20mA Transmitter ----------------
  // dac.begin(0x62);  // Adafruit
  dac.begin(0x60);   //ncd
  dac.setVoltage(midValue , true);  

  // temp 4-20mA Transmitter
    pinMode(5, OUTPUT);   // PWM out 0-5V -> 4-20mA

  // ------------- SD Card log -----------------------
  // Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // Serial.println("Card failed, or not present");
    // don't do anything more:
    // while (1);
    SDCardState = 0;
  } else {
    // dataFile = SD.open("datalog2.txt", FILE_WRITE);
    // configFile = SD.open("config.txt", FILE_WRITE);

    /*
    configFile = SD.open("config.txt", O_WRITE);
    if(configFile){
      unsigned long testNumber = 456;
      char writeBuffer[8];
      // sprintf(writeBuffer, "%08d", testNumber);
      sprintf(writeBuffer, "% 8d", testNumber);
      // ltoa(testNumber, writeBuffer, 8);
      configFile.seek(0);
      configFile.write(writeBuffer, 8);
      configFile.println();
      configFile.close(); 
    }
    */
    configFile = SD.open("config.txt", FILE_READ);
    SDCardState = 1;
    if(configFile){
      char buffer[8];
      configFile.read(buffer, 9);
      testBenchActualCycle = strtoul(buffer, NULL, 10);
      configFile.read(buffer, 9);
      testBenchTargetCycle = strtoul(buffer, NULL, 10);     
      configFile.close();
    }
  }
  // Serial.println("card initialized.");


  


  // ------------- Nextion Display -------------------
  // Serial.begin(115200);



}
// ========================== END setup() ===============================

int n_test = 0;


// ========================== START loop() ==============================
void loop() {


  // Hydraulik Break Test

  /* ---  
    if(n_test==0){
     valve1(HIGH);
     valve2(HIGH);

     brakeMotor(255, LOW);
     //   brakeMotor(0, LOW);
     delay(1000);
     valve1(HIGH);
     valve2(HIGH);
     brakeMotor(0, LOW);
     delay(4000);
     valve1(LOW); //
     valve2(LOW); //

     n_test=1;
    }
    delay(4000);
    n_test=0;

    /* ---- */ 


  // digitalWrite(2, LOW);     // Closed->LOW Open->HIGH
  // digitalWrite(3, LOW);     // Closed->LOW Open->HIGH



  //---------------- Load Cell ------------------

  //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
  //use of delay in sketch will reduce effective sample rate (be carefull with use of delay() in the loop)
  LoadCell.update();
  // loadCellForce =  LoadCell.getData();

  //get smoothed value from data set
  if (millis() - loadCellPreviousMillis > loadCellInterval) {
    loadCellPreviousMillis = millis();
    loadCellForce =  LoadCell.getData();
  }

  //---------------- 4-20mA Reciver ------------------
  int16_t adc0, adc1, adc2, adc3;
  float adc0_mA;
  if (millis() - reciverPreviousMillis > reciverInterval) {
    reciverPreviousMillis = millis();
    adc0 = ads.readADC_Differential_0_1();
    adc0_mA = adc0 * 0.000628;
    cylinderPos = floatMap(adc0_mA, 4, 20, 150, 0);  
  } 

  //------------- 4-20mA Reciver CRONOS  ----------------
  int16_t adc0_CRONOS, adc1_CRONOS, adc2_CRONOS, adc3_CRONOS;
  float adc0_V;
  if (millis() - reciverPreviousMillisCRONOS > reciverIntervalCRONOS) {
    reciverPreviousMillisCRONOS = millis();
    // adc0_CRONOS = adsCRONOS.readADC_Differential_0_1();
    adc0_CRONOS = adsCRONOS.readADC_Differential_2_3();
    adc0_V = (adc0_CRONOS * 0.125); 
    // cylinderPos = adc0_V; 
  }


  //-------------- 4-20mA Transmitter ------------------

  // 590  -> 4 mA
  // 1785 -> 12mA
  // 2979 -> 20mA
  //               dac_mA=4095;
  //               dac_mA=4095/2;
  //dac_mA=40;

  if (millis() - transmitterPreviousMillis > transmitterInterval) {
    transmitterPreviousMillis = millis();
    // float i = LoadCell.getData();
    // dac.setVoltage(4090, false);
  }

  //---------------- Motor ------------------
  /*
    // unsigned long currentMillis = millis();
    motorCurrentMillis = millis();

    if (motorCurrentMillis - motorPreviousMillis >= motorInterval) {
    motorPreviousMillis = motorCurrentMillis;

    if (motorStatus == 0) {
      motorStatus = 127;   // max 255


    if (motorDirection == LOW) {
       motorDirection = HIGH;
    } else {
       motorDirection = LOW;
    }


      digitalWrite(8, motorDirection);
      analogWrite(9, 127);  // max 255 ,127
    } else {
      digitalWrite(8, motorDirection);
      motorStatus = 0;
      analogWrite(9, 0);
    }
    // analogWrite(3, MotorStatus);
    }
  */


  //--------------- SD Card Log -------------


 if (state == 8){ 
   logForceF1min=9999;
   logForceF1max=-9999;
   logForceF2min=-9999;
   logForceF2max=9999;
   logPosF1min=0;
   logPosF1max=900;
   logPosF2min=0;
   logPosF2max=900;
 }


 // if (state == 14 | state == 16) {
  if (SDCardState == 1 & testBenchMode == 1) {

    if (millis() - SDCardPreviousMillis > SDCardInterval) {
      SDCardPreviousMillis = millis();



 if (state == 14) {

      if (loadCellForce<logForceF1min){
        logForceF1min=loadCellForce;
      }   
      if (loadCellForce>logForceF1max){
        logForceF1max=loadCellForce;
      } 
      if (logPosF1min<cylinderPos) {
        logPosF1min=cylinderPos;
      }
      if (logPosF1max>cylinderPos){
        logPosF1max=cylinderPos;
      }
 }
 
 if (state == 16) {
      if (loadCellForce>logForceF2min){
        logForceF2min=loadCellForce;
      }
      if (loadCellForce<logForceF2max){
        logForceF2max=loadCellForce;
      }
      if (logPosF2min<cylinderPos) {
        logPosF2min=cylinderPos;
      }
      if (logPosF2max>cylinderPos){
        logPosF2max=cylinderPos;
      }

 }



/* -------

      // make a string for assembling the data to log:
//--       String dataString = "";

      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      //       File dataFile = SD.open("datalog1.txt", FILE_WRITE); <-------------------
      // if the file is available, write to it:
      if (dataFile) {
        // dataFile.println(dataString);
        dataFile.print(millis());
        // dataFile.print("1");
        dataFile.print(",");
        dataFile.print(loadCellForce);
        dataFile.print(",");
        dataFile.println((int)cylinderPos);
      }
      // if the file isn't open, pop up an error:
      else {
        // Serial.println("error opening datalog.txt");
      }
-------------- */

      if (state == 18) {  // End Cycle
        if (dataFile) {

    //    dataFile.print(testBenchActualCycle); 
    //    dataFile.print(","); 
    //    dataFile.print(logForceF1min); 
    //    dataFile.print(","); 
    //    dataFile.print(logForceF1max); 
    //    dataFile.print(","); 
    //    dataFile.print(logPosF1min); 
    //    dataFile.print(","); 
    //    dataFile.print(logPosF1max); 
    //    dataFile.print(","); 
    //    dataFile.print(logForceF2min); 
    //    dataFile.print(","); 
    //    dataFile.print(logForceF2max); 
    //    dataFile.print(","); 
    //    dataFile.print(logPosF2min); 
    //    dataFile.print(","); 
    //    dataFile.println(logPosF2max); 

      

        
      }
      // if the file isn't open, pop up an error:
      else {
        // Serial.println("error opening datalog.txt");
      }
      }

      

    }

  }

// }
  // --------- Nextion Display -----------------
  // Display send data


  // Data update by state (Cylce)
  if (state==TestBenchEndCycle) {
    if (nextionActivePage == 0) { 
      int cycleDurationTimeDays = (int)(((testBenchTargetCycle-testBenchActualCycle)*cycleDurationTime)/86400); 
      int cycleDurationTimeHours = (int)(((((testBenchTargetCycle-testBenchActualCycle)*cycleDurationTime)/3600)) % 24); 
      nextionULongNumberSend("n3", cycleDurationTimeDays); 
      nextionULongNumberSend("n4", cycleDurationTimeHours);
      nextionProgressBarSend("j0", (int)(100.0/testBenchTargetCycle*testBenchActualCycle));      // Send cycle progress to the progressbar
      
    }
  }

  
  

  // Data update all displayIntraval (Millis)
  if (millis() - displayPreviousMillis > displayInterval) {
    displayPreviousMillis = millis();

    if (nextionActivePage == 0) {                             // Send for Page 0
      // nextionNumberSend("n0", testBenchActualCycle);          // Send to Actual Cyclus Field
      // nextionNumberSend("n1", testBenchTargetCycle);          // Send to Target Cyclus Field
      nextionULongNumberSend("n0", testBenchActualCycle); 
      nextionULongNumberSend("n1", testBenchTargetCycle); 
      
      // nextionTextSend("t5", String(state));                   // Send to Test Bench State Field
      nextionNumberSend("n2", loadCellForce);                 // Send loadCell to Force Field
      nextionXfloatSend("x0", (int)(cylinderPos * 10));       // Send Cylinder Position to Force field
      // nextionProgressBarSend("j0", 0.625 * cylinderPos);      // Send Cylinder Position to Progressbar


      if ( playButtonState == 1 ) {
         nextionBackgroundColor("b2", 1024);
      } else {
         nextionBackgroundColor("b2", 65535);
      }

      if ( stopButtonState == 1 ) {
         nextionBackgroundColor("b0", 64070);
      } 
      if ( stopButtonState == 0 ) {
         nextionBackgroundColor("b0", 65535);
      }
    }

    if (nextionActivePage == 1) {                             // Send for Page 1
      // nextionTextSend("t6", String(loadCellForce));
      // Cylinder position output
      // nextionTextSend("t7",String(posTransmitterMilliAmpere));
      // nextionTextSend("t7", String((int)cylinderPos));

      nextionNumberSend("n2", loadCellForce);                 // Send to loadCell Force Field
      nextionXfloatSend("x0", (int)(cylinderPos * 10));       // Send Cylinder Position to Force field

    if ( valveButtonState1 == 1 ) {
         nextionBackgroundColor("b10", 1024);
      } else {
         nextionBackgroundColor("b10", 65535);
      }
    
    if ( valveButtonState2 == 1 ) {
         nextionBackgroundColor("b14", 1024);
      } else {
         nextionBackgroundColor("b14", 65535);
      }
    }


    if (nextionActivePage == 3) {                             // Send for Page 3
      int analogValue02;
      float ampereADC02;
      // analogValue02=analogRead(2);
      // ampereADC02=((0.039*(analogValue02-512)));
      // nextionNumberSend("n0", (int) ampereADC02);
      // nextionXfloatSend("x0",(int)(ampereADC02*10));

      int analogValue03;
      float ampereADC03;
      analogValue03 = analogRead(3);
      ampereADC03 = (0.15625 * analogValue03);
      nextionNumberSend("n0", (int) ampereADC03);
      nextionXfloatSend("x0", (int)(ampereADC03 * 10));


      // Calculates the angle of the force gauge pointer
      forceGaugePointerAngle = loadCellForce / 33.33;
      if (forceGaugePointerAngle > -90) {
        forceGaugePointerAngle = forceGaugePointerAngle + 90;
      } else {
        forceGaugePointerAngle = 360 + (forceGaugePointerAngle + 90);
      }

      // force gauge pointer output only if more than 2 degrees
      if (abs(forceGaugePointerAngle - forceGaugePointerAnglePrevious) > 2) {
        nextionGaugeSend("z0", (int)forceGaugePointerAngle);
        forceGaugePointerAnglePrevious = forceGaugePointerAngle;
      }
      // Test bench state output
      nextionTextSend("t40", String(state));

      // Number of test bench cycles output
       nextionTextSend("t5", String(testBenchActualCycle));
    }

  }

  // Display recive data
  data = 0;
  // put your main code here, to run repeatedly:

  /*
    if(Serial.peek()!=0x65) {
        // Serial.flush();
        Serial.read();
    }
  */

  if (Serial.available() > 6)  {
    data = Serial.read();

    if (data == 0x65) {

      hexValue[0] = 0x65;
      // delay(100);
      data = Serial.read();
      n = 0;
      for (int i = 1; i < 7; i++) {
        n = n + 1;
        hexValue[n] = data;
        data = Serial.read();
      }
      nextionMSG.page = hexValue[1];
      nextionMSG.id = hexValue[2];
      nextionMSG.touchEvt = hexValue[3];


      if (nextionMSG.page == 0) {

        if (nextionMSG.id == 6) {                    // Page 0 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 0;
          }
        }
        if (nextionMSG.id == 8) {                    // Page 1 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 1;
          }
        }
        if (nextionMSG.id == 9) {                    // Page 2 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 2;
          }
        }
        if (nextionMSG.id == 16) {                    // Page 3 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 3;
          }
        }

        if (nextionMSG.id == 12) {                   // Go to Init
          if (nextionMSG.touchEvt == 1) {
            state = TestBenchInit;
          }
        }

        if (nextionMSG.id == 26) {                   // Play Button State 
          if (nextionMSG.touchEvt == 1) {
            if (playButtonState == 0) {
              playButtonState = 1;
            } else {
              playButtonState = 0;
              // state=="TestBenchModeSelect";
            }
          }
        }

        if (nextionMSG.id == 10) {                   // Stop Button State
         //  if (nextionMSG.touchEvt == 1) {
         //   state = TestBenchStop;
         // }
          if (nextionMSG.touchEvt == 1) {
            state = TestBenchStop;
            if (stopButtonState==0) {
              stopButtonState=1;
            } else {
              stopButtonState=0;
            }    
          }
        }
      }
      
      if (nextionMSG.page == 1) {

        if (nextionMSG.id == 13) { // Page 0 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 0;
          }
        }
        if (nextionMSG.id == 15) { // Page 1 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 2;
          }
        }
        if (nextionMSG.id == 22) { // Page 2 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 3;
          }
        }

        //  Force -/+
        if (nextionMSG.id == 3) {
          if (nextionMSG.touchEvt == 1) {
            // nextionNumberSend( "n0", 88);
            loadingForceManual=loadingForceManual+100;
            setForce(loadingForceManual);
        
          }
          if (nextionMSG.touchEvt == 0) {

          }
        }
        if (nextionMSG.id == 4) {
          if (nextionMSG.touchEvt == 1) {
            loadingForceManual=loadingForceManual-100;
            setForce(loadingForceManual);
          }
          if (nextionMSG.touchEvt == 0) {

          }
        }

        // Position +/-
        if (nextionMSG.id == 7) {
          if (nextionMSG.touchEvt == 1) {            // - Cylinder move <---
            dac.setVoltage(midValue - 210, false);  //-210
          }
          if (nextionMSG.touchEvt == 0) {
            dac.setVoltage(midValue, false);
          }
          // temp
          analogWrite(5,0);  
        }
        if (nextionMSG.id == 8) {
          if (nextionMSG.touchEvt == 1) {            // + Cylinder move --->
            dac.setVoltage(midValue + 210, false); //+210
          }
          if (nextionMSG.touchEvt == 0) {
            dac.setVoltage(midValue, false);
          }

          // temp 
          analogWrite(5,255);
        }


        // Brake Motor open/close
        if (nextionMSG.id == 10) {
          if (nextionMSG.touchEvt == 1) {            // Brake motor closes
            digitalWrite(8, LOW);  // direction
            analogWrite(9, motorPWM);  // max 255 ,12
          }
          if (nextionMSG.touchEvt == 0) {
            digitalWrite(8, LOW);  // direction
            analogWrite(9, 0);  // max 255 ,12
          }
        }
        if (nextionMSG.id == 11) {                   // Brake motor opens
          if (nextionMSG.touchEvt == 1) {
            digitalWrite(8, HIGH);  // direction
            analogWrite(9, motorPWM);  // max 255 ,12
          }
          if (nextionMSG.touchEvt == 0) {
            digitalWrite(8, HIGH);  // direction
            analogWrite(9, 0);  // max 255 ,12
          }
        }


        // Brake open/close Manual
        if (nextionMSG.id == 23) {                   // Brake open  29
          if (nextionMSG.touchEvt == 1) {
            // state="BrakeOpening";
            state = BrakeOpeningManual;
          }
        }
        if (nextionMSG.id == 24) {                   // Brake close 30
          if (nextionMSG.touchEvt == 1) {
            // state="BrakeClosing";
            state = BrakeClosingManual;
          }
        }

        if (nextionMSG.id == 26) {                   // Valve 1 Button State 
          if (nextionMSG.touchEvt == 1) {
            if (valveButtonState1 == 0) {
              valveButtonState1 = 1;
            } else {
              valveButtonState1 = 0;
            }
          }
        }

        if (nextionMSG.id == 27) {                   // Valve 2 Button State 
          if (nextionMSG.touchEvt == 1) {
            if (valveButtonState2 == 0) {
              valveButtonState2 = 1;
            } else {
              valveButtonState2 = 0;
            }
          }
        }

        if (nextionMSG.id == 17) {                   // Go to Stop
          if (nextionMSG.touchEvt == 1) {
            state = TestBenchStop;
          }
        }

      }

      if (nextionMSG.page == 2) {

        if (nextionMSG.id == 4) { // Page 0 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 0;
          }
        }
        if (nextionMSG.id == 5) { // Page 1 Selector
          if (nextionMSG.touchEvt == 1) {
             nextionActivePage = 1;
          }
        }
        if (nextionMSG.id == 9) { // Page 3 Selector
          if (nextionMSG.touchEvt == 1) {
             nextionActivePage = 3;
          }
        }
        if (nextionMSG.id == 7) {                   // Go to Stop
          if (nextionMSG.touchEvt == 1) {
            state = TestBenchStop;
         }
       }
      }



      if (nextionMSG.page == 3) {

        if (nextionMSG.id == 2) { // Page 0 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 0;
          }
        }
        if (nextionMSG.id == 3) { // Page 1 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 1;
          }
        }
        if (nextionMSG.id == 4) { // Page 2 Selector
          if (nextionMSG.touchEvt == 1) {
            nextionActivePage = 2;
          }
        }

        //  Force -/+
        if (nextionMSG.id == 4) {
          if (nextionMSG.touchEvt == 1) {

          }
          if (nextionMSG.touchEvt == 0) {

          }
        }
        if (nextionMSG.id == 5) {
          if (nextionMSG.touchEvt == 1) {

          }
          if (nextionMSG.touchEvt == 0) {

          }
        }

        // Force Manual +/-
        if (nextionMSG.id == 8) {
          if (nextionMSG.touchEvt == 1) {             // Cylinder move for Force <---
            state = BrakeForceUnloadingManual;
          }
          if (nextionMSG.touchEvt == 0) {

          }
        }
        if (nextionMSG.id == 9) {
          if (nextionMSG.touchEvt == 1) {            // Cylinder move for Force --->
            state = BrakeForceLoadingManual;
          }
          if (nextionMSG.touchEvt == 0) {

          }
        }

        // Brake Motor open/close Manual
        if (nextionMSG.id == 10) {                   // Brake opens
          if (nextionMSG.touchEvt == 1) {
            // state="BrakeOpening";
            state = BrakeOpeningManual;
          }
        }
        if (nextionMSG.id == 11) {                   // Brake closes
          if (nextionMSG.touchEvt == 1) {
            // state="BrakeClosing";
            state = BrakeClosingManual;
          }
        }

        if (nextionMSG.id == 17) {                   // Go to Init
          if (nextionMSG.touchEvt == 1) {
            state = TestBenchInit;
          }
        }

        // Auto Button State
        if (nextionMSG.id == 13) {
          if (nextionMSG.touchEvt == 1) {
            if (testBenchMode == 0) {
              testBenchMode = 1;
            } else {
              testBenchMode = 0;
              // state=="TestBenchModeSelect";
            }
          }
        }
        if (nextionMSG.id == 6) {                   // Go to Stop
          if (nextionMSG.touchEvt == 1) {
            state = TestBenchStop;
        }
       }

      }

    }
  }

  // Set Manual Mode
  if (valveButtonState1==0){
    valve1(LOW);
  }  
  if (valveButtonState1==1){
    valve1(HIGH);
  }
  if (valveButtonState2==0){
    valve2(LOW);
  }  
  if (valveButtonState2==1){
    valve2(HIGH);
  }


  // --- State Machine ---
  // if (testBenchMode==1){

  switch (state) {

    // if (state=="TestBenchModeSelect"){
    case TestBenchModeSelect:
      logState = 1;
      if (playButtonState==1){
        testBenchMode = 1;
      } else {
        testBenchMode = 99;
      }
      
      if (testBenchMode == 1 & (testBenchActualCycle < testBenchTargetCycle)) {
        // dataFile.flush();
        state = TestBenchStartCycle;
      }
      break;

    // if (state=="TestBenchInit"){
    case TestBenchInit:
      if (MilliDelay(1000)) {
        state = TestBenchInitBrakeClosing;
      }
      break;

    // if (state=="TestBenchInitBrakeClosing"){
    case TestBenchInitBrakeClosing:
      valve1(LOW);
      valve2(LOW);
      if (MilliDelay(brClosingTime)) {
        // brakeMotor(0, LOW);
        state = TestBenchInitBrakeOpening;;
      }   
     break;

    // if (state=="TestBenchInitBrakeOpening"){
    case TestBenchInitBrakeOpening:
      brakeMotor(255, LOW);
      valve1(HIGH);
      valve2(HIGH);
      if (MilliDelay(brOpeningTime)) {
        brakeMotor(0, LOW);
        state = TestBenchInitLoadcell;  //  state = TestBenchInitCylinderPos1;
      } 
      break;


    // if (state=="TestBenchInitLoadcell"){
    case TestBenchInitLoadcell:
      LoadCell.start(2000);
      if (MilliDelay(5000)) {
        // state = TestBenchInitCylinderPos1;
        state = TestBenchInitCylinderPos3;
      }
      break;
      

    // if (state=="TestBenchInitCylinderPos1"){
    case TestBenchInitCylinderPos1:
      if (setCylinderPos(0)) {
        state = TestBenchInitCylinderPos2;
      }
      break;

    // if (state=="TestBenchInitCylinderPos2"){
    case TestBenchInitCylinderPos2:
      if (setCylinderPos(150)) {
        state = TestBenchInitCylinderPos3;
      }
      break;

    // if (state=="TestBenchInitCylinderPos3"){
    case TestBenchInitCylinderPos3:
      if (setCylinderPos(150 / 2)) {
        state = TestBenchModeSelect;
      }
      break;

    // if (state=="TestBenchStartCycle"){
    case TestBenchStartCycle:
      // testBenchActualCycle=testBenchActualCycle+1;


/*
  if (!SD.begin(chipSelect)) {  // Test
    // Serial.println("Card failed, or not present");
    // don't do anything more:
    // while (1);
    SDCardState = 0;
  } else {
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    SDCardState = 1;
  }
*/

   
      logState = 1;
      if (setCylinderPos(150 / 2)) {
        testBenchActualCycle = testBenchActualCycle + 1;
        if (intermediateCycleCounter > 1000) {
          intermediateCycleCounter = 0;
        }
        intermediateCycleCounter = intermediateCycleCounter + 1;
        if (intermediateCycleCounter%2 == 0) {
          loadingForceF1 = -1500;
          loadingForceF2 = 1500;

          //   loadingForceF1 = 1500+90+50;
          //   loadingForceF2 = -1500-90-50;
          
        } else {
          loadingForceF1 = 1500;
          loadingForceF2 =-1500;
        }

        if (intermediateCycleCounter>1){
          if (intermediateCycleCounter%10 == 0) {
            loadedTimeF1 = 5000;         
            loadedTimeF2 = 5000;
          } else {
            loadedTimeF1 = 1000;         
            loadedTimeF2 = 1000;       
          }

          if ((intermediateCycleCounter-1)%10 == 0) {
            loadedTimeF1 = 5000;         
            loadedTimeF2 = 5000;
          } else {
            loadedTimeF1 = 1000;         
            loadedTimeF2 = 1000;       
          }
        }

        state = BrakeClosing;
      }
      /*
        if(abs(cylinderPos-75)>20) {  // cylinderPos
        if(setCylinderPos(150/2)){
           state = "BrakeClosing";
        } else {
          state = "BrakeClosing";
        }
        }
      */
      break;

    // if (state=="BrakeOpen"){
    case BrakeOpen:
        valve1(HIGH);  //->
        valve2(HIGH);  //->
        if (MilliDelay(brOpenTime)) { 
          // state = BrakeClosing;
          state = TestBenchModeSelect;
        }
        break;

    // if (state=="BrakeClosing"){
    case BrakeClosing:
      valve1(LOW);
      valve2(LOW);
      if (MilliDelay(brClosingTime)) {
        // brakeMotor(0, LOW);
        state = BrakeClosed;
      }
      break;

    // if (state=="BrakeClosed"){
    case BrakeClosed:
      if (MilliDelay(brClosedTime)) { 
        state = BrakeForceLoadingF1;
      }
      break;

    // if (state=="BrakeForceLoadingF1"){
    case BrakeForceLoadingF1:
      setForce(loadingForceF1);
      if (MilliDelay(1000)) {
        // setForce(loadingForceF1);
        state = BrakeForceLoadedF1;
      }
      break;

    // if (state=="BrakeForceLoadedF1"){
    case BrakeForceLoadedF1:
      setForce(loadingForceF1);
      if (MilliDelay(loadedTimeF1)) {  
        state = BrakeForceLoadingF2;
      }
      break;

    //if (state=="BrakeForceLoadingF2"){
    case BrakeForceLoadingF2:
      setForce(loadingForceF2);
      if (MilliDelay(1000)) {
        //- setForce(loadingForceF2);
        state = BrakeForceLoadedF2;
      }
      break;

    // if (state=="BrakeForceLoadedF2"){
    case BrakeForceLoadedF2:
      setForce(loadingForceF2);
      if (MilliDelay(loadedTimeF2)) {  
        // setForce(0);
        state = BrakeForceUnloading;
      }
      break;

    // if (state=="BrakeForceUnloading"){
    case BrakeForceUnloading:
      setForce(0);
      if (MilliDelay(1000)) {
        // setForce(0);
        state = BrakeForceUnloaded;
      }
      break;

    // if (state=="BrakeForceUnloaded"){
    case BrakeForceUnloaded:
      dac.setVoltage(midValue, false);
      if (MilliDelay(1000)) {
        state = BrakeOpening;
      }
      break;

    // if (state=="BrakeOpening"){
    case BrakeOpening:
      brakeMotor(255, LOW);
      valve1(HIGH);
      valve2(HIGH);
      if (MilliDelay(brOpeningTime)) {
        brakeMotor(0, LOW);
        state = TestBenchEndCycle;
       }
    break;

 

    case TestBenchEndCycle:
       // log
       dataFile = SD.open("datalog.txt", FILE_WRITE);
       // SDCardState = 1;
       if(dataFile){
         dataFile.print(testBenchActualCycle); 
         dataFile.print(","); 
         dataFile.print(logForceF1min); 
         dataFile.print(","); 
         dataFile.print(logForceF1max); 
         dataFile.print(","); 
         dataFile.print(logPosF1min); 
         dataFile.print(","); 
         dataFile.print(logPosF1max); 
         dataFile.print(","); 
         dataFile.print(logForceF2min); 
         dataFile.print(","); 
         dataFile.print(logForceF2max); 
         dataFile.print(","); 
         dataFile.print(logPosF2min); 
         dataFile.print(","); 
         dataFile.println(logPosF2max); 
         // --- dataFile.flush();
         dataFile.close();     
       }

       configFile = SD.open("config.txt", O_WRITE);
       if(configFile){
         // unsigned long testNumber = 456;
         char writeBuffer[8];
         sprintf(writeBuffer, "% 8d", testBenchActualCycle);
         configFile.seek(0);
         configFile.write(writeBuffer, 8);
         configFile.println();
         configFile.close(); 
       }
       
       // state = TestBenchModeSelect;
       state = BrakeOpen;
    break;

    // if (state=="BrakeClosingManual"){
    case BrakeClosingManual:
      // brakeMotor(motorPWM, LOW);
      valve1(LOW);
      valve2(LOW);
      if (MilliDelay(brClosingTime)) {
        // brakeMotor(0, LOW);
        state = TestBenchModeSelect;
      }
      break;

    // if (state=="BrakeOpeningManual"){
    case BrakeOpeningManual:
      brakeMotor(255, LOW);
      valve1(HIGH);
      valve2(HIGH);
      if (MilliDelay(brOpeningTime)) {
        brakeMotor(0, LOW);
        state = TestBenchModeSelect;
      }
      break;

    // if (state=="BrakeForceLoadingManual"){
    case BrakeForceLoadingManual:
      setForce(3000);
      if (MilliDelay(5000)) {
        // setForce(1000);
        state = TestBenchModeSelect;
      }
      break;

    // if (state=="BrakeForceUnloadingManual"){
    case BrakeForceUnloadingManual:
      setForce(0);
      if (MilliDelay(2000)) {
        // setForce(0);
        dac.setVoltage(midValue, false);
        state = TestBenchModeSelect;
      }
    break;

    case TestBenchStop:
      brakeMotor(0, LOW);
      valve1(LOW);
      valve2(LOW);
      dac.setVoltage(midValue, false); 
      testBenchMode = 0;
      playButtonState = 0;
      // stopButtonState = 1;
      if (stopButtonState == 0){ 
        testBenchMode = 99;
        state = TestBenchModeSelect;
        stopButtonState = 0;
      }
    break;
  }
}

// ================= END loop() =========================

bool MilliDelay(unsigned long millisIntervall) {
  static unsigned long intervall = 0;
  static unsigned long prevMillis;
  unsigned long actMillis = millis();
  if (intervall == 0) {
    intervall = millisIntervall;
    prevMillis = actMillis;
  }
  if (actMillis - prevMillis >= intervall) {
    intervall = 0;
    return true;
  }
  return false;
}

void brakeMotor(int motorPWM, bool motorDirection) {
  digitalWrite(8, motorDirection); //CW->LOW CCW->HIGH //
  analogWrite(9, motorPWM);  // max 255 ,min 12
}


void valve1( bool valveState) {
  digitalWrite(2, valveState);     // Closed->LOW Open->HIGH
}

void valve2( bool valveState) {
  digitalWrite(3, valveState);     // Closed->LOW Open->HIGH
}




float floatMap(float xValue, float x1, float x2, float y1, float y2) {
  // return ((x2-x1)*(xValue-y1))/(y2-y1);
  // return ((y2-y1)*(xValue-y1))/(x2-x1);
  return (xValue - x1) * (((y2 - y1) / (x2 - x1))) + y1;
}



void setForce(float targetForce) {     // actual
  int tempVoltage = 0;
  loadCellForce =  LoadCell.getData();
  if (targetForce >= (loadCellForce )) { //+10

    if (abs(targetForce) > 1200) { //1000 //1200
      // tempVoltage = 165 + ((targetForce - loadCellForce) / 5); // 35       /2
      tempVoltage = 50 + ((targetForce - loadCellForce) / 3); 
    } else {
      tempVoltage = 35 + ((targetForce - loadCellForce) / 2); // 35       /2
    }

    if ( tempVoltage > 300) {
      tempVoltage = 300;                            // 300
    }

    dac.setVoltage(midValue - tempVoltage, false); // + <<<
    // dac.setVoltage(midValue+((targetForce-loadCellForce)/6), false);
  }
  if (targetForce < (loadCellForce - 0)) {
    // tempVoltage=165+((loadCellForce-targetForce)/5);  // 35      /2
    if (abs(targetForce) > 1200) { //1000 //1200
      // tempVoltage = 165 + ((loadCellForce - targetForce) / 5); // 35      /2
       tempVoltage = 50 + ((loadCellForce - targetForce) / 3); 
    } else {
      tempVoltage = 35 + ((loadCellForce - targetForce) / 2); // 35      /2
    }

    if ( tempVoltage > 300) {
      tempVoltage = 300;                         // 300
    }
    dac.setVoltage((midValue + tempVoltage), false); // - <<<
  }

  // if(abs(loadCellForce-targetForce)<=50){
  //    dac.setVoltage(midValue, false);
  // }
}

void setForceNew(float targetForce) {    // New
  int tempVoltage = 0;
  loadCellForce =  LoadCell.getData();

  if (targetForce >= 0) {
    if (targetForce >= loadCellForce) {
      tempVoltage = (targetForce * 0.04) + ((targetForce - loadCellForce) / 3);
      // tempVoltage = 50 + ((targetForce - loadCellForce) / 3);
      if ( tempVoltage > 300) {
        tempVoltage = 300;
      }
      if ( tempVoltage < 35) {
        tempVoltage = 35;
      }
      dac.setVoltage(midValue - tempVoltage, false); 
    } else {
      tempVoltage = 35 + ((loadCellForce - targetForce) / 2);
      dac.setVoltage(midValue + tempVoltage, false); 
    }
  }
  if (targetForce < 0) {
    if (targetForce <= loadCellForce) {
      tempVoltage = (targetForce * 0.04) + ((loadCellForce - targetForce) / 3);
      // tempVoltage = 50 + ((targetForce - loadCellForce) / 3);
      if ( tempVoltage > 300) {
        tempVoltage = 300;
      }
      if ( tempVoltage < 35) {
        tempVoltage = 35;
      }
      dac.setVoltage(midValue + tempVoltage, false); 
    } else {
      tempVoltage = 35 + ((loadCellForce - targetForce) / 2);
      dac.setVoltage(midValue - tempVoltage, false); 
    }
  }
}



/*
bool setCylinderPos(float targetPos) {
  int tempVoltage = 0;
  if (targetPos > (cylinderPos)) {
    // dac.setVoltage(midValue+150, false);  // 150
    // tempVoltage=35+((targetPos-cylinderPos)*5); // 35        /5
    if ((targetPos - cylinderPos) > 10) {
      tempVoltage = 230;
    } else {
      tempVoltage = 180;
    }
    dac.setVoltage(midValue - tempVoltage, false);
    // dac.setVoltage(midValue+((targetForce-loadCellForce)/6), false);
  }
  if (targetPos < (cylinderPos)) {
    // tempVoltage=35+((cylinderPos-targetPos)*5);  // 35      /5
    if ((cylinderPos - targetPos) > 10) {
      tempVoltage = 230;
    } else {
      tempVoltage = 180; //200
    }
    dac.setVoltage((midValue + tempVoltage), false);
  }

  if (abs(cylinderPos - targetPos) <= 1) {
    dac.setVoltage(midValue, false);
    return true;
  } else {
    return false;
  }
}
*/



bool setCylinderPos(float targetPos) {
  int tempVoltage = 0;
  if (targetPos < (cylinderPos)) {
    // dac.setVoltage(midValue+150, false);  // 150
    // tempVoltage=35+((targetPos-cylinderPos)*5); // 35        /5
    if ((targetPos - cylinderPos) > 10) {
      tempVoltage = 230;
    } else {
      tempVoltage = 180;
    }
    dac.setVoltage(midValue - tempVoltage, false);
    // dac.setVoltage(midValue+((targetForce-loadCellForce)/6), false);
  }
  if (targetPos > (cylinderPos)) {
    // tempVoltage=35+((cylinderPos-targetPos)*5);  // 35      /5
    if ((cylinderPos - targetPos) > 10) {
      tempVoltage = 230;
    } else {
      tempVoltage = 180; //200
    }
    dac.setVoltage((midValue + tempVoltage), false);
  }

  if (abs(cylinderPos - targetPos) <= 1) {
    dac.setVoltage(midValue, false);
    return true;
  } else {
    return false;
  }
}


// -----------------  Nextion Display -----------------------------------
void nextionTextSend(char objname[], String text) {
  // Serial.print(F("t0.txt=\""));
  // Serial.print(objname);
  Serial.print(objname);
  Serial.print(".txt=\"");
  Serial.print(text);
  Serial.print(F("\""));
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void nextionNumberSend(char objname[], int value) {
  Serial.print(objname);
  Serial.print(".val=");
  // Serial.print("n2.val=");
  Serial.print(value);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void nextionULongNumberSend(char objname[], unsigned long  value) {
  Serial.print(objname);
  Serial.print(".val=");
  // Serial.print("n2.val=");
  Serial.print(value);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}
void nextionXfloatSend(char objname[], int value) {
  Serial.print(objname);
  Serial.print(".val=");
  // Serial.print("x0.val=");
  Serial.print(value);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void nextionGaugeSend(char objname[], int value) {
  Serial.print(objname);
  Serial.print(".val=");
  // Serial.print("z0.val=");
  Serial.print(value);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void nextionProgressBarSend(char objname[], int value) {
  Serial.print(objname);
  Serial.print(".val=");
  Serial.print(value);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void nextionPageSend(char page[]) {
  Serial.print("page ");
  Serial.print(page);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void nextionBackgroundColor(char objname[], unsigned int value) {
  Serial.print(objname);
  Serial.print(".bco=");
  Serial.print(value);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// void nextionButtonId002Press(){
// nextionTextSend("t1","B2 press");
// nextionTextSend("t0","2");
// nextionProgressBarSend("j0",10);
// }

// void nextionButtonId002Release(){
// nextionTextSend("t1","B2 release");
// nextionTextSend("t0","3");
// nextionProgressBarSend("j0",90);
// }

// void nextionButtonId003Press(){
// nextionTextSend("t1","B3 press");
// }

// void nextionButtonId003Release(){
// nextionTextSend("t1","B3 release");
// }
