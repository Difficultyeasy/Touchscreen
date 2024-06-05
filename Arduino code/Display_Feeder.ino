///////////////////////////////////////////////////////////////////
//////////Dan Pesserl
//////////10/2/2023
//////////Display Feeder for Stone HMI Displays 
////////// 9 axis gyro, ammeter, 4 line voltmeter for 4s battery, radiator temp and fan switch, battery temp, usb-c power in
///////////////////////////////////////////////////////////////////


////Sections of code by feature 

//////////////// AMMETER  ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

        /* 0- General */

        int decimalPrecision = 2;                   // decimal places for all values shown in LED Display & Serial Monitor

        /* 1- AC Current Measurement */

        int currentAnalogInputPin = A4;             // Which pin to measure Current Value (A0 is reserved for LCD Display Shield Button function)
        int calibrationPin = A5;                    // Which pin to calibrate offset middle value
        float manualOffset = 0.00;                  // Key in value to manually offset the initial value
        float mVperAmpValue =0.9;// 5.2083; //4.166;                 // If using "Hall-Effect" Current Transformer, key in value using this formula: mVperAmp = maximum voltage range (in milli volt) / current rating of CT
                                                    // For example, a 20A Hall-Effect Current Transformer rated at 20A, 2.5V +/- 0.625V, mVperAmp will be 625 mV / 20A = 31.25mV/A 
                                                    // For example, a 50A Hall-Effect Current Transformer rated at 50A, 2.5V +/- 0.625V, mVperAmp will be 625 mV / 50A = 12.5 mV/A
        float supplyVoltage = 5000;                 // Analog input pin maximum supply voltage, Arduino Uno or Mega is 5000mV while Arduino Nano or Node MCU is 3300mV
        float offsetSampleRead = 0;                 /* to read the value of a sample for offset purpose later */
        float currentSampleRead  = 0;               /* to read the value of a sample including currentOffset1 value*/
        float currentLastSample  = 0;               /* to count time for each sample. Technically 1 milli second 1 sample is taken */
        float currentSampleSum   = 0;               /* accumulation of sample readings */
        float currentSampleCount = 0;               /* to count number of sample. */
        float currentMean ;                         /* to calculate the average value from all samples, in analog values*/ 
        float RMSCurrentMean ;                      /* square roof of currentMean, in analog values */   
        float FinalRMSCurrent ;                     /* the final RMS current reading*/


//////////////// GYRO  ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/*
Test on MEGA 2560. use WT901CTTL sensor

WT901CTTL     MEGA 2560
    VCC <--->  5V/3.3V
    TX  <--->  19(TX1)
    RX  <--->  18(RX1)
    GND <--->  GND
*/

#include <SoftwareSerial.h>
#include <REG.h>
#include <wit_c_sdk.h>

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
int loopCount;
//////////////////////////////////////////////////////////////////////////




///////////////////  TEMP  ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

int tempAnalogIn = A15; //battery
int ThermistorPin = A14; //radiator
//////////////////////////////////////////////////////////////////////////




///////////////////  DISPLAY  ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
int cnt;
int gaugeVal = 0;
int img4runFrontRot = 1;
int img4runFrontRotTicToc = 0;


int img4runSideRot = 185;
int img4runSideRotTicToc = 0;


String endChar = String(char(0xff)) + String(char(0xff)) + String(char(0xff));


int debug = 0;


//////////////////////////////////////////////////////////////////////////




///////////////////  VOLTMETER  ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//#include "ADS1X15.h"
#include <Adafruit_ADS1X15.h>
//ADS1115 ADS(0x48);
Adafruit_ADS1115 ADS;

//////////////////////////////////////////////////////////////////////////



void setup() {

  //////////////// GYRO  ///////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  Serial.begin(115200);
  Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
  //Serial1.begin(c_uiBaud[6]);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  //Serial.print("\r\n********************** wit-motion normal example  ************************\r\n");
  //	AutoScanSensor();

  //needed to keep connection
  Serial1.begin(c_uiBaud[2]);
  Serial1.flush();
  ///////////////////////////
  int loopCount = 0;

  //////////////////////////////////////////////////////////////////////////



  ///////////////////  TEMP  ////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  pinMode(tempAnalogIn, INPUT);

  //////////////////////////////////////////////////////////////////////////

  ///////////////////  DISPLAY  ////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  cnt = 0;

  gaugeVal = 270;
  //Serial.begin(115200);

  while (!Serial) {

    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial3.begin(115200);


  //////////////////////////////////////////////////////////////////////////



///////////////////  VOLTMETER  ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

  //Serial.print("ADS1X15_LIB_VERSION: ");
  //Serial.println(ADS1X15_LIB_VERSION);

  pinMode(25, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  
/*
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
  

  pinMode(41, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(49, OUTPUT);


digitalWrite(45, HIGH); 
digitalWrite(47, HIGH); 
*/

  ADS.begin();


}
//////////////////////////////////////////////////////////////////////////



//////////////// GYRO  ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

int i;
float fAcc[3], fGyro[3], fAngle[3];


    
//////////////////////////////////////////////////////////////////////////

void loop() {

  displayTest();
 // gyroTest(); //get debug info from gyro
  

//needed to keep connection
//for gyro
  if (loopCount % 300 == 0 || loopCount < 10) {
    
    Serial1.begin(c_uiBaud[2]);
    Serial1.flush();
  }
////////////////////////////

//for gyro to get data every 10th loop
  if (loopCount % 10 == 0) {
    
    AutoScanSensor2(); //get data from gyro
  }


  if (loopCount % 3000 == 0) {
    
    TempRad();

  }

  if (loopCount % 7000 == 0) {
  
    TempBat();
  
  }  


  if (loopCount % 11000 == 0) {
   
    Ammeter();
  }

if (loopCount % 13000 == 0) {
  
    Voltmeter();

  }

//reset loop
  if (loopCount % 9000000000 == 0) {
    loopCount = 0;
  }

//for gyro - reset connection 
  if (loopCount % 100000000 == 0) {

    Serial1.begin(c_uiBaud[i]);
    Serial1.flush();

    if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else {
      Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
     if(debug==3) Serial.print(" 115200 Baud rate modified successfully\r\n");
    }
  }



  loopCount++;
}


static void Voltmeter()
{

//Code and ideas stolen from these guys and expanded
//https://www.instructables.com/Make-a-Mini-Arduino-programmable-4-channel-DC-DVM/
//https://www.instructables.com/Arduino-Volt-Meter-0-100V-DC-Version-2-better/>
//https://github.com/RobTillaart/ADS1X15  https://electropeak.com/learn/


    float sum[4] = {0.0};             // sums of samples taken
    unsigned char Scount = 0;        // current sample number
    float AVvolts[4] = {0.0};       // calculated voltages
    float Cellvolts[4] = {0.0};       // calculated cell voltages
    float ADCVal[4] = {0.0};       // ADC values
    char cnt1 = 0;                // used in 'for' loops
    int calib = 0; // Value of calibration of ADS1115 to reduce error
    int16_t adc; // 16 bits ADC read of input 2
    float voltage=0;
    float Radjust = 0.0438473656572729; // Voltage divider factor ( R2 / R1+R2 )
    int pin = 25; //starting control pin for SSR 

    int calibDelay = 10000; //give 10 sec for voltmeter
    int calibLine = 66; //above 3 it's disabled
    
    // number of analog samples to take per reading, per channel
    int NSAMP = 25; // increase for more accuracy but will delay display refresh noticably above 50. Decrease each calibb value accordingly 

       
      for (byte cnt1 = 0; cnt1 < 4; cnt1++) {

        pin = 25 + (cnt1*2);           
        digitalWrite(pin, HIGH);

        while (Scount < NSAMP) {    

            //calibrating each channel
            if(cnt1==0)
            {
               calib = 9;

            }  
            else if(cnt1==1)
            {
               calib = 13;
            } 
            else if(cnt1==2)
            {
               calib = 16;

            } 
            else if(cnt1==3)
            {
               calib = 15; 
 
               //delay(500);

            } 
            
            adc = ADS.readADC_SingleEnded(2);
        
            voltage = ((adc + calib) * 0.1875)/1000;
            voltage = voltage / Radjust;

            sum[cnt1] += voltage;
            ADCVal[cnt1] += adc;
            voltage=0;

            Scount++;
            //delay(1);
        }
        if(calibLine==cnt1)
        {
          delay(calibDelay);
        }
        digitalWrite(pin, LOW);
        Scount=0;
       // delay(1);
    }

    // calculate the average voltage for each channel
    for (cnt1 = 0; cnt1 < 4; cnt1++) {
        AVvolts[cnt1] = (((float)sum[cnt1] / (float)NSAMP)); 
        ADCVal[cnt1] = ADCVal[cnt1] / (float)NSAMP; 

    // calculate the cell voltage for each channel        
        if(cnt1>0)
        {
          Cellvolts[cnt1] = AVvolts[cnt1] - AVvolts[cnt1 -1] ;    
        }
        else
        {
          Cellvolts[cnt1] = AVvolts[cnt1];
        }
    }

    // display voltages 

    // voltage 1 - V1(pin A0  
    Serial.print("V1 ");
    Serial.print(AVvolts[0] , 4);
    Serial.print("V ");
    
    // voltage 2 - V2(pin A1)
    Serial.print("V2 ");
    Serial.print(AVvolts[1] , 4);
    Serial.print("V ");
    
    // voltge 3 - V3(pin A2)
    Serial.print("V3 ");
    Serial.print(AVvolts[2] , 4);
    Serial.print("V ");
    
    // voltage 4 - V4(pin A3)
    Serial.print("V4 ");
    Serial.print(AVvolts[3] , 4);
    Serial.print("V ");

    Serial.println();
    

    Serial.print("V1 raw ");
    Serial.print(ADCVal[0]);
    
    Serial.print("  V2 raw ");
    Serial.print(ADCVal[1]);
    
    Serial.print("  V3 raw ");
    Serial.print(ADCVal[2]);
    
    Serial.print(" V4 raw ");
    Serial.print(ADCVal[3]);

    Serial.println();



    Serial.print("V1 cell ");
    Serial.print(Cellvolts[0] , 4);
    
    Serial.print("  V2 cell ");
    Serial.print(Cellvolts[1] , 4);
    
    Serial.print("  V3 cell ");
    Serial.print(Cellvolts[2] , 4);
    
    Serial.print(" V4 cell ");
    Serial.print(Cellvolts[3] , 4);

    Serial.println();


    Scount = 0;
    for (cnt1 = 0; cnt1 < 4; cnt1++) {
        sum[cnt1] = 0;
        AVvolts[cnt1] = 0;
        ADCVal[cnt1] = 0;
        Cellvolts[cnt1] = 0;
    }


}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  // from https://forum.arduino.cc/t/read-internal-voltage-referance/130996

  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}



static void Ammeter()
{

//from https://solarduino.com/how-to-measure-dc-and-ac-current-using-hsts016l-hall-effect-current-transformer/

        /* 1- AC & DC Current Measurement */
  while(true)
  {
        if(micros() >= currentLastSample + 200)                                                               /* every 0.2 milli second taking 1 reading */
          { 
           currentSampleRead = analogRead(currentAnalogInputPin)-analogRead(calibrationPin);                  /* read the sample value including offset value*/
           currentSampleSum = currentSampleSum + sq(currentSampleRead) ;                                      /* accumulate total analog values for each sample readings*/
           currentSampleCount = currentSampleCount + 1;                                                       /* to count and move on to the next following count */  
           currentLastSample = micros();                                                                      /* to reset the time again so that next cycle can start again*/ 
          }
        
        if(currentSampleCount == 4000)                                                                        /* after 4000 count or 800 milli seconds (0.8 second), do this following codes*/
          { 
            currentMean = currentSampleSum/currentSampleCount;                                                /* average accumulated analog values*/
            RMSCurrentMean = sqrt(currentMean);                                                               /* square root of the average value*/
            FinalRMSCurrent = (((RMSCurrentMean /1023) *supplyVoltage) /mVperAmpValue)- manualOffset;         /* calculate the final RMS current*/
            if(FinalRMSCurrent <= (625/mVperAmpValue/100))                                                    /* if the current detected is less than or up to 1%, set current value to 0A*/
            { FinalRMSCurrent =0; }
            Serial.println();
            Serial.print(" The Current RMS value is: ");
            Serial.print(FinalRMSCurrent,decimalPrecision);
            Serial.println(" A ");
            Serial.println();
            currentSampleSum =0;                                                                              /* to reset accumulate sample values for the next cycle */
            currentSampleCount=0;                                                                             /* to reset number of sample for the next cycle */
            break;
          }
  }
}

static void TempRad() {
//from https://www.electronicdiys.com/2020/05/ntc-temperature-sensor-with-arduino-esp.html

  int ThermistorPin = A14;
  int Vo;
  float R1 = 50000; //@25C 
  float logR2, R2, T, Tc, Tf;
  //float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07; //orig
float c1 = 1.972609583e-03, c2 = 0.3350778613e-04, c3 = 8.034145893e-07;


  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0) / 5.0 + 32.0;

  Serial.println();
  Serial.println("----------TEMP RAD");
  Serial.print("Temperature: ");
  Serial.print(Tf);
  Serial.print(" F; ");
  Serial.print(Tc);
  Serial.println(" C");

  //delay(500);
}



static void TempBat() {

  //Thermistor is 10500 NTC

 //from https://www.electronicdiys.com/2020/05/ntc-temperature-sensor-with-arduino-esp.html

  int ThermistorPin = A15;
  int Vo;
  //float R1 = 30000;
  float R1 = 10500;
  float logR2, R2, T, Tc, Tf;
  //float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07; //orig
  float c1 = 1.1384e-03, c2 = 2.3245e-04, c3 = 9.489e-08; //from CGPT
  //float c1 = 1.217562948-03, c2 = 2.181838337-04, c3 = 1.632756541-07;


  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0) / 5.0 + 32.0;

  Serial.println();
  Serial.println("----------TEMP BAT");
  Serial.print("Temperature: ");
  Serial.print(Tf);
  Serial.print(" F; ");
  Serial.print(Tc);
  Serial.println(" C");

  //delay(500);
}


//alternative way to calculate temps. NOT USED but may be a good reference.
static void getTemp2() {

//from https://forum.arduino.cc/t/100k-thermistor-reading-incorretcly/455065

  // which analog pin to connect
#define THERMISTORPIN A15
// resistance at 25 degrees C
#define THERMISTORNOMINAL 60000
//#define THERMISTORNOMINAL 800
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 20
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3800
//#define BCOEFFICIENT 3700 //accurate at 150c
// the value of the 'other' resistor
#define SERIESRESISTOR 60000


  // RESISTANCE AT -5C = 138       AVG READING IS 14.0
  // RESISTANCE AT 150C =6000        AVG READING IS 377.0

  int samples[NUMSAMPLES];

  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;


  Serial.println();
  Serial.println("----------TEMP2");
  Serial.print("Average analog reading ");
  Serial.println(average);
/*
  float ohm150c = 395;  //accurate at 150c

  if (ohm150c - 10 <= average) {
    ohm150c = average + 18;
    ohm150c = average + 22;
  } else {
    ohm150c = 355;  //accurate at 150c
    ohm150c = 395;  //accurate at 95c
    ohm150c = 400;  //accurate at 95c
  }

  //ohm150c = average +45; //accurate at 100c DOES NOT WORK


  ohm150c = 395;  //accurate at 150c
    //average = ohm150c - average;
  average = 780 - average + 150;

  // convert the value to resistance
  //average = 1023 / average - 1;

  average = 40000 / average - 1;
  average = SERIESRESISTOR / average;
*/

  Serial.print("Thermistor resistance ");
  Serial.println(average);

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;           // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                       // Invert
  steinhart -= 273.15;                               // convert absolute temp to C

  //steinhart -= 10;

  Serial.print("Temperature ");
  Serial.print(steinhart);
  Serial.println(" *C");

  delay(500);
}


//alternative way to calculate temps. NOT USED but may be a good reference.
static void getTemp() {
//from https://projecthub.arduino.cc/manodeep/diy-ohmmeter-using-arduino-c33f9e


  int analogPin = 0;
  int raw = 0;
  int Vin = 5;
  float Vout = 0;
  float R1 = 10000;
  float R2 = 0;
  float buffer = 0;


  int tempValColdest = 138405;  //32f 0c
  int tempValHottest = 2900;    //302f 150c


  tempValColdest = 750000;  //32f 0c
  tempValHottest = 40000;   //302


  //float data=analogRead(tempAnalogIn);
  delay(200);
  raw = analogRead(tempAnalogIn);
  if (raw) {
    buffer = raw * Vin;
    Vout = (buffer) / 1024.0;
    buffer = (Vin / Vout) - 1;
    R2 = R1 * buffer;

    int inverseR = tempValColdest - R2;

    //int temp = (int)map(R2,tempValColdest,tempValHottest,20,150);

    int temp = (int)map(inverseR, 0, tempValColdest, 0, 150);


    Serial.print("Temp C  ");
    Serial.println(temp);
    Serial.print("R2: ");
    Serial.println(R2);

    Serial.print("inverseR: ");
    Serial.println(inverseR);
  }
  //Serial.print("tempAnalogIn reading=");
  //Serial.println(data,7);
}



//shared lib from gyro WT901CTTL OEM
static void AutoScanSensor2(void) {
  int i, iRetry;

  //for (i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
//    Serial1.begin(c_uiBaud[2]);
//    Serial1.flush();
    //Serial.print("c_uiBaud: ");
    //Serial.print(i);
    //Serial.println();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(20);
      while (Serial1.available()) {
        WitSerialDataIn(Serial1.read());
      }
      if (s_cDataUpdate != 0) {
        for (int j = 0; j < 3; j++) {
          fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
          fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
          fAngle[j] = sReg[Roll + j] / 32768.0f * 180.0f;
        }
        if (s_cDataUpdate & ANGLE_UPDATE) {
         
          if(debug==1)
          {
            Serial.println("angle:");
            Serial.println(fAngle[0], 3);
          }
          gyroToDisplay(fAngle);
        }

        return;
      }
      iRetry--;
    } while (iRetry);
//  }
  if(debug==2)
  {
    Serial.print("can not find sensor 1 \r\n");
    Serial.print("please check your connection 1 \r\n");
  }
}


//shared lib from gyro WT901CTTL OEM
void gyroTest() {

  while (Serial1.available()) {
    WitSerialDataIn(Serial1.read());
  }
  while (Serial1.available()) {
    CopeCmdData(Serial1.read());
  }
  CmdProcess();
  if (s_cDataUpdate) {
    for (i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
    }
    if (s_cDataUpdate & ACC_UPDATE) {
      Serial.print("acc:");
      Serial.print(fAcc[0], 3);
      Serial.print(" ");
      Serial.print(fAcc[1], 3);
      Serial.print(" ");
      Serial.print(fAcc[2], 3);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ACC_UPDATE;
    }
    if (s_cDataUpdate & GYRO_UPDATE) {
      Serial.print("gyro:");
      Serial.print(fGyro[0], 1);
      Serial.print(" ");
      Serial.print(fGyro[1], 1);
      Serial.print(" ");
      Serial.print(fGyro[2], 1);
      Serial.print("\r\n");
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if (s_cDataUpdate & ANGLE_UPDATE) {
      Serial.print("angle:");
      Serial.print(fAngle[0], 3);
      Serial.print(" ");
      Serial.print(fAngle[1], 3);
      Serial.print(" ");
      Serial.print(fAngle[2], 3);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    if (s_cDataUpdate & MAG_UPDATE) {
      Serial.print("mag:");
      Serial.print(sReg[HX]);
      Serial.print(" ");
      Serial.print(sReg[HY]);
      Serial.print(" ");
      Serial.print(sReg[HZ]);
      Serial.print("\r\n");
      s_cDataUpdate &= ~MAG_UPDATE;
    }
    s_cDataUpdate = 0;
  }
}

//shared lib from gyro WT901CTTL OEM
void CopeCmdData(unsigned char ucData) {
  static unsigned char s_ucData[50], s_ucRxCnt = 0;

  s_ucData[s_ucRxCnt++] = ucData;
  if (s_ucRxCnt < 3) return;  //Less than three data returned
  if (s_ucRxCnt >= 50) s_ucRxCnt = 0;
  if (s_ucRxCnt >= 3) {
    if ((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
      s_cCmd = s_ucData[0];
      memset(s_ucData, 0, 50);
      s_ucRxCnt = 0;
    } else {
      s_ucData[0] = s_ucData[1];
      s_ucData[1] = s_ucData[2];
      s_ucRxCnt = 2;
    }
  }
}

//shared lib from gyro WT901CTTL OEM
static void ShowHelp(void) {
  Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
  Serial.print("\r\n************************          HELP           ************************\r\n");
  Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
  Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
  Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
  Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
  Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
  Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
  Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void) {
  switch (s_cCmd) {
    case 'a':
      if (WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
      break;
    case 'm':
      if (WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'e':
      if (WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'u':
      if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'U':
      if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'B':
      if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else {
        Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
        Serial.print(" 115200 Baud rate modified successfully\r\n");
      }
      break;
    case 'b':
      if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else {
        Serial1.begin(c_uiBaud[WIT_BAUD_9600]);
        Serial.print(" 9600 Baud rate modified successfully\r\n");
      }
      break;
    case 'r':
      if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'R':
      if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'C':
      if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c':
      if (WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'h':
      ShowHelp();
      break;
    default: break;
  }
  s_cCmd = 0xff;
}

//shared lib from gyro WT901CTTL OEM
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}
//shared lib from gyro WT901CTTL OEM
static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

//shared lib from gyro WT901CTTL OEM
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

//shared lib from gyro WT901CTTL OEM
static void AutoScanSensor(void) {
  int i, iRetry;

  for (i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    Serial1.begin(c_uiBaud[i]);
    Serial1.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial1.available()) {
        WitSerialDataIn(Serial1.read());
      }
      if (s_cDataUpdate != 0) {
        //Serial.print(c_uiBaud[i]);
        //Serial.print(" baud find sensor\r\n\r\n");
        //ShowHelp();
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}


void displayTest() {
  ///////////////////  DISPLAY  ////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  if (gaugeVal > 360) {
    gaugeVal = 0;
  }


  Serial3.print("z0.val=" + String(gaugeVal) + endChar);
  delay(5);
  gaugeVal++;

  //Serial3.print("p0.pic=" + String(img4runFrontRot) + endChar);

  if (img4runFrontRot >= 183) {
    img4runFrontRotTicToc = 1;

  }

  else if (img4runFrontRot <= 2) {
    img4runFrontRotTicToc = 0;
  }

  if (img4runFrontRotTicToc == 1) {
    img4runFrontRot--;
  }

  else if (img4runFrontRotTicToc == 0) {
    img4runFrontRot++;
  }

  //Serial3.print("p1.pic=" + String(img4runSideRot) + endChar);

  if (img4runSideRot >= 364) {
    img4runSideRotTicToc = 1;

  }

  else if (img4runSideRot <= 185) {
    img4runSideRotTicToc = 0;
  }

  if (img4runSideRotTicToc == 1) {
    img4runSideRot--;
  }

  else if (img4runSideRotTicToc == 0) {
    img4runSideRot++;
  }



  if (cnt == 0) {
    //Serial.println("Wrote: hello");
    cnt = 1;
  }


  String displayData = "";


  if (Serial3.available()) {


    //Serial.println("Reply");


    if (Serial3.available()) {

      displayData += char(Serial3.read());
      delay(3);
    }

    Serial.println(displayData);
    Serial.println();

    cnt = 0;
  }
}


void gyroToDisplay(float fAngle[]) {
  ///////////////////  DISPLAY  ////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  if (gaugeVal > 360) {
    gaugeVal = 0;
  }

  int rotateImageX = (int)map(fAngle[0], -180, 180, 2, 183);
  int rotateImageY = (int)map(fAngle[0], -180, 180, 185, 364);

  /*
    Serial.print("rotateImageX: ");
    Serial.println(rotateImageX);
    Serial.println();


    Serial.print("rotateImageY: ");
    Serial.println(rotateImageY);
    Serial.println();

*/
  Serial3.print("z0.val=" + String(gaugeVal) + endChar);
  delay(5);
  gaugeVal++;



  Serial3.print("p0.pic=" + String(rotateImageX) + endChar);

  Serial3.print("p1.pic=" + String(rotateImageY) + endChar);


  String displayData = "";
}
