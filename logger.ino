/* A simple data logger than
*/
#include <SD.h>               // I much prefer SdFat.h by Greiman over the old SD.h library used here
#include  <SPI.h>

const int chipSelect = 10;    //CS moved to pin 10 on the arduino

#include <Wire.h>
#include "LowPower.h"         // from https://github.com/rocketscream/Low-Power
#include <RTClib.h>           // library from   https://github.com/MrAlvin/RTClib

RTC_DS3231 RTC;               // creates an RTC object in the code


//variables for reading the RTC time & handling the INT(0) interrupt it generates

#define DS3231_I2C_ADDRESS 0x68
int RTC_INTERRUPT_PIN = 2;
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char CycleTimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds)


//#define SampleIntervalMinutes 1
#define RTCPOWER_PIN 7

/*
  // CHANGE SampleIntervalMinutes to the number of minutes you want between samples! Whole numbers 1-59 only
  // NOTE yes the RTClib will do down to one second intervals with the right code
  // HOWEVER if you cut power to an SD card while it
  // is writing information you destroy the data file on the card….
  // so how do you solve that problem if you start sampling too frequently?
*/

#define TS_TMP102  INSTALLED

//Voltage Monitoring

#define vRegulatedMCU 1  //Pro Mini needs to use a voltage divider on analog pin 0
int Vcc = 9999; //the supply voltage (via 1.1 internal band gap OR analog read)
int Vcc2 = 9999; //the post SD card writing supply voltage, 9999 until first card write cycle
int Vdelta;   //change in supply voltage after each SD write cycle...to track battery conditioin

const float referenceVolts = 3.3;
const float resistorFactor = 511; // = 1023.0 * (R2/(R1 + R2));  if R1 and R2 are the same - max measurable input voltage of 6.6v


//----------------------------------------------

volatile boolean clockInterrupt = false;  //this flag is set to true when the RTC interrupt handler is executed

//variables for reading the DS3231 RTC temperature register
float temp3231C;    //Celcius
float temp3231F;     //Farenhiet
byte tMSB = 0;
byte tLSB = 0;


// TMP102 Temperature Sensor
int TEMP_Raw = 0;
float TEMP_degC = 0.0;
uint8_t wholeTemp = 0;
uint8_t fracTemp = 0;

#ifdef TS_TMP102
int TMP102_ADDRESS = 0x48;  // 72=base address (3 others possible)
byte errorflag = 0; //used in tmp102 functions... might not be needed after error process is integrated
#endif


//indicator LED pins
int RED_PIN = 4;
int GREEN_PIN = 5;
int BLUE_PIN = 6;

//Reading Config File

File myFile;
char c, fileName[] = "CONFIG.TXT";
int samplingPeriod;


//Creating timestamped file name

char filename[] = "00000000.TXT";
File nameFile;

//################################################  SETUP  ################################################################

void setup() {

  Serial.begin(9600);    // Open serial communications and wait for port to open:
  Wire.begin();          // start the i2c interface for the RTC
  RTC.begin();           // start the RTC

  delay(3000);


  //Temp sensor(s) init
  //**********************

  initTMP102();



  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card"); delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000);
  // see if the card is present and can be initialized
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don’t do anything more:
    return;
  }
  Serial.println(" Card initialized."); delay(1000);
  Serial.println(" ");

  //Getting time from RTC

  Serial.print("Getting clock time"); delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000); Serial.print(". "); delay(1000);

  DateTime now = RTC.now(); //this reads the time from the RTC
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());

  Serial.println(CycleTimeStamp); delay(1000);
  Serial.println(" ");


  //Loading the config settings from the config file pre saved on the SD card.
  //What's in the CONFIG.TXT: [int],XXXXXXXX.XXX e.g. 10,DATA_SET.TXT      !!!filename must only have 8 charachters.

  myFile = SD.open(fileName);

  Serial.print("Reading config file"); delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000);

  if (myFile) {
    samplingPeriod = myFile.parseInt(); Serial.print(" Sampling period set to "); Serial.print(samplingPeriod); Serial.println(" minutes"); delay(1000);

    myFile.close();
  }

  else {
    Serial.println("error opening data file");
  }

  Serial.println(" ");


  //TIMESTAMPED FILENAME CREATION

  Serial.print("Creating data file"); delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000); Serial.print(". "); delay(1000);
  getFileName();
  Serial.println(filename);
  
  
  
  delay(3000);


  // You must already have a plain text file file named ‘datalog.txt’ on the SD already for this to work!


  //————-print a header to the data file———-
  File nameFile = SD.open(filename, FILE_WRITE);
  if (nameFile) { // if the file is available, write to it:
    nameFile.println("Timestamp, DS3231 Temp(C), TMP102 Temp (C), Battery Voltage (mV)");
    //I often print many extra lines of text in file headers, identifying details about the hardware being used, the code version that was running, etc
    nameFile.close();
  }
  else {
    Serial.print("error opening "); Serial.println(filename);// if the file isn’t open, pop up an error:
  }

  pinMode(RED_PIN, OUTPUT); //configure 3 RGB pins as outputs
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH); // startup with green led lit
  digitalWrite(BLUE_PIN, LOW);

  Serial.println("Logger successfully started..."); delay(100);
  Serial.println("--------------------------------------------------"); delay(1000);

} // end of setup


// ################################################  THE LOOP   ################################################

void loop() {
  //—–This part reads the time and disables the RTC alarm
  DateTime now = RTC.now(); //this reads the time from the RTC
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  //loads the time into a string variable
  //don’t record seconds in the time stamp because
  //the interrupt to time reading interval is <1s, so seconds are always ’00’

  // We set the clockInterrupt in the ISR, deal with that now:
  if (clockInterrupt) {
    if (RTC.checkIfAlarm(1)) {       //Is the RTC alarm still on?
      RTC.turnOffAlarm(1);              //then turn it off.
    }
    //print (optional) debugging message to the serial window if you wish
    //Serial.print("RTC Alarm on INT-0 triggered at ");
    //Serial.println(CycleTimeStamp);
    clockInterrupt = false;                //reset the interrupt flag to false
  }//—————————————————————–
  // read the RTC temp register and print that out
  // Note: the DS3231 temp registers (11h-12h) are only updated every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);                     //the register where the temp data is stored
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  if (Wire.available()) {
    tMSB = Wire.read();            //2’s complement int portion
    tLSB = Wire.read();             //fraction portion
    temp3231C = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;  // Allows for readings below freezing: thanks to Coding Badly
    temp3231F = (temp3231C * 1.8) + 32.0; // To Convert Celcius to Fahrenheit

  }
  else {
    temp3231C = 0;
    //if temp3231C contains zero, then you know you had a problem reading the data from the RTC!
  }






  //Read TMP102 sensor

  TEMP_Raw = readTMP102();
  TEMP_degC = TEMP_Raw * 0.0625;




  // You could read in other variables here …like the analog pins, etc,
  // and just add them to the dataString before you write it to the file



  Vcc = readExternalVcc(); //get voltage

  //——– concatenate data into a string ———-
  // Add each piece of information to the string that gets written to the SD card with:dataFile.println(dataString);
  String dataString = ""; //this line simply erases the string
  dataString += CycleTimeStamp;
  dataString += ", ";     //puts a comma between the two bits of data
  dataString = dataString + String(temp3231C) + String(", ") + String(TEMP_degC) + String(", ") + String(Vcc);



  Serial.println(dataString);


  //——– Now write the data to the SD card ——–
  File nameFile = SD.open(filename, FILE_WRITE);// if the file is available, write to it:
  if (nameFile) {
    nameFile.println(dataString);
    nameFile.close();
  }
  else {
    Serial.print("error opening "); Serial.println(filename); // if the file isn’t open, pop up an error:
  }
  // delay(10000);
  // instead of using the delay we will use RTC interrupted sleeps

  //——– Set the next alarm time ————–
  Alarmhour = now.hour();
  Alarmminute = now.minute() + samplingPeriod;
  Alarmday = now.day();

  // check for roll-overs
  if (Alarmminute > 59) { //error catching the 60 rollover!
    Alarmminute = 0;
    Alarmhour = Alarmhour + 1;
    if (Alarmhour > 23) {
      Alarmhour = 0;
      // put ONCE-PER-DAY code here -it will execute on the 24 hour rollover
    }
  }
  // then set the alarm
  RTC.setAlarm1Simple(Alarmhour, Alarmminute);
  RTC.turnOnAlarm(1);
  if (RTC.checkAlarmEnabled(1)) {
    //you would comment out most of this message printing
    //if your logger was actually being deployed in the field
    //Serial.print("RTC Alarm Enabled!");
    //Serial.print(" Next data point in : "); // Serial.print(" Going to sleep for : ");
    //Serial.print(samplingPeriod);
    //Serial.println(" minutes");
    //Serial.println();                                      //just adds a carriage return
  }
  delay(100); //this delay is only here so we can see the LED’s it is totally optional!
  //otherwise the entire loop would execute too fast for us to see it!
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
  // Turn on red led as our indicator that the Arduino is sleeping.
  // Note: Normally you would NOT leave an LED on like this during sleep! This is just so you can see what is going on..

  //——– sleep and wait for next RTC alarm ————–
  // Enable interrupt on pin2 & attach it to rtcISR function:
  attachInterrupt(0, rtcISR, LOW);
  // Enter power down state with ADC module disabled to save power:
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  //processor starts HERE AFTER THE RTC ALARM WAKES IT UP
  detachInterrupt(0); // immediately disable the interrupt on waking
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH); //Interupt woke processor, turn on green led

}// this is the END of the MAIN LOOP
// This is the Interrupt subroutine that only executes when the rtc alarm goes off

void rtcISR() {
  clockInterrupt = true;
}



/*******************************************************
   READ VCC          using internal 1.1 v  OR analog pin
 ********************************************************/
// from http://forum.arduino.cc/index.php/topic,15629.0.html and http://forum.arduino.cc/index.php?topic=88935.0

int readExternalVcc()
{
  int result;

  //10 bit resolution, returning integers from 0 to 1023
  // we have a simple equal value resistor divider supplying this pin so it can read Vraw above Vcc

  //the supply voltage via analog read from a 10k ohm resistor divider
  int avrgVraw = 0;
  int ActualRaw_mV = 0;
  int vRaw = 0;
  analogRead(A0);        //ignore the first reading
  delay(1);              //settling time
  vRaw = analogRead(A0); // read the value from the A0 pin
  avrgVraw = avrgVraw + vRaw;
  delay(1);
  vRaw = analogRead(A0);
  avrgVraw = avrgVraw + vRaw;
  delay(1);
  vRaw = analogRead(A0);
  avrgVraw = avrgVraw + vRaw;
  avrgVraw = avrgVraw / 3; //avrg of 3 readings
  float volts = (avrgVraw / resistorFactor) * referenceVolts ; // calculate the ratio
  result = (volts * 1000); // conv to millivolts




  return result;
}

/**********************************************
 * ERROR HANDLER "Houston we have a problem..."
 ***********************************************/
// more advanced debugging: http://forum.arduino.cc/index.php?topic=203282.0
void error()
{
  // if less than a day has passed, X seconds of flashing red light on error, otherwise just shut down immediately
  //byte Currentday;
  //DateTime now = RTC.now(); Currentday = now.day();
  //if(Currentday == Startday){
  for (int CNTR = 0; CNTR < 50; CNTR++) { //seconds of flashing red light on error = CNTR/2
    digitalWrite(RED_PIN, HIGH);
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
    digitalWrite(RED_PIN, LOW);
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
  }

  //}

  pinMode(RTCPOWER_PIN, INPUT);    //stop sourcing or sinking current
  digitalWrite(RTCPOWER_PIN, LOW); // driving this LOW FORCES to the RTC to draw power from the coin cell

#if defined POWERDOWN_PIN
  digitalWrite(POWERDOWN_PIN, HIGH);// driving this pin high shuts down the system if pololu power switch attached
  delay(10);// just in case it takes time to power down...
#endif

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); //error catch if pololu does not work.
  //sleepNwait4WDT(); //since we have gone to sleep without FIRST setting the wdt, this is sleep forever!

}


// ************************************************************************************************************
//   *  *   *  *  *  *  *  *  *  *  SENSOR FUNCTIONS  *  *  *  *  *  *  *  *  *  *  *  *  *
// ************************************************************************************************************

// ************************************************************************************************************
// I2C TMP102 TEMPERATURE SENSOR
// ************************************************************************************************************
//  writes 2 bytes to registers, instead of one, so not itegrated with I2C write byte functions yet!

#if defined(TS_TMP102)

#define TMP102_CFG_default_byte1 B01100001  // 12 bit rez WITH ShutDown bit turned ON
#define TMP102_CFG_default_byte2 B10100000  // just the defaults from pg 7
#define TMP102_OneShotBit B10000000         // one-shot by ORing D7 of CFG byte 1 to 1
// err1-byte pointer to write to tmp102 BEFORE reading back 2 bytes of data from that register
#define TMP102_TEMP_REG_pointer 0x00  // temperature register, read only, 16bits
#define TMP102_CONF_REG_pointer 0x01  // config register, read/write, 16 bits

void initTMP102()
{
#ifdef ECHO_TO_SERIAL
  Serial.println(F("Initializing TMP102 Temperature sensor..."));
  Serial.flush();
#endif

  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer);
  Wire.write(TMP102_CFG_default_byte1);  //Sets to 12bit, sd mode on
  Wire.write(TMP102_CFG_default_byte2);  //none of these settings matter in one shot mode, at my temperature range, but set them to defaults anyway
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("Initial control reg writing on TMP102 failed!"));
    error();
    errorflag = 0;
  }

  // set one-shot bit to "1" - starts a single conversion then sleeps the sensor
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer); // Select control register.
  Wire.write(TMP102_CFG_default_byte1 | TMP102_OneShotBit); // Start one-shot conversion 40μA during conv
  //Wire.write(TMP102_CFG_default_byte2);  //dont need to bother writing the second byte
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("OOPS! Problem setting OneSHOT bit on TMP102! "));
    error();
    errorflag = 0;
  }
  TEMP_Raw = readTMP102();

#ifdef ECHO_TO_SERIAL
  Serial.print(F("TMP102 initialized: First Raw read="));
  Serial.print(TEMP_Raw);
  Serial.println(F(" "));
#endif

}

int readTMP102()
{
  //float deg_c;
  errorflag = 0;
  // start by resetting the one shot bit back to zero
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer);
  Wire.write(TMP102_CFG_default_byte1);  //Sets to 12bit, sd mode on
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("TMP102 clearing OS bit in CFG reg failed..."));

#ifdef ECHO_TO_SERIAL  //if echo is on, we are in debug mode, and errors force a halt. 
    error();
#endif

    errorflag = 0;
  }
  // now seting the one-shot bit to "1" will start a single conversion
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer); // point at the control register.
  Wire.write(TMP102_CFG_default_byte1 | TMP102_OneShotBit); // ORing the bits together
  //Wire.write(TMP102_CFG_default_byte2);  //I don't need to bother writing the second byte?
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("OOPS! problem setting OneSHOT bit on TMP102"));

#ifdef ECHO_TO_SERIAL  //if echo is on, we are in debug mode, and errors force a halt. 
    error();
#endif

    errorflag = 0;
  }

  //delay(28);  OR:
  //setWTD_32ms();
  //sleepNwait4WDT();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  // conversion: 26ms according to the sheet
  // during the conversion the OS bit will temporarily read "0", then revert to "1" after the conversion so you could check for that

  Wire.beginTransmission(TMP102_ADDRESS); //now read the temp
  Wire.write(TMP102_TEMP_REG_pointer); // Select temperature register.
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("Can't set temp reg pointer on TMP102"));

#ifdef ECHO_TO_SERIAL  //if echo is on, we are in debug mode, and errors force a halt. 
    error();
#endif

    errorflag = 0;
  }
  Wire.requestFrom(TMP102_ADDRESS, 2);
  const byte TempByte1 = Wire.read(); // MSByte, should be signed whole degrees C.
  const byte TempByte2 = Wire.read(); // unsigned because I am not reading any negative temps
  const int Temp16 = (TempByte1 << 4) | (TempByte2 >> 4);    // builds 12-bit value
  //TEMP_degC = Temp16*0.0625;
  return Temp16;
}
#endif

//################################################  FUNCTIONS   ################################################

void getFileName() {

  DateTime now = RTC.now();

  filename[0] = (now.year() / 1000) % 10 + '0'; //To get 1st digit from year()
  filename[1] = (now.year() / 100) % 10 + '0'; //To get 2nd digit from year()
  filename[2] = (now.year() / 10) % 10 + '0'; //To get 3rd digit from year()
  filename[3] = now.year() % 10 + '0'; //To get 4th digit from year()
  filename[4] = now.month() / 10 + '0'; //To get 1st digit from month()
  filename[5] = now.month() % 10 + '0'; //To get 2nd digit from month()
  filename[6] = now.day() / 10 + '0'; //To get 1st digit from day()
  filename[7] = now.day() % 10 + '0'; //To get 2nd digit from day()
}

