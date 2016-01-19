# OpenOceanDataLogger

This is the source code for the Arduino Data logger using the TMP102 i2C temperature sensor.
The datalogger uses the DS3231 RTC for timestamping and 
Data is logged to an SD card at a user-programmed sampling interval (between 1-59 minutes).
Logger powered by  3x 1.5v AA batteries.



//////////////////  DATA  ///////////////////////////////

Temperature data read from the onbaord DS3231 sensor (+/-3C) and TMP102 sensor (accurate to 0.5C)
Data is timestamped to the nearest minute (eg. 2016/01/19 09:00) using RTC
Saved in a headered text file in the format : Timestamp, DS3231 Temp(C), TMP102 Temp (C), Battery Voltage (mV)
Text file named using the 8.3 filename format using dynamic date time. e.g 20160119.txt



//////////////////  ALARMS  ///////////////////////////////

Logger reads RTC and checks if alarm is ON, if it is ON then it turns alram OFF and samples DS3231 and TMP102 sensors and reads vcc.
It then sets a next alarm time using the RTC + sampling interval (1-59 minutes)
RTC then turns alarm back ON
ADC module is powered down to conserve energy



//////////////////  LED STATUS  ///////////////////////////////

LED green on start up
LED red when logger is sleeping



//////////////////  POWER  ///////////////////////////////

Logger is designed to operate as a stand-alone device running on 3 x 1.5v batteries in series.
Battery supply voltage is monitored using a voltage divider and reading an 3-sampling moving average on an analog input using a 10-bit resolution ADC.



/////////////////  USER CONFIGURATION  //////////////////////////

The sampling interval can be adjusted without the need for the field engineer to connect the logger to a computer.
Sampling interval settings saved to a config.txt file and pre-loaded onto the SD card.
What's in CONFIG.TXT: [int]   e.g. 10
