# OpenOceanDataLogger

This is the source code for the Arduino Data logger using the TMP102 i2C temperature sensor.
The datalogger uses the DS3231 RTC using the I2C protocol.
Data is logged to an SD card at a user-programmed sampling interval (between 1-59 minutes).


//////////////////  DATA  ///////////////////////////////

Data is timestamped to the nearest minute (eg. 2016/01/19 09:00)
Saved in a headered text file in the format : Timestamp, DS3231 Temp(C), TMP102 Temp (C), Battery Voltage (mV)
Text file named using the 8.3 name format using dynamic date time. e.g 20160119.txt


//////////////////  POWER  ///////////////////////////////

Logger is designed to operate as a stand-alone device running on 3 x 1.5v batteries in series.
Battery supply voltage is monitored using a voltage divider and reading an 3-sampling moving average on an analog input using a 10-bit resolution ADC.


/////////////////  CONFIGURATION  //////////////////////////

The sampling interval can be adjusted without the need for the field engineer to connect the logger to a computer.
Sampling interval settings saved to a config.txt file and pre-loaded onto the SD card.
What's in CONFIG.TXT: [int]   e.g. 10
