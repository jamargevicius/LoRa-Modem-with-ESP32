/*  eSoilTX_rev1.ino

  Joem   started 10/29/22  (adapted from Random Nerd Tutorials and other sources)
    - uses RTC to wakeup (EXT0 = RTC SW (nInt) pin connected to pin 27 of ESP32; when goes LO, triggers wakeup).  
    - allows for time setting from NTP, or manual Serial Port entry
    - does not use LoRa.h --- all access to the LoRa modem is done via specific register access.  This allows for better understanding of the modem
    - has an 8 pole SPST switch bank for selecting options, as follows:
      
       // switches
      displayOLED         // switch 1  // allows saving a few mAmps by clearning the display and not displaying anything
      oneMinuteTesting    // switch 2  // allows sending a packet every minute, instead of the 1/hour
      wifiEnabled         // switch 3  // goes through setup of a wifi connection
      setTime             // switch 4  // allows time to be set using switch 5's setting
      useNTP              // switch 5  // On = use NTP; Off = use manual entry via a serial monitor
      useHighestPower;    // switch 6  // uses 20dbm output when HI; otherwise uses 17dbm out 
      unitID_1            // switch 7  // sets the 2 bit ID for 1 of 4 transmitters
      unitID_0            // switch 8  
      
 */

/**************************************************************************************
* ********************************* includes ******************************************
* ************************************************************************************/
// *** note on #includes: <xx.h> is in the Arduino install location, whereas "xx.h" is in the local place specified in preferences in the Arduino IDE
// e.g. #include <DS3231.h"> looks for this in C:\...\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.6\tools\sdk\include\config/DS3231.h"

#include "DS3231.h" // note: 2 other libraries exist:  #include "ds3231.h" from github, and DS3231_RTC downloaded in Arduino; use the DS3231 downloaded library 
#include <WiFi.h>
#include <NTPClient.h>
#include <time.h> 

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
//#include <LoRa.h>  -- not used (write to registers instead)

// Libraries for SD card
#include "FS.h"
#include "SD.h"  // library for file operation.

/**************************************************************************************
* ********************************* constants and object declaration *******************
* ************************************************************************************/
// LoRa stuff -- registers (77 of them!)
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_HOP_CHAN             0x1c  // read only: bit 6 on Rx = 1 if Header contains directive to use payload CRC ; check before checking payload CRC
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e  // set bit 2 = 1 to transmit CRC for payload... header will indicate this; check REG_HOP_CHAN to verify this. 
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// LoRa modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// LoRa PA config
#define PA_BOOST                 0x80

// LoRa IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define ISR_PREFIX ICACHE_RAM_ATTR

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

 // LoRa Modem Configuration
#define RX_PAYLOAD_CRC_ON        0x04  // sets CRC enable in Tx (not used in Rx)
#define MAX_PKT_LENGTH           255

// LoRa misc stuff
#define LoRa_nCS 5  // chip select
#define LoRa_nRst 17 
#define LoRa_dio0 16   // could be used for an ISR, (needed here for LoRa.setPins(LoRa_CS, LoRa_rst, LoRa_dio0);)
int pktCounter = 0;
String unitNumber = "Tx1"; // put unit number here

  // the 8 switches
bool displayOLED;        // switch 1  GPIO 26
bool oneMinuteTesting;   // switch 2  GPIO 25
bool wifiEnabled;        // switch 3  GPIO 33
bool setTime;            // switch 4  GPIO 32
bool useNTP;             // switch 5  GPIO 35
bool useHighestPower;    // switch 6  GPIO 34 // uses 20dbm output when HI; otherwise uses 17dbm out
bool unitID_1;           // switch 7  GPIO 39
bool unitID_0;           // switch 8  GPIO 36

#define LEDbuiltin 2          // LED on ESP32 board - connected to pin 2

 // Internet connection
const char* ssid     = "xx";
const char* password = "yy";

 // NTP stuff
time_t rightNow;    // the UNIX timestamp, the epoche. This variable holds the seconds since January 1st, 1970:
tm timeinfo;        // the structure that holds time information in a more convenient way
#define MY_NTP_SERVER "pool.ntp.org"  // time server

#define MY_TZ "PST8PDT,M3.2.0,M11.1.0" // M3.2.0,M11.1.0 = (for Menlo Park CA): March, 2nd week, Sunday; November, 1st week, Sunday 
  // choose your time zone from this list .. ref: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
  //America/New_York      EST5EDT,M3.2.0,M11.1.0
  //America/Chicago       CST6CDT,M3.2.0,M11.1.0
  //America/Denver        MST7MDT,M3.2.0,M11.1.0
  //America/Los_Angeles   PST8PDT,M3.2.0,M11.1.0

 // RTC Time stuff
DS3231 myRTC;
#define DS3231_I2C_ADDRESS 0x68
String formattedDate; // Global Variables to save date and time
String dayStamp;
String timeStamp;
byte yearRTC;  // make these global for use in all functions
byte monthRTC;
byte dayRTC;
byte dayOfWeekRTC;
byte hourRTC;
byte minuteRTC;
byte secondRTC;

 // RTC Alarm Stuff *********** : Assign parameter values for Alarm 1
byte alarmDay = 0;
byte alarmHour = 0;
byte alarmMinute = 5;   // for when using 1 hour or more
byte alarmSecond = 14;  // when using 1 minute or more; it will take a 1sec delay on Serial Monitor, 2 sec for WiFi if used on reboot

//byte alarmBits = 0b00001111; // 1 1 1 1 = Alarm 1 every second (see Table 2 at end of this program, below)
//byte alarmBits = 0b00001110; //  1 1 1 0 = Alarm when seconds match --> triggers every 1 minute (see below)
//byte alarmBits = 0b00001100; //  1 1 0 0 Alarm when minutes and seconds match  --> triggers every hour
byte alarmBits; //  will define how often the alarm is set for. 

bool alarmDayIsDay = false; // 
bool alarmH12 = false;
bool alarmPM = false;  

 // ESP Sleep stuff **********************
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex  -- used for EXT1 only
RTC_DATA_ATTR int packetNum = 0;   // stored in internal RTC; new value will be stored each reboot
#define interruptPin GPIO_NUM_27
 
 // BME280 stuff
float tempC; String tempF;
float pres; 
float humid;
float alt;
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

 //  BH1750 stuff 
BH1750 lightMeter;

 // Soil Moisture sensor stuff
const int moisture1 = 13;  // pin GPIO-13 = ADC2 ch 4
float soilMoistureVolts = 0;
float soilMoisturePercent = 0;
float correctionFactor = 1.076;

 // Soil Temperature sensor - DS18B20
const int oneWireBus = 4;  // connected to GPIO-4    
OneWire oneWire(oneWireBus); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 
String soilTemp = ""; // Handle received and sent messages

 // OLED stuff  see https://randomnerdtutorials.com/guide-for-oled-display-with-arduino/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // i2c interface used 


/**************************************************************************************
* ********************************* functions *****************************************
* ************************************************************************************/
byte decToBcd(byte val) {  // Convert normal decimal numbers to binary coded decimal - used to program RTC
  return( (val/10*16) + (val%10) );
  // e.g. val = 10 decimal. BCD = 10d/10d * 16d = 16d = 00010000 binary = 0001 0000 BCD (why byte val ?  = 1010)
}

byte bcdToDec(byte val) {   // Convert binary coded decimal to normal decimal numbers
  return( (val/16*10) + (val%16) );
}

void setRTCtime(byte yearRTC, byte monthRTC, byte dayRTC, byte dayOfWeekRTC, byte hourRTC, byte minuteRTC, byte secondRTC){
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(secondRTC)); // set seconds
  Wire.write(decToBcd(minuteRTC)); // set minutes
  Wire.write(decToBcd(hourRTC)); // set hours
  Wire.write(decToBcd(dayOfWeekRTC)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayRTC)); // set date (1 to 31)
  Wire.write(decToBcd(monthRTC)); // set month
  Wire.write(decToBcd(yearRTC)); // set year (0 to 99)
  Wire.endTransmission();
}

void readRTCtime(byte *secondRTC,byte *minuteRTC,byte *hourRTC,byte *dayOfWeekRTC,byte *dayRTC,byte *monthRTC,byte *yearRTC){
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);// request seven bytes of data from DS3231 starting from register 00h
  *secondRTC    = bcdToDec(Wire.read() & 0x7f);  
  *minuteRTC    = bcdToDec(Wire.read());
  *hourRTC      = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeekRTC = bcdToDec(Wire.read());
  *dayRTC       = bcdToDec(Wire.read());
  *monthRTC     = bcdToDec(Wire.read());
  *yearRTC      = bcdToDec(Wire.read());
}

String readNdisplayRTCtime(){  // uses global definitions of secondRTC etc. ... easier to call later without arguments
  byte secondRTC, minuteRTC, hourRTC, dayOfWeekRTC, dayRTC, monthRTC, yearRTC;
  String stringSecondRTC, stringMinuteRTC, stringHourRTC,stringDayRTC,stringMonthRTC;

  readRTCtime(&secondRTC, &minuteRTC, &hourRTC, &dayOfWeekRTC, &dayRTC, &monthRTC, &yearRTC);   // retrieve data from DS3231
  Serial.print("...time just read from RTC is: ");
  Serial.print("20");Serial.print(yearRTC, DEC); Serial.print(" ");
  if (monthRTC<10)  {stringMonthRTC = String("0") + String(monthRTC);} else {stringMonthRTC = String(monthRTC);}
  Serial.print(stringMonthRTC); Serial.print("-");
  if (dayRTC<10)  {stringDayRTC = String("0") + String(dayRTC);} else {stringDayRTC = String(dayRTC);} 
  Serial.print(stringDayRTC); Serial.print(" @ ");
  if (hourRTC<10)  {stringHourRTC = String("0") + String(hourRTC);} else {stringHourRTC = String(hourRTC);} 
  Serial.print(stringHourRTC); Serial.print(":");  
  if (minuteRTC<10)  {stringMinuteRTC = String("0") + String(minuteRTC);} else {stringMinuteRTC = String(minuteRTC);} 
  Serial.print(stringMinuteRTC); Serial.print(":");
  if (secondRTC<10)  {stringSecondRTC = String("0") + String(secondRTC);} else {stringSecondRTC = String(secondRTC);} 
  Serial.print(stringSecondRTC);
  Serial.print(" - Day of week: ");
  switch(dayOfWeekRTC){
    case 1:
      Serial.println("Sunday");
      break;
    case 2:
      Serial.println("Monday");
      break;
    case 3:
      Serial.println("Tuesday");
      break;
    case 4:
      Serial.println("Wednesday");
      break;
    case 5:
      Serial.println("Thursday");
      break;
    case 6:
      Serial.println("Friday");
      break;
    case 7:
      Serial.println("Saturday");
      break; }
   String timeNowString = stringMonthRTC + "/" + stringDayRTC + "/" + String(yearRTC) + " @ " + stringHourRTC + ":" + stringMinuteRTC + ":" + stringSecondRTC;
   return timeNowString; 
}

void getNsetTimeFromTerminal() {      // the order YYMMDDwHHMMSSx  -- put an x at the end e.g. 2210161073400x
    boolean gotString = false;
    char inChar;
    byte temp1, temp2;
    char inString[15] = "00000000000000";  // initialize the 14 character entry as 15; last character inserted = \0 = null
    byte j=0;

    while (!gotString) {
        if (Serial.available()) {
            inChar = Serial.read();
            inString[j] = inChar;
            j += 1;
            if (inChar == 'x' ) {  // supposedly, the null character = end of string, but there isn't one found. 
                gotString = true;
                inChar = Serial.read();   }  }  }  // read the last character = \0 = null
    
    Serial.print("just entered "); Serial.println(inString);

     // now define year, month, etc, - for year 22, (byte)inString[0] = 50 (= decimal).  So, 50 - 48 = 2
    temp1 = (byte)inString[0] - 48;  // year
    temp2 = (byte)inString[1] - 48;
    byte yearRTC = temp1*10 + temp2;
    temp1 = (byte)inString[2] -48;   // month
    temp2 = (byte)inString[3] -48;
    byte monthRTC = temp1*10 + temp2;
    temp1 = (byte)inString[4] -48;   // day
    temp2 = (byte)inString[5] -48;
    byte dayRTC = temp1*10 + temp2;
    byte dayOfWeekRTC = (byte)inString[6] - 48;
    temp1 = (byte)inString[7] -48;   // hour
    temp2 = (byte)inString[8] -48;
    byte hourRTC = temp1*10 + temp2;
    temp1 = (byte)inString[9] -48;   // minute
    temp2 = (byte)inString[10] -48;
    byte minuteRTC = temp1*10 + temp2;
    temp1 = (byte)inString[11] -48;  // second
    temp2 = (byte)inString[12] -48;
    byte secondRTC = temp1*10 + temp2;

    setRTCtime(yearRTC, monthRTC, dayRTC, dayOfWeekRTC, hourRTC, minuteRTC, secondRTC);
}

void getNsetTimeFromNTP() {   
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){Serial.println("Failed to obtain time"); return; }
  Serial.print("Time to be written into RTC from NTP: "); Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  byte yearRTC = timeinfo.tm_year - 100; //Serial.print("year is: "); Serial.println(yearRTC);
  byte monthRTC = timeinfo.tm_mon + 1;  //Serial.print("month is: "); Serial.println(monthRTC);
  byte dayRTC = timeinfo.tm_mday;  //Serial.print("day is: "); Serial.println(dayRTC);
  byte dayOfWeekRTC = timeinfo.tm_wday + 1;  //Serial.print("dayOfWeek is: "); Serial.println(dayOfWeekRTC);
  byte hourRTC = timeinfo.tm_hour;  //Serial.print("hour is: "); Serial.println(hourRTC);
  byte minuteRTC = timeinfo.tm_min;  //Serial.print("minutes are: "); Serial.println(minuteRTC);
  byte secondRTC = timeinfo.tm_sec;  //Serial.print("seconds are: "); Serial.println(secondRTC);
 
  setRTCtime(yearRTC, monthRTC, dayRTC, dayOfWeekRTC, hourRTC, minuteRTC, secondRTC);
}

  //************************* LoRa section of functions
char readLoRaReg(uint8_t address){
  char response;
  digitalWrite(LoRa_nCS, LOW);
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));   // 200,000 Hz = default SPI frequency
  SPI.transfer(address & 0x7f); // read has MSbit lo
  response = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(LoRa_nCS, HIGH);
  return response; 
}

void writeLoRaReg(uint8_t address, uint8_t value){
  digitalWrite(LoRa_nCS, LOW);
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));   // 200,000 Hz = default SPI frequency
  SPI.transfer(address | 0x80); // write has MSbit hi
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(LoRa_nCS, HIGH);
}

void reinitializeLoRaModem(){
    // start of LoRa.begin(915E6)
  SPI.begin();  // start the SPI interface (LoRa Modem will always have this i/f active, even in sleep mode) 
  delay(100);   // need a little time for SPI to settle down

  pinMode(LoRa_nCS, OUTPUT); digitalWrite(LoRa_nCS, HIGH);  
  pinMode(LoRa_nRst, OUTPUT); digitalWrite(LoRa_nRst, HIGH); delay(10);
  digitalWrite(LoRa_nRst, LOW); delay(10); digitalWrite(LoRa_nRst, HIGH); delay(10);  // resets regs, and puts in standby mode
  delay(500);  /// delay for the RC ckt
    
   // LoRa Modem setup --- must be in sleep (not standby) to configure modem; note: in sleep mode, the Fifo data is cleared, so read it before this
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);  // must be put in sleep mode (not standby !! doesn't work!) to be able to write to config frequency
                // note: Sleep mode = only SPI i/f active;  Standby mode = Sleep + RC osc on + xtal on;  RX mode = everything on
                // note: FIFO memory is cleared when put LoRa is put to sleep !

   // start of setFrequency(915E6)
    // configure the LoRa Modem frequency -- must be done in standby (or sleep mode) (from begin() in LoRa.cpp)
  long frequencyLoRa = 915E6; // 915 Mhz for North America  // this is setFrequency(frequency) in LoRa.cpp
  uint64_t frf = ((uint64_t)frequencyLoRa << 19) / 32000000;
  writeLoRaReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeLoRaReg(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeLoRaReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
  writeLoRaReg(REG_SYNC_WORD, 0xFA);                  //sync word: ranges from 0-0xFF
    // end of setFrequency(915E6) ... continue with LoRa.begin(915E6)

  writeLoRaReg(REG_FIFO_TX_BASE_ADDR, 0); // set base addresses
  
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);    // move from sleep to standby mode (RC osc and xtal on)
    // end of LoRa.begin(915E6)

  
    // setup the Amplifier section - the RMF95 module is wired to the boost amp of the RF96 chip; this code follows LoRa.setTxPower(xx)
  int PA_DACvalue; int OCPtrim; int powerLevel;
  if(useHighestPower){Serial.println("Pout = 20dbm");PA_DACvalue = 0x87; OCPtrim = 0x11; powerLevel = 17;}      // PA_DAC provides extra 3 db --> power = 20dbm
  else               {Serial.println("Pout = 17dbm");PA_DACvalue = 0x84; OCPtrim = 0x0B; powerLevel = 17;}      // PA_DAC doesn't provide extra db --> power = 17dbm
  writeLoRaReg(REG_PA_DAC, PA_DACvalue);
  writeLoRaReg(REG_OCP, 0x20 | (0x1F & OCPtrim)); // this is LoRa.setOCP(value in mA);
  writeLoRaReg(REG_PA_CONFIG, PA_BOOST | (powerLevel - 2)); // 0x8F = PA_BOOST | (powerLevel - 2)

    /* output power vs. current consumption
      RFOP = +20 dBm, on PA_BOOST - 120mA
      RFOP = +17 dBm, on PA_BOOST - 87 mA
      RFOP = +13 dBm, on RFO_LF/HF pin - 29 mA
      RFOP = + 7 dBm, on RFO_LF/HF pin - 10 mA
    */

  writeLoRaReg(REG_MODEM_CONFIG_2, (readLoRaReg(REG_MODEM_CONFIG_2) | RX_PAYLOAD_CRC_ON)); // configure Tx to indicate use of payload CRC in the header (header CRC is always on) 
    // end of LoRa.begin(long frequency)

    // begin LoRa.beginPacket() ********************* is any of this necessary ?????
  writeLoRaReg(REG_MODEM_CONFIG_1, readLoRaReg(REG_MODEM_CONFIG_1) & 0xfe); //explicitHeaderMode();  // necessary ???
  writeLoRaReg(REG_FIFO_ADDR_PTR, 0); // reset FIFO address and paload length
  writeLoRaReg(REG_PAYLOAD_LENGTH, 0); // reset payload length = 0
    // end LoRa.beginPacket()

  Serial.println("LoRa Initialized OK");
} // end of reinitializeLoRaModem()

void setupNclearOLED() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    while(1) {Serial.print("...can't connect to OLED display"); delay(1000);} } // Don't proceed, loop forever}
  display.clearDisplay();
  display.setTextSize(1);  // supports 1 to 8  1 = 10 pixels high
  display.setTextColor(WHITE);
  display.display();     // implements display info
}

float *BME280msmt() {  // creates a list, returns the pointer to the list
  static float  msmt[4];
  msmt[0] = bme.readTemperature();
  msmt[1] = bme.readPressure() / 101325 ;  // 1013.25hPa/atm; add'l /100 to convert reading to hPa
  msmt[2] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  msmt[3] = bme.readHumidity();
  return msmt;
}

/**************************************************************************************
* ************************************** setup ***** **********************************
* *************************************************************************************/
void setup(){  // This runs EVERY TIME ESP32 wakes up !!!
  Serial.begin(115200);
  while (!Serial);          // stay here until Serial port is setup 
  delay(1000);              // delay a little to allow for getting the terminal up. 
  Wire.begin();             // Start the I2C interface: used on DS3231(RTC), OLED, BH1750(lux)and HTU21 (or BME280)
  delay(100);               // slight delay to setup
  pinMode(LEDbuiltin, OUTPUT);                // Use builtin LED to blink - need to restabalish this after a wakeup-reboot
  pinMode(interruptPin, INPUT);          // this is the line from RTC to ESP32 - need to restabalish this after a wakeup-reboot
  digitalWrite(LEDbuiltin, true);      // start  with LED on
  delay(100);  
  
   // GPIO and switches
  pinMode(26, INPUT);                 // switch 1  GPIO 26 = displayOLED  
  displayOLED = digitalRead(26);          //Serial.print("displayOLED = "); Serial.println(displayOLED);
  pinMode(25, INPUT_PULLUP);          // switch 2  GPIO 25 = oneMinuteTesting 
  oneMinuteTesting = digitalRead(25);     //Serial.print("oneMinuteTesting = "); Serial.println(oneMinuteTesting);
  pinMode(33, INPUT_PULLUP);          // switch 3  GPIO 33 = wifiEnabled 
  wifiEnabled = digitalRead(33);          //Serial.print("wifiEnabled = "); Serial.println(wifiEnabled);
  pinMode(32, INPUT_PULLUP);          // switch 4  GPIO 32 = setTime 
  setTime = digitalRead(32);              //Serial.print("setTime = "); Serial.println(setTime);
  pinMode(35, INPUT);                 // switch 5  GPIO 35 = useNTP 
  useNTP = digitalRead(35);               //Serial.print("useNTP = "); Serial.println(useNTP);
  pinMode(34, INPUT);                 // switch 6  GPIO 34 = useHighestPower
  useHighestPower = digitalRead(34);     //Serial.print("useHighestPower = "); Serial.println(useHighestPower);
  pinMode(39, INPUT);                 // switch 7  GPIO 39 = unitID_1 
  unitID_1 = digitalRead(39);             //Serial.print("unitID_1 = "); Serial.println(unitID_1);
  pinMode(36, INPUT);                 // switch 8  GPIO 36 = unitID_0 
  unitID_0 = digitalRead(36);             //Serial.print("unitID_0 = "); Serial.println(unitID_0);
    
      
   // ***************** RTC setup - program time into RCT if required ****************
  myRTC.setClockMode(false);  // set to 24h; use setClockMode(true); for 12h
  Serial.print("\n...just woke up: ");

  if(wifiEnabled){
      Serial.print("Connecting to "); Serial.println(ssid); // Connect to Wi-Fi network with SSID and password
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {delay(500); Serial.print(".");  }
      Serial.print("\n WiFi connected. ...IP address: "); Serial.println(WiFi.localIP());}   // Print local IP address

  if(setTime){  // assumes that WiFi is enabled if using NTP
    if(useNTP){
      if(!wifiEnabled) {
        setupNclearOLED();
        while(true) {
          display.setCursor(0,00); display.print("enable WiFi"); 
          display.setCursor(0,11); display.print("then hit Reset"); 
          display.setCursor(0,22); display.print("");
          display.display();     // implements display info
          delay(600);
          display.clearDisplay();
          display.setCursor(0,33); display.print("enable WiFi");      
          display.setCursor(0,44); display.print("then hit Reset");     
          display.setCursor(0,55); display.print("");       
          display.display();     // implements display info
          delay(600);
          display.clearDisplay();
          display.display();     // implements display info
          Serial.println("WiFi is not enabled - can't use NTP - change switches");}}
      else {  // now use NTP
        Serial.println("...programming RTC using NTP");
        configTime(0, 0, MY_NTP_SERVER); // 0, 0 because we will use TZ in the next line
        setenv("TZ", MY_TZ, 1);          // Set environment variable with your time zone
        tzset();
        delay(1000); // delay needed to get 1st time correct  
        getNsetTimeFromNTP();}} 
    else { // Program RTC time using Serial Terminal
      Serial.println("Waiting for manual entry of time from terminal - format: YYMMDDwHHMMSSx, eg. 2210161073400x");
      while(!Serial.available()){;}  // stay here until there is a terminal entry
      getNsetTimeFromTerminal(); }}
  else {Serial.println("...using RTC-stored time");}

    // ***************  Set the RTC Alarm, and enable interrupts  *************
  myRTC.turnOffAlarm(1);
     //if(oneMinuteTesting){byte alarmBits = 0b00001110; ---> doesn't work!!  remove "byte"
  if(oneMinuteTesting){alarmBits = 0b00001110; Serial.println("1 minute testing");} // every 1 minute: 1 1 1 0 = Alarm when seconds match --> triggers every 1 minute
  else {alarmBits = 0b00001100; Serial.println("1 hour testing");}                // every hour: 1 1 0 0 Alarm when minutes and seconds match  --> triggers every hour
  myRTC.setA1Time(alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits, alarmDayIsDay, alarmH12, alarmPM);
  myRTC.turnOnAlarm(1);    // enable Alarm 1 interrupts
  myRTC.checkIfAlarm(1);   // clear Alarm 1 flag

   // deal with Alarm 2:  Program it to prevent it covertly blocking the outgoing interrupt signal (advise from blogger).
  alarmMinute = 0xFF;      // a value that will never match the time
  alarmBits = 0b01100000;  // Alarm 2 when minutes match, i.e., never
  myRTC.setA2Time(alarmDay, alarmHour, alarmMinute, alarmBits, alarmDayIsDay, alarmH12, alarmPM);
  myRTC.turnOffAlarm(2);   // disable Alarm 2 interrupt
  myRTC.checkIfAlarm(2);   // clear Alarm 2 flag - both of the alarm flags must be cleared to enable output of a FALLING interrupt
  
  String timeNow = readNdisplayRTCtime(); // need for OLED display

  ++packetNum;  //Increment boot number and print it every reboot
  Serial.println("....");
  Serial.println("Packet number: (i.e. reboot number): " + String(packetNum));
  Serial.println("....");

    // BH1750 stuff
  lightMeter.begin();  //Serial.println(F("BH1750 setup"));
  delay(100); // slight delay to setup
  //float luxLevel = lightMeter.readLightLevel();
  int luxLevel = lightMeter.readLightLevel();  // only want integer, not decimal

   // BEM280 stuff
  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1){
      Serial.println("Still can't find a valid BME280 sensor");
      delay(1000);} }
  // Serial.println("BME280 Initialized OK!");  Serial.println();
  float *bmeValues; // a pointer
  bmeValues = BME280msmt();
  tempC = *bmeValues;
  //tempF = String(1.8 * tempC + 32);
  int tempF = 1.8 * tempC + 32;
  pres =  *(bmeValues + 1);
  alt =   *(bmeValues + 2);
  int airHumid = *(bmeValues + 3);
  
   // DS18B20 Soil Temp sensor stuff
  sensors.begin();   // Start the DS18B20 sensor
  delay(100); // slight delay to setup
  sensors.requestTemperatures(); 
  //temperatureString = String(sensors.getTempCByIndex(0)) + "C  " +  String(sensors.getTempFByIndex(0)) + "F";
  soilTemp = String(int(sensors.getTempFByIndex(0)));
  //soilTemp = String(sensors.getTempCByIndex(0));
  //Serial.print("soil temperature (F) is: "); Serial.println(soilTemp);

   // Soil Moisture sensor stuff
  int wetnessSoil;
  soilMoistureVolts = 3.322 * correctionFactor * analogRead(moisture1) / 4096;      // Reading sensor value
  // Serial.print("soil moisture in volts: "); Serial.println(soilMoistureVolts);
  if(soilMoistureVolts > 2.6)        {Serial.println("> 2.6;   wetness = 1 -- [VERY DRY, dry, Wet, Very Wet]"); wetnessSoil = 1;}
  else if (soilMoistureVolts > 2.4)  {Serial.println("> 2.4;   wetness = 2 -- [very dry, DRY, wet, very wet]"); wetnessSoil = 2;}
  else if (soilMoistureVolts > 2.0)  {Serial.println("> 2.0;   wetness = 3 -- [very dry, dry, WET, very wet]"); wetnessSoil = 3;}
  else                               {Serial.println("< 2.0;   wetness = 4 -- [very dry, dry, wet, VERY WET]"); wetnessSoil = 4;}

  delay(100); // slight delay to get data

   // LoRa stuff
  reinitializeLoRaModem();  // sets Frequency, etc. and ends in standby

   // format message to send out
  String tempString = "#" + String(unitNumber) + ", Soil T(F):," + String(soilTemp) + ", Soil Moisture(V):," + String(soilMoistureVolts) + ", Soil wetness[4..1]:,"  + String(wetnessSoil);
  String toSend = tempString + ", Air T(F):," + String(tempF) + ", Air Humidity(%):," + String(airHumid)+ ", Light(lux):," + String(luxLevel) + ", pkt#," + String(packetNum) + ", #" + String(unitNumber);
  Serial.print(" packet to send: "); Serial.println(toSend);
  
    /**** case where there is no soil moisture or temp sensore ---- used for debug
    String toSend="#"+String(unitNumber)+", Air T(F):,"+String(tempF)+", Air Humid(%):,"+String(airHumid)+", Light(lux):,"+String(luxLevel)+", pkt#,"+String(packetNum)+",#"+String(unitNumber);
    */
   // write the packet into the Fifo, and specify how big it is
  for (int i = 0; i < strlen(toSend.c_str()); i++) {writeLoRaReg(REG_FIFO, toSend[i]);} // write data - in place of LoRa.print(toSend)
  writeLoRaReg(REG_PAYLOAD_LENGTH, strlen(toSend.c_str()));  // must manually define how big the data was that just wrote to Fifo
  
      //LoRa.endPacket(); --- not "end" ... it's really "send" -- switches to Tx Mode (which sends the packet), waits until done, then clears interrupts
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX); // send the packet by puting in TX mode (part of LoRa.endPacket() - not ending the packet !!! ... sending it!!)
  while((readLoRaReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) { yield(); } //Serial.print("Waiting for TxDone\n"); delay(500);
  writeLoRaReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);  // clear TxDone Flag
  
    
    // OLED Display
    //(column,row)-in pixels, where 1 row = 21 characters across (128 pix/columns across = 6 pix/char width); and 6 lines = 64 pix/rows down = 10.7 pix/line)
  setupNclearOLED();
  display.setCursor(0,00); display.print("packet # = ");         display.setCursor(67,00); display.print(packetNum);  // 12char x 6  = 72
  display.setCursor(0,11); display.print(timeNow); 
  //display.setCursor(0,22); display.print("LED after wakeup:");   display.setCursor(108,22); display.print(bool(state));
  display.setCursor(0,22); display.print("airTemp(F) = ");         display.setCursor(78,22); display.print(int(tempF));
  display.setCursor(0,33); display.print("airHumid(%) = ");      display.setCursor(84,33); display.print(int(airHumid));
  //display.setCursor(0,44); display.print("airPres(atm) = ");     display.setCursor(90,44); display.print(pres);
  display.setCursor(0,55); display.print("Light(lux) = ");        display.setCursor(78,55); display.print(luxLevel);
  display.display();     // implements display info

  delay(2000);             // give time to see the LED and display on
  
   // OLED display during sleep -- save ~0.1 mA while asleet
  if(!displayOLED){setupNclearOLED();} // leave it cleared
  
   // clear Alarm flags and go to sleep
  myRTC.checkIfAlarm(1);   // Clear Alarm 1 flag
  myRTC.checkIfAlarm(2);   // clear Alarm 2 flag - both of the alarm flags must be cleared to enable output of a FALLING interrupt

  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);   // put the modem to sleep during ESP32 sleep

  esp_sleep_enable_ext0_wakeup(interruptPin,0); // Enable Ext0 as wakeup/reboot: Logic Level defined = 0 = interrupt; must do this after a reboot
  Serial.println("Going to sleep now ... Waiting for RTC alarm going Lo to wake up the ESP32\n\n");
  Serial.flush(); 
  esp_deep_sleep_start();  // ESP32 goes to sleep
  Serial.println("This will never be printed");
}


/**************************************************************************************
* ************************************** main loop() **********************************
* ************************************************************************************/
void loop(){
  //This is not going to be called
}















/***************** useful stuff **********
setTxPower()
void RH_RF95::setTxPower   (int8_t  power, bool useRFO = false )     

Sets the transmitter power output level, and configures the transmitter pin. Be a good neighbour and set the lowest power level you need. 
Some SX1276/77/78/79 and compatible modules (such as RFM95/96/97/98) use the PA_BOOST transmitter pin for high power output 
(and optionally the PA_DAC) while some (such as the Modtronix inAir4 and inAir9) use the RFO transmitter pin for lower power 
but higher efficiency. You must set the appropriate power level and useRFO argument for your module. 
Check with your module manufacturer which transmtter pin is used on your module to ensure you are setting useRFO correctly. 
Failure to do so will result in very low transmitter power output. Caution: legal power limits may apply in certain countries. 
After init(), the power will be set to 13dBm, with useRFO false (ie PA_BOOST enabled).

Parameters
    power: power Transmitter power level in dBm. 
      For RFM95/96/97/98 LORA with useRFO false,  valid values are from +2 to +20. For 18, 19 and 20, PA_DAC is enabled,
      For Modtronix inAir4 and inAir9 with useRFO true (ie RFO pins in use), valid values are from 0 to 15.
      
    useRFO: If true, enables the use of the RFO transmitter pins instead of the PA_BOOST pin (false). Choose the correct setting for your module. 

*/
