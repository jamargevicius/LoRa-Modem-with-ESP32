/*  eSoilRX_int_rev1.ino

    JoeM - 9/20/22 start  -- code adapted from Random Nerd Tutorials, and others
      desc - LoRa packet receiver, based on IRQ from LoRa (Rx Done on DI-0 pin of LoRa modem)
             - ESP32 is put to sleep, and wakes when the modem receives a packet 
             - uses an SD card to allow storage of packet data
             - filters the packet. Can collect info on bad header or payload CRCs, and bad content.
             - uses an 8 DIP switch to select various options. 
                  displayOLED;      // switch 1  allows saving a few mAmps if the OLED is blank and not displaying anything
                  write2SD;         // switch 2  allows writing (or not writing) to the SD card -- used in debugging
                  wifiEnabled;      // switch 3  allow for setup of the WiFi connection if WiFi is to be used
                  setTime;          // switch 4  allows for setting up the RTClock - done initially, per switch 5's selection
                  seNTP;            // switch 5  Hi = use the NTP server to set the RTC (requires WiFi); Lo = enter manually via a serial terminal
                  newSDfile;        // switch 6  for startup when no data.txt file exists on the SD card; must run, then switch off, then reset the ESP32
                  printSDdata;      // switch 7  for debugging when using the serial monitor; prints out the contents of the data.txt file
                  printSDerror;     // switch 8  same, but for printing out the error.txt file


*/

// *** note on #includes: <xx.h> is in the Arduino install location, whereas "xx.h" is in the local place specified in preferences in the Arduino IDE
// e.g. #include <DS3231.h"> looks for this in C:\Users\Asus-i7-4GHz-16GB\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.6/tools/sdk/include/config/DS3231.h"

#include "DS3231.h" // RTC module. note: 2 other libraries exist:  #include "ds3231.h" from github, and DS3231_RTC downloaded in Arduino; use the DS3231 downloaded library 
#include <WiFi.h>
#include <NTPClient.h>
#include <time.h> // time() ctime()

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
//#include <LoRa.h>  --- not used
// Libraries for SD card
#include "FS.h"
#include "SD.h"  // library for file operation.

/***********************************************************************
************************** definitions from LoRa.cpp ***************
***********************************************************************/
// registers
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

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06


 // Modem Configuration
#define RX_PAYLOAD_CRC_ON        0x04  // sets CRC enable in Tx (not used in Rx)
#define CRC_ON_PAYLOAD           0x40  // bit 6

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define ISR_PREFIX ICACHE_RAM_ATTR

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

/**************************************************************************************
* ********************************* constants and object declaration *******************
* ************************************************************************************/
  // the 8 switches
bool displayOLED;      // switch 1  GPIO 26
bool write2SD;         // switch 2  GPIO 25
bool wifiEnabled;      // switch 3  GPIO 33
bool setTime;          // switch 4  GPIO 32
bool useNTP;           // switch 5  GPIO 35
bool newSDfile;        // switch 6  GPIO 34 --- for startup when no data.txt file exists on the SD card; must run, then switch off, then reset the ESP32
bool printSDdata;      // switch 7  GPIO 39
bool printSDerror;     // switch 8  GPIO 36

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
byte yearRTC;  // make these global for use in all functions
byte monthRTC;
byte dayRTC;
byte dayOfWeekRTC;
byte hourRTC;
byte minuteRTC;
byte secondRTC;
String timeNow;

 // LoRa stuff -- needed to intialize the unit
String unitNumber = "Tx1";      // put unit number here -- $$$$$$$$$$$$$$$$$$$$$  read input switches to create this
const int LoRa_nCS = 5;         // LoRa radio chip select
const int LoRa_nRst = 17;       // LoRa radio reset
#define LoRa_RxDone GPIO_NUM_4  // must use an RTC_GPIO pin to cause wakeup from sleep; this is DIO-0 of LoRa modem
int pktCounter = 0;
String LoRaData;
String rssi;
float snr;
int packetIndex;
int irqFlags;
bool goodPayloadCRCpacket;
RTC_DATA_ATTR int packetNum = 0;   // stored in internal RTC; new value will be stored each reboot
RTC_DATA_ATTR int packetsMissed = 0;  // stored value

 // BME280 stuff
float tempC; String tempF;
float pres; 
float humid;
float alt;
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

 //  BH1750 stuff 
BH1750 lightMeter;

 // OLED stuff  see https://randomnerdtutorials.com/guide-for-oled-display-with-arduino/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // i2c interface used 

 // SD Card stuff
#define SCK  18
#define MISO  19
#define MOSI  23
#define SD_CS  15    
String dataToRecord;
String stringHeader;

//************************************************************************************
/**************************************************************************************
* ********************************* functions *****************************************
* ************************************************************************************/
//************************************************************************************
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
  //Serial.print("yearRTC in readRTCtime = "); Serial.println(String(*yearRTC));
}

String readNprintRTCtime(){  // uses global definitions of secondRTC etc. ... easier to call later without arguments
  byte secondRTC, minuteRTC, hourRTC, dayOfWeekRTC, dayRTC, monthRTC, yearRTC;
  String stringSecondRTC, stringMinuteRTC, stringHourRTC,stringDayRTC,stringMonthRTC;
  readRTCtime(&secondRTC, &minuteRTC, &hourRTC, &dayOfWeekRTC, &dayRTC, &monthRTC, &yearRTC);   // retrieve data from DS3231
  //Serial.println("\n...time just read from RTC is: ");

  //Serial.print("20");Serial.print(yearRTC, DEC); Serial.print(" ");
  if (monthRTC<10)  {stringMonthRTC = String("0") + String(monthRTC);} else {stringMonthRTC = String(monthRTC);}
  //Serial.print(stringMonthRTC); Serial.print("-");
  if (dayRTC<10)  {stringDayRTC = String("0") + String(dayRTC);} else {stringDayRTC = String(dayRTC);} 
  //Serial.print(stringDayRTC); Serial.print(" @ ");
  if (hourRTC<10)  {stringHourRTC = String("0") + String(hourRTC);} else {stringHourRTC = String(hourRTC);} 
  //Serial.print(stringHourRTC); Serial.print(":");  
  if (minuteRTC<10)  {stringMinuteRTC = String("0") + String(minuteRTC);} else {stringMinuteRTC = String(minuteRTC);} 
  //Serial.print(stringMinuteRTC); Serial.print(":");
  if (secondRTC<10)  {stringSecondRTC = String("0") + String(secondRTC);} else {stringSecondRTC = String(secondRTC);} 
  //Serial.print(stringSecondRTC);
  //Serial.print(" - Day of week: ");
  switch(dayOfWeekRTC){
    case 1:
      //Serial.println("Sunday");
      break;
    case 2:
      //Serial.println("Monday");
      break;
    case 3:
      //Serial.println("Tuesday");
      break;
    case 4:
      //Serial.println("Wednesday");
      break;
    case 5:
      //Serial.println("Thursday");
      break;
    case 6:
      //Serial.println("Friday");
      break;
    case 7:
      //Serial.println("Saturday");
      break; }
   String timeNowString = stringMonthRTC + "/" + stringDayRTC + "/" + String(yearRTC) + " @ " + stringHourRTC + ":" + stringMinuteRTC + ":" + stringSecondRTC;
   //Serial.print("timeNowString = "); Serial.println(timeNowString);
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


 //*************************** SD Card section of functions
void writeFile(fs::FS &fs, const char * path, const char * message) { // Write to the SD card (DON'T MODIFY THIS FUNCTION)
  Serial.printf("\nWriting to : %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file) { Serial.println("Failed to open file for writing");return; }
  if(file.print(message)) {Serial.println("File written to SC card successfully"); } 
  else {Serial.println("Write failed"); }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) { // Append data to the SD card (DON'T MODIFY THIS FUNCTION)
  File file = fs.open(path, FILE_APPEND);
  if(!file) {Serial.println("Failed to open file for appending"); return; }
  if(file.print(message)){delay(10);} // {Serial.print("Message appended successfully. It is: "); Serial.println(message);} 
  else {Serial.println(".... :-(  append failed");}
  file.close();
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("\nREADING THIS FILE: %s", path);
  File file = fs.open(path, FILE_READ);
  if(!file){Serial.println("Failed to open file for reading"); return;}
  while(file.available()){Serial.write(file.read()); }
  file.close();
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



//********************************* OLED
void setupNclearOLED() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    while(1) {Serial.print("...can't connect to OLED display"); delay(1000);} } // Don't proceed, loop forever}
  display.clearDisplay();
  display.setTextSize(1);  // supports 1 to 8  1 = 10 pixels high
  display.setTextColor(WHITE);
  display.display();     // implements display info
}



 //*********************** basic stuff
void wakeupSetup(){  // LoRa setup 1st time, and after every wakeup (reboot)
  Serial.begin(115200);                   
  while (!Serial);  // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& PUT OLED ERROR DISPLAY HERE
  Wire.begin();     // Start the I2C interface: used on DS3231(RTC), OLED, BH1750(lux)and HTU21 (or BME280)
  delay (1000); // delay 1 sec when doing debug

   // GPIO and switches
  pinMode(26, INPUT);                 // switch 1  GPIO 26 = displayOLED  
  displayOLED = digitalRead(26);          //Serial.print("displayOLED = "); Serial.println(displayOLED);
  pinMode(25, INPUT);                 // switch 2  GPIO 25 = write2SD 
  write2SD = digitalRead(25);          Serial.print("write2SD = "); Serial.println(write2SD);
  pinMode(33, INPUT_PULLUP);          // switch 3  GPIO 33 = wifiEnabled 
  wifiEnabled = digitalRead(33);          //Serial.print("wifiEnabled = "); Serial.println(wifiEnabled);
  pinMode(32, INPUT_PULLUP);          // switch 4  GPIO 32 = setTime 
  setTime = digitalRead(32);              //Serial.print("setTime = "); Serial.println(setTime);
  pinMode(35, INPUT);                 // switch 5  GPIO 35 = useNTP 
  useNTP = digitalRead(35);               //Serial.print("useNTP = "); Serial.println(useNTP);
  pinMode(34, INPUT_PULLUP);          // switch 6  GPIO 34 = newSDfile  --- for startup when no data.txt file exists on the SD card; must run, then switch off, then reset ESP32
  newSDfile = digitalRead(34);            Serial.print("newSDfile = "); Serial.println(newSDfile);
  pinMode(39, INPUT);                 // switch 7  GPIO 39 = printSDdata
  printSDdata = digitalRead(39);          Serial.print("printSDdata = "); Serial.println(printSDdata);
  pinMode(36, INPUT);                 // switch 8  GPIO 36 = printSDerror 
  printSDerror = digitalRead(36);         Serial.print("printSDerror = "); Serial.println(printSDerror);
  delay(100);                         // give time to configure the pin  

  myRTC.setClockMode(false);  // set to 24h; use setClockMode(true); for 12h

  if(wifiEnabled){
      Serial.print("Connecting to "); Serial.println(ssid); // Connect to Wi-Fi network with SSID and password
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {delay(500); Serial.print(".");  }
      Serial.print("\n WiFi connected. ...IP address: "); Serial.println(WiFi.localIP());}   // Print local IP address

  if(setTime){  // assumes that WiFi is enabled if using NTP
    if(useNTP){
      if(!wifiEnabled) {  // something wrong --- not near WiFi ?
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
  timeNow = readNprintRTCtime(); // get current time and print it; use for OLED also
  
    // ESP32 pin assignments - needed every re-boot
  pinMode(LoRa_nCS, OUTPUT); digitalWrite(LoRa_nCS, HIGH);  
  pinMode(LoRa_RxDone, INPUT); // set LoRa_RxDone as an input on ESP32 -- the DIO-0 pin from the LoRa modem (defaults to Rx Done output)
  pinMode(LoRa_nRst, OUTPUT); digitalWrite(LoRa_nRst, HIGH); delay(10);
  
  SPI.begin();  // start the SPI interface (LoRa Modem will always have this i/f active, even in sleep mode) 
  delay(100);  // need a little time for SPI to settle down
  
  // Initialize SD card; create 2 files if newSDfile = Hi
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS))      {Serial.println("Card Mount Failed"); return;}
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {Serial.println("No SD card attached"); return;}
  if (!SD.begin(SD_CS))     {Serial.println("ERROR - SD card initialization failed!"); return;}   
  else {Serial.println("SD card initialized OK");}
  if(write2SD){      
    if(newSDfile){
      dataToRecord = "\r\nData File: msmts started at " + timeNow;
      Serial.print("dataToRecord - following line:"); Serial.print(dataToRecord);
      Serial.print("\nNEW data.txt FILE -  being created on SD card"); writeFile(SD, "/data.txt", dataToRecord.c_str());}
    delay(100); // wait for write to complete before starting write to another file
    if(newSDfile){
      dataToRecord = "\r\nError File: msmts started at " + timeNow;
      Serial.print("dataToRecord - following line:"); Serial.print(dataToRecord);
      Serial.print("\nNEW error.txt FILE -  being created on SD card"); writeFile(SD, "/error.txt", dataToRecord.c_str());}
    delay(100);} // wait 
}


 //******************************** packet processing and display
void processPacket(){
  int irqFlags = readLoRaReg(REG_IRQ_FLAGS);
  //Serial.print("irqFlags = "); Serial.println(irqFlags);
  
    // check CRC
  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
      goodPayloadCRCpacket = true; 
      Serial.print("\nPAYLOAD CRC is GOOD ... packet received...");}  
  else {
      Serial.println("PAYLOAD CRC is BAD"); 
      goodPayloadCRCpacket = false;
      if(write2SD){      
          dataToRecord = "\r\nBAD PACKET ... good header CRC, good sync word, BAD PAYLOAD CRC - rcvd @," + timeNow;
          if(!newSDfile){appendFile(SD, "/error.txt", dataToRecord.c_str());Serial.println("\n... appended to error.txt file"); } } }
  writeLoRaReg(REG_IRQ_FLAGS, irqFlags); // clear all IRQ's (the packet reception set the RX done flag, connected to DI/O-0))

   // process the packet if good CRC packet
  if(goodPayloadCRCpacket){
    int packetIndex = 0;
    int packetLength = readLoRaReg(REG_RX_NB_BYTES);  // read packet length
    //if (readLoRaReg(REG_FIFO_RX_CURRENT_ADDR != 0)) {Serial.println("Rx Fifo addr not equal 0");} // recordErrorInSDcard("Rx Fifo addr not equal 0");
    writeLoRaReg(REG_FIFO_ADDR_PTR, readLoRaReg(REG_FIFO_RX_CURRENT_ADDR)); // set FIFO address to current RX address
  
    writeLoRaReg(REG_MODEM_CONFIG_1, readLoRaReg(REG_MODEM_CONFIG_1) & 0xfe); //explicitHeaderMode();  // necessary ???
                                             
     // now read the packet and do filtering  ..  should see: #Tx1, Air T(F):,70, Air Humid(%):,43, Light(lux):,120, pkt#,444,#Tx1
    LoRaData ="";
    while(readLoRaReg(REG_RX_NB_BYTES) - packetIndex){   // this is while(LoRa.available())
      LoRaData = LoRaData + readLoRaReg(REG_FIFO); // was String LoRaData = LoRa.readString();
      packetIndex++;}
    
     // filter this: eg: #Tx1, Air T(F):,tempF, Air Humid(%):,humid, Light(lux):,luxLevel, pkt#,pktCtr,#Tx1  
      // "airHumid(%) = "; String OLEDline4b = LoRaData.substring(LoRaData.indexOf("Air H")+14, LoRaData.indexOf("Light")-2);
    if(((String(LoRaData[3]) == "1") && (String(LoRaData[packetLength - 1]) == "1")) || ((String(LoRaData[3]) == "2") && (String(LoRaData[packetLength - 1]) == "2")) || ((String(LoRaData[3]) == "3") && (String(LoRaData[packetLength - 1]) == "3"))) {
      if (strstr(LoRaData.c_str(),"Humid") != NULL) { // further filtering
        Serial.print("\n...packet after  filtering: "); Serial.print(LoRaData);  // for debug
        rssi = readLoRaReg(REG_PKT_RSSI_VALUE) - 157;   // = packet stength in dbm (see section 5.5.5 in spec for more info) 
        snr = (int8_t)readLoRaReg(REG_PKT_SNR_VALUE) * 0.25;
        Serial.print("\n rssi = "); Serial.print(rssi); Serial.print("  ... snr = "); Serial.print(snr);
        String stringPresentPacketNumber = LoRaData.substring(LoRaData.indexOf("pkt")+5, LoRaData.length()-5); 
        char *ptr; // a mandatory pointer to what follows after the number (there isn't anything in this case)
        long presentPacketNumber = strtol(stringPresentPacketNumber.c_str(), &ptr, 10);  // string to long number
        if(packetNum != 0){  // skip 1st time powered up where packetNum = 0
          if((presentPacketNumber - packetNum) != 1){  // compare to what was stored in RAM in last wakeup
            Serial.print("missed a packet: #"); Serial.print(presentPacketNumber); 
            packetsMissed++;}}
        Serial.print("\n...number of packets missed = "); Serial.println(packetsMissed);
        packetNum = presentPacketNumber;
        
        // timeNow = readNprintRTCtime(); // get current time and print it; use for OLED also
        Serial.print("timeNow is "); Serial.println(timeNow);

        if(write2SD){      
            dataToRecord = "\r\nGOOD PACKET - rcvd @," + timeNow + ", RSSI:," + rssi + ", SNR:, " + snr + ", " + LoRaData;
            Serial.print("dataToRecord - following line:"); Serial.print(dataToRecord);
            if(!newSDfile){appendFile(SD, "/data.txt", dataToRecord.c_str());Serial.println("\n... appended to data.txt file");}}}} //The c_str() method converts a string to an array of characters with a 
 
      else{  // good PAYLOAD CRC but fails filtering
        goodPayloadCRCpacket = false;
        if(write2SD){      
          dataToRecord = "\r\nBAD PACKET ... good header CRC, good sync word, good payload CRC, BAD CONTENT (failed filter) - rcvd @," + timeNow;
          Serial.print("dataToRecord - following line:"); Serial.print(dataToRecord); Serial.print("\n ... here is what was received: "); Serial.print(LoRaData);
          if(!newSDfile){appendFile(SD, "/error.txt", dataToRecord.c_str());Serial.println("\n... appended to error.txt file");} } } //The c_str() method converts a string to an array of characters with a
  }  // end if(goodPayloadCRCpacket())
  if(printSDdata) {readFile(SD, "/data.txt");}
  if(printSDerror) {readFile(SD, "/error.txt");}
} // end of processPacket()
      
void displayPacketOLED(){
  setupNclearOLED();
   // format of packet sent from Tx - #Tx1 for example:
    //****** full message  
    //String tempString = "#" + String(unitNumber) + ",Soil T(C):," + String(soilTemp) + ", Soil Moisture(V):," + String(soilMoistureVolts) + ", Air T(C):," + String(tempC) + ", Air Humidity(%):," + String(humid)+ ", Light(lux):," + String(luxLevel);
    //String toSend = tempString + ", Atm Pressure(atm):," + String(pres) + ", altitude(mts):," + String(alt) + ", pkt#," + String(pktCounter) + ", send interval(minutes) ,3,#" + String(unitNumber);
    // **** message without Soil T or Moisture:
    //String toSend="#"+String(unitNumber)+", Air T(F):,"+String(tempF)+", Air Humid(%):,"+String(airHumid)+", Light(lux):,"+String(luxLevel)+", pkt#,"+String(packetNum)+",#"+String(unitNumber);
    // eg: #Tx1, Air T(F):,tempF, Air Humid(%):,humid, Light(lux):,luxLevel, pkt#,pktCtr,#Tx1 (note - begins and ends with the unit # - this is used for filtering)
    
  String OLEDline1a = LoRaData.substring(1,4); String OLEDline1b = "pkt#"; String OLEDline1c = String(packetNum); 
  String OLEDline2 = timeNow;
  String OLEDline3a = "airTemp(F) = "; String OLEDline3b = LoRaData.substring(LoRaData.indexOf("Air T")+ 10, LoRaData.indexOf("Air H")-2);
  String OLEDline4a = "airHumid(%) = "; String OLEDline4b = LoRaData.substring(LoRaData.indexOf("Air H")+14, LoRaData.indexOf("Light")-2);
  String OLEDline5a = "Light(lux) = "; String OLEDline5b = LoRaData.substring(LoRaData.indexOf("Light")+12, LoRaData.indexOf("pkt")-2);
  String OLEDline6a = "missed:"; String OLEDline6b = String(packetsMissed); String OLEDline6c = "SNR:"; String OLEDline6d = String(snr);
  
  //(column,row)-in pixels, where 1 row = 21 characters across (128 pix/columns across = 6 pix/char width); and 6 lines = 64 pix/rows down = 10.7 pix/line)
  display.setCursor(0,00); display.print(OLEDline1a); display.setCursor(48,00); display.print(OLEDline1b); display.setCursor(78,00); display.print(OLEDline1c);
  display.setCursor(0,11); display.print(OLEDline2);
  display.setCursor(0,22); display.print(OLEDline3a); display.setCursor(78,22); display.print(OLEDline3b);
  display.setCursor(0,33); display.print(OLEDline4a); display.setCursor(84,33); display.print(OLEDline4b);
  display.setCursor(0,44); display.print(OLEDline5a); display.setCursor(78,44); display.print(OLEDline5b);
  //display.setCursor(0,55); display.print(OLEDline6a); display.setCursor(96,55); display.print(OLEDline6b);         
  display.setCursor(0,55); display.print(OLEDline6a); display.setCursor(48,55); display.print(OLEDline6b); display.setCursor(72,55); display.print(OLEDline6c);display.setCursor(96,55); display.print(OLEDline6d);        
  
  display.display();     // implements display info
  
  digitalWrite(LEDbuiltin, true); delay(500); digitalWrite(LEDbuiltin, false);  // blink LED when packet comes in if displaying (shuts off when goes to sleep)
}

void reinitializeLoRaModem(){
  digitalWrite(LoRa_nRst, LOW); delay(10); digitalWrite(LoRa_nRst, HIGH); delay(10);  // resets regs, and puts in standby mode
  delay(500);  /// delay for the RC ckt
     // LoRa Modem setup --- must be in sleep (not standby) to configure modem; note: in sleep mode, the Fifo data is cleared, so read it before this
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);  // must be put in sleep mode (not standby !! doesn't work!) to be able to write to config frequency
                // note: Sleep mode = only SPI i/f active;  Standby mode = Sleep + RC osc on + xtal on;  RX mode = everything on
  
    // configure the LoRa Modem frequency -- must be done in standby (or sleep mode) (from begin() in LoRa.cpp)
  long frequencyLoRa = 915E6; // 915 Mhz for North America  // this is setFrequency(frequency) in LoRa.cpp
  uint64_t frf = ((uint64_t)frequencyLoRa << 19) / 32000000;
  writeLoRaReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeLoRaReg(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeLoRaReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
  writeLoRaReg(REG_FIFO_RX_BASE_ADDR, 0);  // on POR, RX base addr = 0x00 (Total Fifo is 256 bytes)
  writeLoRaReg(REG_FIFO_TX_BASE_ADDR, 0);  // on POR, TX base addr = 0x80 = 128 bytes = top 1/2 of Fifo  (spec is not clear on setting both to 0 at the same time)
  writeLoRaReg(REG_LNA, readLoRaReg(REG_LNA) | 0x03); // set LNA boost
  writeLoRaReg(REG_MODEM_CONFIG_3, 0x04);             // set auto AGC
  writeLoRaReg(REG_SYNC_WORD, 0xFA);                  //sync word: ranges from 0-0xFF   12/11-12 was 0xFA ... then 0x19
  writeLoRaReg(REG_MODEM_CONFIG_1, readLoRaReg(REG_MODEM_CONFIG_1) & 0xfe); // set to explicitHeaderMode(); -- necessary?
  
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);    // put in standby
}


//**************************************************************************************
/**************************************************************************************
* ************************************** setup ***** **********************************
* *************************************************************************************/
//**************************************************************************************
void setup() {
  wakeupSetup();
  processPacket();
  if(goodPayloadCRCpacket) {displayPacketOLED();}
  goodPayloadCRCpacket = false;  // reset this flag
  reinitializeLoRaModem();

  if(!displayOLED){setupNclearOLED();}  // clear display during sleep if selected
  
    // now get ready for ESP32 to sleep (modem doesn't sleep - awake scanning for packets, then sets Rx Done IRQ bit, which sets DI/O-0 high (cleared once wakes up)
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS); // sets Rx in continuous scan mode (modem is not powered down !)
  esp_sleep_enable_ext0_wakeup(LoRa_RxDone,1); // Enable Ext0 as wakeup/reboot: Logic Level defined = 1 = Hi = interrupt; must do this after a reboot
  Serial.println("\nGoing to sleep now ... Waiting for another packet to wake up the ESP32\n\n");
  Serial.flush(); 
  esp_deep_sleep_start();  // go to sleep
  Serial.println("This will never be printed");
}


/**************************************************************************************
* ************************************** main loop() **********************************
* ************************************************************************************/
void loop() {
}


/*******************************************************************************/
/********************************************************************************/
/**************************  useful stuff **************************************/
/*******************************************************************************/

/*  SPI modes:
    Mode 0 (the default) − Clock is normally low (CPOL = 0), and the data is sampled on the transition from low to high (leading edge) (CPHA = 0).
    Mode 1 − Clock is normally low (CPOL = 0), and the data is sampled on the transition from high to low (trailing edge) (CPHA = 1).
    Mode 2 − Clock is normally high (CPOL = 1), and the data is sampled on the transition from high to low (leading edge) (CPHA = 0).
    Mode 3 − Clock is normally high (CPOL = 1), and the data is sampled on the transition from low to high (trailing edge) (CPHA = 1).
*/
  
  
/******* from LoRa.cpp
int LoRaClass::begin(long frequency){
  pinMode(_ss, OUTPUT); digitalWrite(_ss, HIGH);
  pinMode(_reset, OUTPUT);  digitalWrite(_reset, LOW); delay(10); digitalWrite(_reset, HIGH); delay(10);

  _spi->begin();

  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {  return 0; }
  sleep();

  setFrequency(frequency);
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0); // set base addresses
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);   // set LNA boost
  writeRegister(REG_MODEM_CONFIG_3, 0x04); // set auto AGC
  setTxPower(17); // set output power to 17 dBm
  idle(); // put in standby mode
  return 1;
}

int LoRaClass::read()      {
  if (!available()) { return -1; }
  _packetIndex++;
  return readRegister(REG_FIFO);
}

int LoRaClass::available(){
return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}
   
uint8_t LoRaClass::readRegister(uint8_t address){
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value){
  singleTransfer(address | 0x80, value);}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value){
  uint8_t response;
  digitalWrite(_ss, LOW);
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(address);
  response = _spi->transfer(value);
  _spi->endTransaction();
  digitalWrite(_ss, HIGH);
  return response; 
}
  
void LoRaClass::handleDio0Rise() {  // this checks the IRQs, then reads the packet if no CRC error and RX_done, or 
  int irqFlags = readRegister(REG_IRQ_FLAGS);  // read IRQs then clear them
  writeRegister(REG_IRQ_FLAGS, irqFlags);
  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {  // no CRC error
    if ((irqFlags & IRQ_RX_DONE_MASK) != 0) { _packetIndex = 0;  // RX_done: received a packet
      int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);
      writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));  // set FIFO address to current RX address
      if (_onReceive) {_onReceive(packetLength); } }   // _onReceive is myReadPacket() ... so it reads the pakcket
    else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {if (_onTxDone) {_onTxDone();}}
  }

//LoRa.onReceive(readPacket);  // this is the callback - when LoRa.onReceive is executed, the onReceive() below is executed
void LoRaClass::onReceive(void(*callback)(int)){
        _onReceive = callback;
        if (callback) { // enable the interrupt
          pinMode(_dio0, INPUT);
          attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);} 
        else {   // no callback; no interrupt enable
          detachInterrupt(digitalPinToInterrupt(_dio0));}
 }

//LoRa.receive();  // put LoRa into RX_continuous receive mode     
void LoRaClass::receive(int size){
  writeRegister(REG_DIO_MAPPING_1, 0x00);     // DIO0 => RXDONE
  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);} 
  else {
    explicitHeaderMode(); }
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
  
ISR_PREFIX void LoRaClass::onDio0Rise() {LoRa.handleDio0Rise(); }

*/

/* **************** SPI stuff in LoRa library ****************************
void SPIClass::beginTransaction(SPISettings settings){
    //check if last freq changed
    uint32_t cdiv = spiGetClockDiv(_spi);
    if(_freq != settings._clock || _div != cdiv) {
        _freq = settings._clock;
        _div = spiFrequencyToClockDiv(_freq); }
    spiTransaction(_spi, _div, settings._dataMode, settings._bitOrder);
    _inTransaction = true;
}

void SPIClass::endTransaction(){
    if(_inTransaction){
        _inTransaction = false;
        spiEndTransaction(_spi);}
}


  _spi->beginTransaction(_spiSettings);
  _spi->transfer(readAddress & 0x7f);
  response = _spi->transfer(0x00);  // read - no value to send
  _spi->endTransaction();

*/


/********************** delete when confirms changes have worked
void reinitializeLoRaModem(){
  digitalWrite(LoRa_nRst, LOW); delay(10); digitalWrite(LoRa_nRst, HIGH); delay(10);  // resets regs, and puts in standby mode
  delay(500);  /// delay for the RC ckt
     // LoRa Modem setup --- must be in sleep (not standby) to configure modem; note: in sleep mode, the Fifo data is cleared, so read it before this
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);  // must be put in sleep mode (not standby !! doesn't work!) to be able to write to config frequency
                // note: Sleep mode = only SPI i/f active;  Standby mode = Sleep + RC osc on + xtal on;  RX mode = everything on
    // configure the LoRa Modem frequency -- must be done in standby (or sleep mode) (from begin() in LoRa.cpp)
  long frequencyLoRa = 915E6; // 915 Mhz for North America  // this is setFrequency(frequency) in LoRa.cpp
  uint64_t frf = ((uint64_t)frequencyLoRa << 19) / 32000000;
  writeLoRaReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeLoRaReg(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeLoRaReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
  
  writeLoRaReg(REG_FIFO_RX_BASE_ADDR, 0);  // on POR, RX base addr = 0x00 (Total Fifo is 256 bytes)
  writeLoRaReg(REG_FIFO_TX_BASE_ADDR, 0);  // on POR, TX base addr = 0x80 = 128 bytes = top 1/2 of Fifo  (spec is not clear on setting both to 0 at the same time)

  writeLoRaReg(REG_LNA, readLoRaReg(REG_LNA) | 0x03); // set LNA boost
  writeLoRaReg(REG_MODEM_CONFIG_3, 0x04);             // set auto AGC
  writeLoRaReg(REG_SYNC_WORD, 0xFA);                  //sync word: ranges from 0-0xFF
  writeLoRaReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);             // put in standby
  writeLoRaReg(REG_MODEM_CONFIG_1, readLoRaReg(REG_MODEM_CONFIG_1) & 0xfe); // set to explicitHeaderMode(); -- necessary?
  writeLoRaReg(REG_FIFO_RX_BASE_ADDR, 0);                                   // on POR, RX base addr = 0x00 (Total Fifo is 256 bytes)
}
*/


/***************** won't work since the CRC payload bit in the header is ignored when Tx sets it, which is does .... would need to use implicit headers which have other issues
    // check CRC
  if((REG_HOP_CHAN & CRC_ON_PAYLOAD) == CRC_ON_PAYLOAD){           // read only: bit 6 on Rx = 1 if Header contains directive to use payload CRC ; check before checking payload CRC
    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
      goodPayloadCRCpacket = true; 
      Serial.print("\n...PAYLOAD CRC set to be checked, and is GOOD ... packet received...");  
      if(write2SD){      
          dataToRecord = "\r\n...PAYLOAD CRC set to be checked, and is GOOD - pkt rcvd @," + timeNow;
          if(!newSDfile){appendFile(SD, "/error.txt", dataToRecord.c_str());Serial.println("\n... appended to error.txt file");} } }
    else {
      Serial.println("PAYLOAD CRC is set to be provided, but is BAD"); 
      goodPayloadCRCpacket = false;
      if(write2SD){      
          dataToRecord = "\r\nPAYLOAD CRC is set to be provided, but is BAD - pkt rcvd @," + timeNow;
          if(!newSDfile){appendFile(SD, "/error.txt", dataToRecord.c_str());Serial.println("\n... appended to error.txt file"); }}}}
  else {
    Serial.println("PAYLOAD CRC not set to be provided (packet header would have had this bit set)"); 
    goodPayloadCRCpacket = false;
    if(write2SD){      
       dataToRecord = "\r\nPAYLOAD CRC not set to be provided (packet header would have had this bit set) - pkt rcvd @," + timeNow;
       if(!newSDfile){appendFile(SD, "/error.txt", dataToRecord.c_str());Serial.println("\n... appended to error.txt file"); }}}
  writeLoRaReg(REG_IRQ_FLAGS, irqFlags); // clear all IRQ's (the packet reception set the RX done flag, connected to DI/O-0))
*/
