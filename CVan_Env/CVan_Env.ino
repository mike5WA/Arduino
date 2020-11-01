/**************************************************************************/
/*Mike Garner 25th July 2020

  Code for arduino nano to display weather and gps data on 20*4 LCD located in campervan
  Sparkfun weather sensor data + external temp sensor polled every 5 seconds
  Pressure trend calculated every 20 minutes
  GPS polled every second to enable clock to display seconds
  GPS data converted to display day of week, month, year etc via guassian algorithm
  Locale Perth WA so UTC time adjusted by 8hrs 28,800 seconds
  Unit installed in campervan so routine to adjust for timezones for travel
  (3 hr in 30 min increments) commencing West to East & back

  SparkFun Si7021 Breakout  
  Hardware Connections:
      HTU21D ------------- Photon
      (-) ------------------- GND
      (+) ------------------- 3.3V (VCC)
       CL ------------------- D1/SCL
       DA ------------------- D0/SDA

  Hardware Platform: SparkFun RedBoard     Arduino IDE 1.6.5
  2nd temp sensor is 1 wire Dallas DS18B20 connected to pin 2
  Pressure sensor is MPL3115A2 can also supply temp and altitude 
  
  GPS is Tiny GPS; Uno RX = 5 TX = 4 ; Used baud of 9600 not 4800 for software serial
  
    Liquid Crystal 20*4 display 
    Pin configuration
      lcd 1   Grd Display
      lcd 2   +5v Display
      lcd 3   Display contrast via pot     
      lcd 4   RS      to Uno 7
      lcd 5   RW      Grd as we are only writing to display   
      lcd 6   Enable  to Uno 8
      lcd 7   n/c
      lcd 8   n/c 
      lcd 9   n/c 
      lcd 10  n/c
      lcd 11  DB4     to Uno 9
      lcd 12  DB5     to Uno 10
      lcd 13  DB6     to Uno 11
      lcd 14  DB7     to Uno 12
      lcd 15  +5v Backlight for PWM control to Uno 3
      lcd 16  Grd Backlight

UNO Digital Pins used
  D0    RX  Used when gps set to UART NB set switch to SW to upload
  D1    TX  Used when gps set to UART NB set switch to SW to upload   
  D2    to DB18B20 data pin
  D3    to lcd 15 PWM for backlight  
  D4    TX pin for GPS
  D5    RX pin for GPS 
  D6    used somewhere on shield. Had issues with gps when tried to use as digital pin
  D7    to lcd 4 RS 
  D8    to lcd 6 Enable 
  D9    to lcd 11 DB4
  D10   to lcd 12 DB5
  D11   to lcd 13 DB6
  D12   to lcd 14 DB7
  D13   used by uno/shield led

UNO Analog Pins used
  A0  to timezone button. Pulled high (1) so when button pressed pulled low (0)
  A1  to timezone led to indicate timezone being changed 
  A2  Ground likely used to ground shield
  A3  Power likely used to power shield
  A4  n/c
  A5  n/c  

HISTORY
  9/8/20 Start just displaying pressure, working fine. Also temperature approx 5d high
  10/8/20 Next add DS18B20 to mix. OK working,
  10/8/20 Add humidity SI7021
  15/8/20 Stable gps & sensors except date incorrect up to 8am.
  19/8/20 Incorporate pressure trend calculation 20 minute interval
  29/8/20 Revised timezone adjustment routine
   
/**************************************************************************/
//Libraries
#include <Wire.h>
#include "SparkFunMPL3115A2.h"                  //Pressure sensor
#include "SparkFun_Si7021_Breakout_Library.h"   //Humidity & Temperature sensor?
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
#include <OneWire.h>                //Required by DS18B20
#include <DallasTemperature.h>      //Required by DS18B20

#include "Adafruit_LiquidCrystal.h" //Liquid Crystal
#include <TinyGPS++.h>              //TinyGPS
#include <SoftwareSerial.h>         //SoftwareSerial
#include <TimeLib.h>                //Arduino time libra

MPL3115A2 myPressure; //Create an instance of the pressure sensor
Weather myHumidity;   //Create an instance of the humidity sensor

const byte REFERENCE_3V3 = A3;
const byte LIGHT = A4;
//const byte BATT = A2;

//Global Variables
unsigned long ms;
long lastSecond;
long Second_W;        //The millis counter to see when a second rolls by
long Second_T;
int externaltempc = 0;
int humidity = 0;
int hpa_Char = 7;   //Used to determine lcd character for pressure change
float hpa_1 = 1013; //Normal sea level pressure to start
String hpa_Str;
String temp_h_Str;

// DS18B20 data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature DS18B20(&oneWire);

//Create custom char House & Tree for In/Out temp display
byte house[8]={B00000,B00100,B01010,B10001,B01110,B01110,B01110,B00000};        //lcd char 1
byte tree[8]={B00100,B01110,B10101,B01110,B10101,B00100,B00100,B10101};         //lcd char 2
//Create custom char for latitude, longtitude & satelites & arrows
byte latitude[8] = {B11111,B00000,B11111,B00000,B11111,B00000,B11111,B00000};   //lcd char 3
byte longtitude[8]={B10101,B10101,B10101,B10101,B10101,B10101,B10101,B10101};   //lcd char 4
byte satelites[8]={B10001,B01010,B00100,B11111,B00100,B01010,B10001,B00000};    //lcd char 5
byte upArrow[8]={B00000,B00100,B01110,B10101,B00100,B00100,B00100,B00000};      //lcd char 6
byte rightArrow[8]={B00000,B00100,B000010,B11111,B00010,B00100,B00000,B00000};  //lcd char 7
byte downArrow[8]={B00100,B00100,B00100,B00100,B00100,B10101,B01110,B00100};    //lcd char 8

// Associate lcd pins with arduino 
const int RS = 7, EN = 8, DB4 = 9, DB5 = 10, DB6 = 11, DB7 = 12;
Adafruit_LiquidCrystal lcd(RS, EN, DB4, DB5, DB6, DB7);
int backlightPin = 3;
int brightness = 128;
const int numRows = 4;
const int numCols = 20;

//Create an array for each row of lcd 0 - 3 chars 0-20
char lcdRow0[20];
char lcdRow1[20];
char lcdRow2[20];
char lcdRow3[20];

//Tiny GPS
static const int RXPin = 5, TXPin = 4;
static const uint32_t GPSBaud = 9600;   //Baud rate for GPS 4800 did not work
TinyGPSPlus gps;                        // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin);        // The serial connection to the GPS device

// Define variables for GPS data
unsigned long last = 0UL;
unsigned long age;              //Used to get age of gps data
int gpsYear,gpsMonth,gpsDay;    //Calculated from gps data
int hr,min,sec;                 //hours, minutes, seconds
const int offset = 8;           //Perth offset hours from UTC
String WeekDay = "";        
String TheMonth = "";
long TZ = 28800;              //WST = UTC+8hrs in seconds
//int TZButtonState = 0;        //Timezone button monitor A0
int TZDirection = 1;          //Assumes starting in WST
int period = 8500;            //Period ms between weather data readings
unsigned long startMillis = 0;

//Arrays for Days of Week & Months of Year
const char* DOW[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char* MOY[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

float Lat, Long;
int Sats, altitude;
static char dToString[8];   //Create array to take float values for dtostrf function
String LatStr,LongStr;      //Strings for Lat & Long

//Co-Ordinates for distance & bearing calcs amend as required
//eg 5 Urbahns -31.813900:115.745400  Sydney -33.865143:151.209900
float my_Lat = -31.813900;   
float my_Lon = 115.745400;
int distanceToDest, courseToDest ;
const char* bearingToDest;

//On time & Off time for display 
int OnTime = 6;
int OffTime = 21;

//********************SETUP**********************************

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  Serial.println("Weather Shield Setup!");

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  myPressure.begin();               //Get sensor online
  myPressure.setModeBarometer();    // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7);  // Set Oversample to the recommended 128
  myPressure.enableEventFlags();    // Enable all three pressure and temp event flag 
 
  myHumidity.begin();               //Configure the humidity sensor

  lastSecond = millis();

  Serial.println("Weather Shield online!");

  //Set up LCD display & create special characters
    pinMode(backlightPin, OUTPUT);
    analogWrite(backlightPin, 255);   //Turn backlight to max  
    lcd.begin(numCols, numRows);      
    lcd.createChar(1, house);
    lcd.createChar(2, tree);
    lcd.createChar(3, latitude);
    lcd.createChar(4, longtitude);
    lcd.createChar(5, satelites);
    lcd.createChar(6, upArrow);
    lcd.createChar(7, rightArrow);
    lcd.createChar(8, downArrow);
    lcd.home();
    lcd.print(" G'Day! Setting Up");
}

//*************End of Setup************************************

// SmartDelay custom version of delay() ensures that the gps object
// is being "fed" while delay is on as standard delay() pauses computing.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
}

//----------------------------------------------------------------

void DigitalToString (float f_val, int Mylength, int Mydecimals)
{
  //sprintf does not display strings (%f) on arduino
  //So this function converts floats to strings to then use sprintf %s
  //NB %s expects char* not str so need to use string.c_str() in sprintf function.
  dtostrf(f_val,Mylength,Mydecimals,dToString);
}

//-----------------------------------------------------------

void printToLCD()
{
//Print data to 20*4 LCD via lcdRow arrays 0-3
//%f floats not supported so convert float to string if decimal places required
//s% takes a char* not a std::string. Thus use string.c_str() which gives const char* to the contents of a std::string
//%.2d formats to display 2 numbers with 0 padding if num<2
sprintf(lcdRow0, " %s-%s-%.2d %.2d:%.2d:%.2d", WeekDay.c_str(), TheMonth.c_str(), gpsDay, hr, min, sec);
sprintf(lcdRow1, "%c%.2s %c%.2d H%d%% P%c%.4s", 1, temp_h_Str.c_str(), 2, externaltempc, humidity, hpa_Char, hpa_Str.c_str());    //1,2 &  8 are defined lcd chars
sprintf(lcdRow2, "%c%d %c%7s %c%7s", 5, Sats, 3, LatStr.c_str(), 4, LongStr.c_str()); 
sprintf(lcdRow3, "m%c%-4d H%.4dkm %c%-3s", 6,altitude, distanceToDest, 7, bearingToDest);

lcd.setCursor(0,0);
lcd.print(lcdRow0);
lcd.setCursor(0,1);
lcd.print(lcdRow1);
lcd.setCursor(0,2);
lcd.print(lcdRow2); 
lcd.setCursor(0,3);
lcd.print(lcdRow3);

} 
//--------------------------------------------------------------------

//Get weather data from sensors
void WeatherData()
{
  humidity = myHumidity.getRH();  //Get relative humidity

    if (humidity == 998)          //Humidty sensor failed to respond 
    {
      Serial.println("I2C communication to SI7021 failed.");
      //Try re-initializing the I2C comm and the sensors
      myPressure.begin(); 
      myPressure.setModeBarometer();
      myPressure.setOversampleRate(7);
      myPressure.enableEventFlags();
      myHumidity.begin();
    }

    else  //Proceed with temperature reading from SI7021 sensor
    {
      float temp_h = myHumidity.getTempF();
      temp_h = ((temp_h-32)/1.8)-5;   //Convert to centigrade deduct 5 degrees as sensor adj to processor
      DigitalToString(temp_h,2,0);    //Convert to string for LCD display
      temp_h_Str = dToString;         //String passed to LCD array
  /*   
      Serial.print("Humidity = ");
      Serial.print(humidity);
      Serial.print("%,");
      Serial.print(" temp_h = ");
      Serial.print(temp_h, 2);
      Serial.print("C,");
  */  
      //Check Pressure Sensor MPL3115A2
      float pressure = (myPressure.readPressure()/100);
      //Round pressure reading to 0 decimal places
      DigitalToString(pressure,4,0);  //Convert pressure to string
      hpa_Str = dToString;            //String passed to LCD array
    
     //Check if pressure over last x mins rising or falling
          
      if (millis()>lastSecond + 1200000) //Time elapsed 1,200,000 =  20min
      {
        if ((pressure - hpa_1) >=0.1)  //Pressure change of +0.1hpa
          {
            hpa_Char = 6;             //Set lcd character to up arrow
            hpa_1 = pressure;         //Reset base pressure reading
          }
        else if ((pressure - hpa_1) <=-0.1) //Pressure change of -0.1hpa
          {
            hpa_Char = 8;             //Set lcd character to down arrow
            hpa_1 = pressure;         //Reset base pressure reading
          }
        else 
          {
           hpa_Char = 7;             //Set lcd character to straight arrow
          //hpa_1 not reset as trend insufficient 
          }
      /*        
        Serial.print(" Pressure = ");
        Serial.print(pressure);
        Serial.print("hpa,");
        Serial.print(" hpa_Char = ");
        Serial.print(hpa_Char);
        Serial.print(" hpa_1 = ");
        Serial.print(hpa_1);
        Serial.print(" Diff ");
        Serial.println(pressure - hpa_1);
      */
      
        lastSecond = millis();
      }
          
     //Get temp from DS18B20 outside sensor
      DS18B20.requestTemperatures();
      externaltempc = DS18B20.getTempCByIndex(0); //Only one DS18B20 so index = 0
    }
} 

//-----------------------------------------------------------------

void gpsData ()
{
 //Get time data
    if (gps.time.isUpdated())
    {
      hr = (gps.time.hour());
      min = (gps.time.minute());
      sec = (gps.time.second());
    }

//Get date data 
    if (gps.date.isUpdated())
    { 
      gpsYear = (gps.date.year());
      gpsMonth = (gps.date.month());
      gpsDay = (gps.date.day());
    }

//Get location data
    if (gps.location.isUpdated())
    {
      Lat = (gps.location.lat());       
      Long = (gps.location.lng()); 
    }

//Get satellite data  
    if (gps.satellites.isUpdated())
    {
      Sats = (gps.satellites.value());
    } 

//Get altitude data
    if (gps.altitude.isUpdated())
    {
      altitude = (gps.altitude.meters());
    } 
  
//Error routine checks for minimum data over 5 seconds
    if (millis() - last > 5000)
    {
      if (gps.charsProcessed() <10)
      {
        Serial.println(F("WARNING: No GPS data. "));
      }

     last = millis(); 
    } 

//Manage data
//Set current UTC time
    setTime(hr, min, sec, gpsDay, gpsMonth, gpsYear); 
    //Add offset seconds TZ to get local time (WST = 8 * 3,600 secs = 28800)
    adjustTime(TZ);
    //Reset variables to local time via time library functions
    hr = hour();
    min = minute();
    sec = second();
    gpsYear = year();
    gpsMonth = month();
    gpsDay = day();

    //Routine to implement guassian algorithm for day of week
    int c,y,m,d;                      //century, year, month, day
    int cc,yy;            
    int dayofweek, monthofyear;    
      
    cc = gpsYear/100;                
    yy = gpsYear - ((gpsYear/100)*100); 
    c = (cc/4) - 2*cc-1; 
    y = 5*yy/4;
    m = 26*(gpsMonth+1)/10;
    d = gpsDay;
    dayofweek = (c+y+m+d)%7;        //Gives a value from 0 - 6
     
    WeekDay = DOW[dayofweek];       //Fetch day string from DOW array
    monthofyear = gpsMonth - 1;     //Adjust month as array starts at 0
    TheMonth = MOY[monthofyear];    //Fetch month string from array MOY
  /*
    Serial.print("D/M/Y hh:mm:ss ");
    Serial.print(WeekDay);
    Serial.print("/ ");
    Serial.print(TheMonth);
    Serial.print("/ ");
    Serial.print(gpsYear);
    Serial.print("  ");
    Serial.print(hr);
    Serial.print(":");
    Serial.print(min);
    Serial.print(":");
    Serial.println(sec);
  */

  //Convert Lat & Long to string via digitalToString()
    DigitalToString(Lat,7,3);
    LatStr = dToString;
    DigitalToString(Long,7,3);
    LongStr = dToString;

  //Altitude 
  if (gps.altitude.isUpdated())  altitude = (gps.altitude.meters());

    //Distance & bearing routine
    distanceToDest = 
    TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    my_Lat,
    my_Lon);
    distanceToDest = (distanceToDest/10); //Converted to Km

    courseToDest =
    TinyGPSPlus::courseTo(
    gps.location.lat(),
    gps.location.lng(),
    my_Lat, 
    my_Lon);
    bearingToDest = (TinyGPSPlus::cardinal(courseToDest));

}

//---------------------------------------------------------------
void TZAdjust()
{
/*
Function to loop through 6 * 30 min adjustments on button press
Starts at 28800 and adds 1800 increments to 39600 (WST + 3hrs)
Then decrease from 39600 back to 28800 (WST)
When led lights routine to increase decrease time has been actioned

Monitor A0 which will give 1 if high or 0 if low viz button pressed
LED connected to A1 to indicate routine activated
*/

  pinMode(A0, INPUT_PULLUP);
  int TZpin = digitalRead(A0);

//Button pressed takes TZpin low ie 0 
  if (TZpin == 0) 
  {
    analogWrite(A1, 255);  //Turn LED on

    //Select case dependant upon east or west travel
    switch (TZDirection)
    {
      case 0: //Decrement time 30 min westwards
        {
          TZ = (TZ - 1800);   
          Serial.print("TZ Case 0 = ");
          Serial.println(TZ);
          break;
        }

      case 1: //Increment time 30 min eastwards
        {
          TZ = (TZ + 1800);   
          Serial.print("TZ  Case 1 = ");
          Serial.println(TZ);
          break;
        }
      }

    //Check and adjust direction if required
    if ((TZ <= 28800)) (TZDirection = 1); //Move east +1800 second increments
    if ((TZ >= 39600)) (TZDirection = 0); //Move west -1800 second increments
    smartDelay(1500);                     //Keep led on for short time
    analogWrite(A1, 0);                   //Turn LED off  
  }
}

//-----------------------------------------------------------------------

//***********************VOID LOOP*********************************  

void loop()
{
  //Get weather sensor readings every 5 seconds
  if (millis() - Second_W >= 5000)
  {
    Second_W += 4000;       //Compound increment to Second_W
    WeatherData();          //Get weather data
  }

  //Get gps data
    gps.encode(ss.read());
    gpsData();

  //Timezone adjustment (TZ) routine
  TZAdjust();
    
  //Turn on display between OnTime & OffTime
  if ((hr >= OnTime) && (hr < OffTime))   
  {
    lcd.display();
    analogWrite(backlightPin, 255); //Turn backlight to max    
    printToLCD();
  }
  else
  {
    lcd.noDisplay();               //Turn off display
    analogWrite(backlightPin, 3);  //Dim backlight
  }
    smartDelay(1000);
}

