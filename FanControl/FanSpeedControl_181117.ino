
/*
 * Date 11th November 2018 (Mike Garner)
 * ======================================================================================
 * DHT22 sensor located in fridge space monitors fridge exhaust temp
   Run fridge exhaust fan relative to temperature
  <10C slow  >40 Fast and pro rata between the two adjust via tempMin & Max values
  Display Fridge temp via 5 led bar graph.
  1green to show on & working; 2green ~15; All on at 45+
  330 ohm on green leds 0 ohm on reds to make them brighter
  As DHT sensor is slow, put in a delay so data is read only every 5 seconds or therabouts 
  =============================================== ==========================================
  Pin Configs
  DHT22 data pin = pin2
  PWM Fan control = pin 11
  Bar graph led connected to pins 6-10
  
 */
 
#include "DHT.h"          //Sensor for fridge
#define DHTTYPE DHT22
#define DHTPIN 2          // digital pin DHT sensor connected to
DHT dht(DHTPIN, DHTTYPE);

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Define pins for led bar graph
  const byte green1 = 6;
  const byte green2 = 7;
  const byte green3 = 8;
  const byte red1 = 9;
  const byte red2 = 10; 

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

long lastSecond;        //The millis counter to see when a second rolls by
unsigned long lastmillis = 0;
int fanSpeed;
int fanPercent;
int ledCase;
int tempMin = 5;        //temperature at which fan starts
int tempMax = 40;       //temperature at which fan at 100%
int fanPin = 11;           //pin for fan control

void setup() {
  
    Serial.begin(9600);     //Start serial
  // Initialize led pins as output to drive bar graph
    pinMode(green1,OUTPUT);
    pinMode(green2,OUTPUT);
    pinMode(green3,OUTPUT);
    pinMode(red1,OUTPUT);
    pinMode(red2,OUTPUT);
    
  //Start the DHT sensor 
    dht.begin();
    Serial.println("DHT sensor online!");
  
//Set start of program to enable readings at specific intervals, via smart delay routine
    int seconds = 0;
    lastSecond = millis();    //Returns the number of miliseconds program has been running  
}
//End of Set Up============================================================================


void loop() {
  
//Check DHT sensor
      int t_dht = dht.readTemperature();
      Serial.print(" DHT temp ");
      Serial.print(t_dht);
      Serial.print("C");

//Fan Control
      fanSpeed = fanControl(t_dht);    // gets 0 if temp <5c and value between 32 and 255 for 40c
      //fanPercent = ((fanSpeed/255)*100);  //Gives percent of full speed
      analogWrite(fanPin, fanSpeed);   // sends PWM signal to pin_11 for fan control 
      Serial.print(" Fan Speed ");
      Serial.println(fanSpeed); 
      //Serial.println(fanPercent);

//LED Bar graph display get temp and derivet case for led display. Case must be integer
      int ledCase = caseNumber(t_dht);
      Serial.print(" Led Case = ");
      Serial.println(ledCase); 
      
     switch(ledCase)
     {
      case 1:                     //1 green irrespective of temp shows unit functioning            
      digitalWrite(green1,HIGH);
      digitalWrite(green2,LOW);
      digitalWrite(green3,LOW);
      digitalWrite(red1,LOW);
      digitalWrite(red2,LOW);
      break;
 
      case 2:                   
      digitalWrite(green1,HIGH);
      digitalWrite(green2,HIGH);
      digitalWrite(green3,LOW);
      digitalWrite(red1,LOW);
      digitalWrite(red2,LOW);
      break;

      case 3:
      digitalWrite(green1,HIGH);
      digitalWrite(green2,HIGH);
      digitalWrite(green3,HIGH);
      digitalWrite(red1,LOW);
      digitalWrite(red2,LOW);
      break;

      case 4:
      digitalWrite(green1,HIGH);
      digitalWrite(green2,HIGH);
      digitalWrite(green3,HIGH);
      digitalWrite(red1,HIGH);
      digitalWrite(red2,LOW);
      break;

      case 5:                   //Temp >45c
      digitalWrite(green1,HIGH);
      digitalWrite(green2,HIGH);
      digitalWrite(green3,HIGH);
      digitalWrite(red1,HIGH);
      digitalWrite(red2,HIGH);
      break;
     }

//Control the timing for re-running the data collection
//millis() gives the number of milliseconds program has been running
//lastSecond = program running time during setup stage
    if (millis() - lastSecond >= 1000)  //
  { 
    lastSecond += 1000; //Increment lastSecond by 1,000 
  } 
    smartDelay(5000); //We get here if program has been running for +1 second and pass 5000 to smartDelay (~5s) 
     
}     

//End of VOID Loop========================================================================


//Fan Control function accepts DHT temperature, returns fanSpeed for PWM
//By adjusting tempMin & tempMax you control when fan will activate, full speed and pro rate between
//Min fan speed of 32 used to prevent stalling

  int fanControl(float t_dht)
{
      if (t_dht<tempMin)
      {
        fanSpeed = 0;
      }
      else if (t_dht>tempMax)
      { 
        fanSpeed = 255;
      }
      else 
      {
        fanSpeed = map(t_dht, tempMin, tempMax, 32, 255); //32 min speed to prevent fan stalling
      } 
      return fanSpeed;
}  
//End of fanSpeed routine =========================================================================
  

//Routine to get case numbers 1-5 from t_dht to use in switch command for bar graph display.

int caseNumber(float t_dht)
{
    if (t_dht<tempMin)
    {
      ledCase = 1;    //one green light shows unit working at <5
    }
    else if (t_dht>tempMax+5)   //45c+
    {
      ledCase = 5;    //all on shows high temperature >45 
    }
    else
    {
      ledCase = map(t_dht,tempMin+10,tempMax,2,4);   // return 2,3 at(28c),4 at(40c) 
    }
    
    return ledCase;
}
//End of ledCase==========================================================================


//Routine to delay for a given amount of time passed to it viz (ms)
//As routine is called when void loop been running for +1s the delay will be ms+1s

  static void smartDelay(unsigned long ms)  
{
  unsigned long start = millis();   //time program been running
  do 
  {
    delay(100);                     //A small pause of 100 milliseconds to facilitate millis()
  } while (millis() - start < ms);  //Exit once time ticks over ms value passed
}

//End of smartdelay======================================================================================
