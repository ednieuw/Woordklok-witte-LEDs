// ==============================================================================================================================
// Title ..........: Woordklok
// Author .........: Ed Nieuwenhuys
// ===============================================================================================================================

#include <Wire.h>
#include <SPI.h>][
#include <RTClib.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#define HET	    Serial.print("Het ");   Bluetooth.print("Het ");      Display1=Display1 | (Toggle_HetWasIs<<0)
#define IS	    Serial.print("is ");    Bluetooth.print("is ");       Display1=Display1 | (Toggle_HetWasIs<<1)
#define WAS	    Serial.print("was ");   Bluetooth.print("was ");      Display1=Display1 | (Toggle_HetWasIs<<2)
#define MVIJF	  Serial.print("vijf ");  Bluetooth.print("vijf ");     Display1=Display1 | (1<<3)
#define MTIEN   Serial.print("tien ");  Bluetooth.print("tien ");     Display1=Display1 | (1<<4) 
#define KWART	  Serial.print("kwart "); Bluetooth.print("kwart ");    Display1=Display1 | (1<<5)
#define VOOR	  Serial.print("voor ");  Bluetooth.print("voor ");     Display1=Display1 | (1<<6)
#define OVER	  Serial.print("over ");  Bluetooth.print("over ");     Display1=Display1 | (1<<7)

#define PRECIES Serial.print("precies ");Bluetooth.print("precies "); Display2=Display2 | (1<<0)
#define HALF	  Serial.print("half ");   Bluetooth.print("half ");    Display2=Display2 | (1<<1)
#define ELF	    Serial.print("elf ");    Bluetooth.print("elf ");     Display2=Display2 | (1<<2)
#define VIJF	  Serial.print("vijf ");   Bluetooth.print("vijf ");    Display2=Display2 | (1<<3)
#define TWEE	  Serial.print("twee ");   Bluetooth.print("twee ");    Display2=Display2 | (1<<4)
#define EEN	    Serial.print("een ");    Bluetooth.print("een ");     Display2=Display2 | (1<<5)
#define VIER	  Serial.print("vier ");   Bluetooth.print("vier ");    Display2=Display2 | (1<<6)
#define TIEN	  Serial.print("tien ");   Bluetooth.print("tien ");    Display2=Display2 | (1<<7)

#define TWAALF	Serial.print("twaalf "); Bluetooth.print("twaalf ");  Display3=Display3 | (1<<0)
#define DRIE	  Serial.print("drie ");   Bluetooth.print("drie ");    Display3=Display3 | (1<<1)
#define NEGEN	  Serial.print("negen ");  Bluetooth.print("negen ");   Display3=Display3 | (1<<2)
#define ACHT	  Serial.print("acht ");   Bluetooth.print("acht ");    Display3=Display3 | (1<<3)
#define ZES	    Serial.print("zes ");    Bluetooth.print("zes ");     Display3=Display3 | (1<<4)
#define ZEVEN	  Serial.print("zeven ");  Bluetooth.print("zeven ");   Display3=Display3 | (1<<5)
#define UUR	    Serial.print("uur ");    Bluetooth.print("uur ");     Display3=Display3 | (1<<6)
#define EDSOFT	Serial.print("Edsoft "); Bluetooth.print("Edsoft ");  Display3=Display3 | (1<<7)

#define DS3231_I2C_ADDRESS          0x68 //57 //68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
// Digital hardware constants ----
enum DigitalPinAssignments {
  encoderPinB  = 2,               // left (labeled CLK on decoder)
  encoderPinA  = 3,               // right (labeled DT on decoder)
  clearButton  = 4,               // switch (labeled SW on decoder)
  PWMpin       = 5,               // Pin that controle PWM signal on BC327 transistor to dim light
  Empty6       = 6,               // Bluetooth RX
  Empty7       = 7,               // Bluetooth TX
  Empty8       = 8,               //
  AMPMpin      = 9,               // define pin voor AM PM Led
  LEDDataPin   = 10,              // blauw HC595
  LEDStrobePin = 11,              // groen HC595
  LEDClockPin  = 12,              // geel  HC595
  secondsPin   = 13};
// Analogue hardware constants ----
enum AnaloguePinAssignments {
 PhotoCellPin = 2,                // LDR pin
 Leeg3        = 3,                //
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5};               // SCL pin

#define BRIGHTNESS  200           // BRIGHTNESS 0 - 255
byte BrightnessCalcFromLDR = BRIGHTNESS;
                                  //KY-040 rotary encoder ----------
Encoder myEnc(2, 3);              // Use digital pin 2 and 3 for encoder
long Looptime;
unsigned long RotaryPressTimer = 0;
                                  // Photocell ---------------------
float LightReducer = 0.80 ;       // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
int OutPhotocell;                 // stores reading of photocell;
int MinPhotocell = 100;           // stores minimum reading of photocell;
int MaxPhotocell = 100;           // stores maximum reading of photocell;
                                  // Clock -------------------------
static unsigned long msTick =0;   // the number of millisecond ticks since we last incremented the second counter
int  count; 
int  Delaytime = 200;
byte Display1 = 0, Display2 = 0, Display3 = 0;
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte lastminute = 0, lasthour = 0, sayhour = 0;
byte Toggle_HetWasIs    = 1;      // Turn On/Off HetIsWas lights
byte Toggle_HetWasIsUit = 0;      // Turn off HetIsWas after 10 sec
byte SecPulse           = 0;      // give a pulse to the second led
RTC_DS1307 RTC;
DateTime now;
                                   // Bluetooth ---------------------
SoftwareSerial Bluetooth(6, 7);    // RX, TX
String BluetoothString;

void SWversion(void) 
{
 Serial.println(F("======================================="));
 Serial.println(F("Title ..........: Woordklok zwartwit met rotary KY-040, Temp, Bluetooth"));
 Bluetooth.println("Woordklok No xx connected  ");
 Serial.println(F("Author    ......: Ed Nieuwenhuys"));
 Serial.println(F("Version.........: 0.76 Mrt-2016"));
 Serial.println(F("Changes.........: 0.27-->0.40 read clock every second instead of every hour"));
 Serial.println(F("Changes.........: 0.27-->0.40 Encoder turn writes time directly to clock"));
 Serial.println(F("Changes.........: 0.42 toggle HetWasIs"));
 Serial.println(F("Changes.........: 0.43 Added PWM on Pin5 to control dimming "));
 Serial.println(F("Changes.........: 0.48 Minor changes"));
 Serial.println(F("Changes.........: 0.49 rotary encoder improved"));
 Serial.println(F("Changes.........: 0.50 Stable and organised coding"));
 Serial.println(F("Changes.........: 0.51 (SQRT (long)analog read *  63.5) for less lineair dimming"));
 Serial.println(F("Changes.........: 0.52 Changed rotary pins"));
 Serial.println(F("Changes.........: 0.54 Coding voor Klok 3 en 5"));
 Serial.println(F("Changes.........: 0.60 Nieuwe display format. Hetiswas na 10 sec uit "));
 Serial.println(F("Changes.........: 0.61 Pinchanges"));
 Serial.println(F("Changes.........: 0.63 Ebben klok No13"));
 Serial.println(F("Changes.........: 0.64 Light reducer added"));
 Serial.println(F("Changes.........: 0.65 Programma aangepast voor standaard front en KY-040 rotary-besturing ingebouwd"));
 Serial.println(F("Changes.........: 0.66 Random tijd when no RTC signal"));
 Serial.println(F("Changes.........: 0.67 Source code cleaning"));
 Serial.println(F("Changes.........: 0.69 Bluetooth added"));
 Serial.println(F("Changes.........: 0.70 Temperature added"));
 Serial.println(F("Changes.........: 0.76 Rotary delay 200 ---> 0 ms"));
 Serial.print(F(  "LightReducer: ")); Serial.println(LightReducer);
 Serial.println(F("========================================")); 
}
// --------------------------- SETUP ----------------------------------
void setup()
{                                              // initialise the hardware // initialize the appropriate pins as outputs:
  pinMode(LEDClockPin,  OUTPUT); 
  pinMode(LEDDataPin,   OUTPUT); 
  pinMode(LEDStrobePin, OUTPUT); 
  pinMode(PWMpin,       OUTPUT);
  pinMode(secondsPin,   OUTPUT );
  pinMode(encoderPinA,  INPUT_PULLUP);
  pinMode(encoderPinB,  INPUT_PULLUP);  
  pinMode(clearButton,  INPUT_PULLUP);
  Serial.begin(9600);                          // setup the serial port to 9600 baud 
  Bluetooth.begin(9600);                       // setup the Bluetooth port to 9600 baud 
  analogWrite(PWMpin, 255);                    // the duty cycle: between 0 (lights off) and 255 (light full on).
  Wire.begin();
  RTC.begin();
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println("RTC is older than compile time! Updating");         // following line sets the RTC to the date & time this sketch was compiled
   RTC.adjust(DateTime(__DATE__, __TIME__));
  } 
  if (EEPROM.read(0) <10 || EEPROM.read(0) > 100) EEPROM.write(0, 80); // default intensity for this clock
  LightReducer = ((float) EEPROM.read(0) / 100);                       // store it is the work variabele
  Looptime = millis();                         // Used in KY-040 rotary
  SWversion();                                 // Display the version number of the software
  Selftest();                                  // Play the selftest
  GetTijd(1);                                  // Get the time and print it to serial
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
//------------------------------- LOOP -----------------------------------
void loop(void)
{
 if ( millis() - msTick >999)                // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                        // second++; 
   ++SecPulse;                               // blinks Led13 in function DimLeds
   GetTijd(0);                               // synchronize time with RTC clock
   if ((Toggle_HetWasIsUit == 2) && (second > 10)) Toggle_HetWasIs = 0  ; else Toggle_HetWasIs = 1;  // HET IS WAS staat aan
   DimLeds();                                                         //Led Intensity Control + seconds tick
   if ((Toggle_HetWasIsUit == 2) && (second ==11)) Displaytime();     // turn Leds OFF on second == 11
 //  Serial.print(Toggle_HetWasIsUit);    Serial.print(" : ");     Serial.print(Toggle_HetWasIs); Serial.print(" : ");
  }
 while (Bluetooth.available())
  {
   delay(3);  
   char c = Bluetooth.read();
   if (c>31 && c<128) BluetoothString += c;    
  }
 if (BluetoothString.length() >0)     ReworkBluetoothString();      // Rework BluetoothString();
 BluetoothString="";
 if (minute != lastminute)                                          //show time every minute
  { 
   lastminute = minute;
   Displaytime();
   Print_RTC_tijd();
   Print_BTtijd();                                                 // print time to Bluetooth
  } 
 if (hour != lasthour) {lasthour = hour;}

   
//------------------------ ky-040 rotary encoder -------------------------
// If button pressed, 60 sec after start ATMEGA, then there are 60 seconds to adjust the light intensity. 
// RotaryPressTimer is the time in millisec after start ATMEGA 
 long encoderPos = myEnc.read()/2;
 if (encoderPos && (millis() - Looptime >10)) 
  {   
   Serial.print("Index:");
   Serial.println(encoderPos, DEC);
   if (encoderPos >0)                                              // increase the MINUTES
    {
     if ( millis() > 60000 && (millis() - RotaryPressTimer) < 60000){ WriteLightReducer(0.05); }     // If time < 60 sec then adjust light intensity factor
     else 
     {
     if( ++minute >59) { minute = 0; second = 0;  }
     SetTime();  
 //    delay(50);                                                  // delay to prevent bouncing
     myEnc.write(0);                                               // Set encoder pos back to 0
    }     
     }
   if (encoderPos <0)                                              // increase the HOURS
    {
     if (millis() > 60000 &&  (millis() - RotaryPressTimer) < 60000) { WriteLightReducer(-0.05); }    // If time < 60 sec then adjust light intensity factor
     else
      { 
      if( ++hour >23) { hour = 0; }
      SetTime();   
      }
 //  delay(50);                                                     // delay to prevent bouncing
   myEnc.write(0);                                                  // Set encoder pos back to 0      
    }                                                
  }
 if (digitalRead(clearButton) == LOW )                              // set the time by pressing rotary button
  { 
    delay(200);
    RotaryPressTimer =  millis();                                   // If time < 60 sec then adjust light intensity factor
    Toggle_HetWasIsUit++;
    if (Toggle_HetWasIsUit >= 3)  { Toggle_HetWasIsUit = 0 ; }
    if (Toggle_HetWasIsUit == 0)  { Toggle_HetWasIs = 1;} // On
    if (Toggle_HetWasIsUit == 1)  { Toggle_HetWasIs = 0;} // Off
    if (Toggle_HetWasIsUit == 2)  { Toggle_HetWasIs = 0; Play_Lights(); } // Off after 10 sec
    Serial.print("Toggle_HetWasIsUit: ");   Serial.println(Toggle_HetWasIsUit);
    Serial.print("Toggle_HetWasIs: ");      Serial.println(Toggle_HetWasIs);    
    Displaytime();
    myEnc.write(0);  
   }
 Looptime = millis();
}	
//------------------------------- End LOOP 

void Selftest(void)
{
  GetTijd(1);             //Prints time in Serial monitor
  LedsOff(); 
  HET;     Laatzien();
  IS;      Laatzien();
  WAS;     Laatzien();
  MVIJF;   Laatzien();
  MTIEN;   Laatzien();
  KWART;   Laatzien(); 
  VOOR;    Laatzien();
  OVER;    Laatzien();
  PRECIES; Laatzien(); 
  HALF;    Laatzien();
  ELF;     Laatzien();  
  VIJF;    Laatzien();
  TWEE;    Laatzien();  
  EEN;     Laatzien();
  VIER;    Laatzien();
  TIEN;    Laatzien();
  TWAALF;  Laatzien();
  DRIE;    Laatzien();
  NEGEN;   Laatzien(); 
  ACHT;    Laatzien();
  ZES;     Laatzien(); 
  ZEVEN;   Laatzien();
  UUR;     Laatzien();
  EDSOFT;  Laatzien();
   
  for(int i=0; i<2; i++)
  {
   Display1=255;   Display2=255;   Display3=255;  Laatzien();
   Display1=0;     Display2=0;     Display3=0;    Laatzien();
  }  
  Play_Lights();     
  Displaytime();
}
// -------------------------- End Selftest   --------------------------
//--------------------------- Time functions --------------------------
void Displaytime(void)
{
 LedsOff();                                  // start by clearing the display to a known state
 HET;                                        // HET light is always on
 switch (minute)
 {
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; UUR; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  MTIEN; VOOR; HALF; break;
  case 22: 
  case 23: WAS; MTIEN; VOOR; HALF; break;
  case 24: 
  case 25: 
  case 26: IS;  MVIJF; VOOR; HALF; break;
  case 27: 
  case 28: WAS; MVIJF; VOOR; HALF; break;
  case 29: IS;  HALF; break;
  case 30: IS;  PRECIES; HALF; break;
  case 31: IS;  HALF; break;
  case 32: 
  case 33: WAS; HALF; break;
  case 34: 
  case 35: 
  case 36: IS;  MVIJF; OVER; HALF; break;
  case 37: 
  case 38: WAS; MVIJF; OVER; HALF; break;
  case 39: 
  case 40: 
  case 41: IS;  MTIEN; OVER; HALF; break;
  case 42: 
  case 43: WAS; MTIEN; OVER; HALF; break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
}
 if (hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);
 sayhour = hour;
 if (minute > 18 )  sayhour = hour+1;
 if (sayhour == 24) sayhour = 0;

 switch (sayhour)
 {
  case 13:
  case 1: EEN; break;
  case 14:
  case 2: TWEE; break;
  case 15:
  case 3: DRIE; break;
  case 16:
  case 4: VIER; break;
  case 17:
  case 5: VIJF; break;
  case 18:
  case 6: ZES; break;
  case 19:
  case 7: ZEVEN; break;
  case 20:
  case 8: ACHT; break;
  case 21:
  case 9: NEGEN; break;
  case 22:
  case 10: TIEN; break;
  case 23:
  case 11: ELF; break;
  case 0:
  case 12: TWAALF; break;
 } 
 switch (minute)
 {
  case  0: UUR; break;
  case  1: UUR;  break;
  case 59: UUR;  break;
 }
 Bluetooth.println("");
 WriteLEDs();
}

void GetTijd(byte printit)
{
 now =    RTC.now();
 hour =   now.hour();
 minute = now.minute();
 second = now.second();
 if (hour > 24) { hour = random(12)+1; minute = random(60)+1; second = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

void Print_RTC_tijd(void)
{
 now = RTC.now();
 if (now.hour() < 10) Serial.print("0");
 Serial.print(now.hour(), DEC);
 Serial.print(":");
 if (now.minute() < 10) Serial.print("0");
 Serial.print(now.minute(), DEC);
 Serial.print(":"); 
 if (now.second() < 10) Serial.print("0");
 Serial.print(now.second(), DEC);
 Serial.print("  ");
 Serial.print(now.day(), DEC);
 Serial.print("/");
 Serial.print(now.month(), DEC);
 Serial.print("/");
 Serial.println(now.year(), DEC); 
}

void Print_tijd(void)
{
 if (hour < 10) Serial.print("0");
 Serial.print(hour);
 Serial.print(":");
 if (minute < 10) Serial.print("0");
 Serial.print(minute);
 Serial.print(":");
 if (second < 10) Serial.print("0");
 Serial.println(second);
}

void Print_BTtijd(void)
{
 Bluetooth.print("T: ");
 if (hour < 10) Bluetooth.print("0");
 Bluetooth.print(hour);
 Bluetooth.print(":");
 if (minute < 10) Bluetooth.print("0");
 Bluetooth.print(minute);
 Bluetooth.print(":");
 if (second < 10) Bluetooth.print("0");
 Bluetooth.print(second);
 Bluetooth.print(" I:");         
 Bluetooth.print(( int) (BrightnessCalcFromLDR/2.55)); 
 Bluetooth.print("% LDR:");
 Bluetooth.print(analogRead(PhotoCellPin)); 
 Bluetooth.print(" T:"); 
 Bluetooth.print(get3231Temp());
 Bluetooth.println("C"); 
}

void SetTime(void)
{ 
 RTC.adjust(DateTime(now.year(), now.month(), now.day(), hour, minute, second));
 GetTijd(0);                               // synchronize time with RTC clock
 Displaytime();
 Print_tijd();
// delay(300);
}
// ------------------- End  Time functions ------------------------------

// --------------------Light functions -----------------------------------
void LedsOff(void){  Display1=0;  Display2=0;  Display3=0; }

void WriteLEDs(void) 
{                          // Write the actual values to the hardware                                      
 digitalWrite(LEDStrobePin,LOW);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display3);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display2);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display1);
 digitalWrite(LEDStrobePin,HIGH);
 delay(2);
 Serial.print(Display1, BIN);
 Serial.print(":");
 Serial.print(Display2, BIN);
 Serial.print(":");
 Serial.println(Display3, BIN);
}

void Laatzien()
{ 
 WriteLEDs();  
 delay(Delaytime);  
 LedsOff();
}

void DimLeds() 
{                                               // dim the leds by PWM and tick the seconds on pin 13                                                                 
 if (SecPulse) 
 {
  digitalWrite(secondsPin,HIGH);                // tick the second on pin 13
//  delayMicroseconds(500); 
  int LDR_read = analogRead(PhotoCellPin);
                                                // Read lightsensor
  OutPhotocell = (int) (LightReducer * sqrt( (float) 63.5 * constrain(LDR_read,10,1023))); // filter out of strange results and set minimum brightness
  MinPhotocell = MinPhotocell > OutPhotocell ? OutPhotocell : MinPhotocell;
  MaxPhotocell = MaxPhotocell < OutPhotocell ? OutPhotocell : MaxPhotocell;
  Serial.print(F("SensorRead: "));       Serial.print(LDR_read);
  Serial.print(F("  LightReducer: "));   Serial.print((int) (LightReducer * 100)); 
  Serial.print(F("%  Min: "));           Serial.print(MinPhotocell);  
  Serial.print(F("  Max: "));            Serial.print(MaxPhotocell);      
  Serial.print(F("  Now: "));           Serial.print(OutPhotocell);  
  BrightnessCalcFromLDR = constrain(OutPhotocell, 30 , 255);                              // filter out of strange results
  Serial.print(F(" LI Now: "));         Serial.print(( int) (BrightnessCalcFromLDR/2.55)); 
  Serial.print(F("%"));    
  Serial.print(F(" Temp: ")); Serial.print(get3231Temp());   Serial.print(F("C "));   
  Print_tijd();
  analogWrite(PWMpin, BrightnessCalcFromLDR);  // write PWM 
  digitalWrite(secondsPin,LOW); 
 }
 SecPulse = 0;
}

void Play_Lights()
{
 Display1=255;   Display2=255;   Display3=255; Laatzien();
 for (int n=255 ; n>=0; n--) { analogWrite(PWMpin, n); delay(2);}    // the duty cycle: between 0 (lights off) and 255 (light full on).
 for (int n=0 ; n<=255; n++) { analogWrite(PWMpin, n); delay(2);}  
 LedsOff();
}

void WriteLightReducer(float amount)
{
 LightReducer += amount; 
 if (LightReducer < 0.00 ) LightReducer = 0.05;
 if (LightReducer > 1.00   ) LightReducer = 1.00;
 EEPROM.write(0, (int) (LightReducer * 100));                       // Store the value (0-100) in permanent EEPROM memory at address 0
 Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
 Serial.print(LightReducer * 100); Serial.println("%");
}
// --------------------End Light functions --------------------------------------

void ReworkBluetoothString(void)
{
 String temp;
 Serial.println(BluetoothString);
 if (BluetoothString.length() > 3 && BluetoothString.length() <7 )
 {
  temp = BluetoothString.substring(0,2);   hour = temp.toInt(); 
  if (BluetoothString.length() > 3) { temp = BluetoothString.substring(2,4); minute = temp.toInt(); }
  if (BluetoothString.length() > 5) { temp = BluetoothString.substring(4,6); second = temp.toInt(); }
  BluetoothString="";
  temp="";
  SetTime();
 }
}
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  //temp registers (11h-12h) get updated automatically every 64s
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read(); //2's complement int portion
    tLSB = Wire.read(); //fraction portion 
    temp3231 = (tMSB & B01111111); //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5; //only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else {  temp3231 = -273; }   
  return (temp3231);
}


