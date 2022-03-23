// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 ot ARMEGA1284 chip on the PCB board that controls the LED strips of the Word Clock.
This minimum source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
A 74HC595 ULN2803APG combination regulates the LEDs by shifting in bits into the 74HC595 LED are turn On or Off
A FT232RL 5.5V FTDI USB to TTL Serial Module can be attached to program te ATMEGA and read the serial port

This is the smallest program to run a word clock with white LEDs like the 2835 or 3528 white LEDs
Have a look at Character_Clock_V112 or higher version for more functionality

 Author .: Ed Nieuwenhuys 2021
 Changes : V1.0.0  Derived from: Character_Clock_V112_328-1284  
 */
// ===============================================================================================================================
char VERSION[] = "WhiteLED_Word_clock_V1.0.0";
//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
#define MOD_DS3231                 // The DS3231 module is installed, if not defined the Arduino internal clock is used
#define ROTARYMOD                  // Use a rotary encode

#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Encoder.h>

//--------------------------------------------
// ARDUINO Definition of installed language word clock
// Choose Dutch NL or english UK
//--------------------------------------------
 #define NL         
// #define UK

                    #ifdef NL
#define HET     ShiftInTime("Het ",    0, 1,Toggle_HetWasIs,0);
#define IS      ShiftInTime("is ",     1, 1,Toggle_HetWasIs,1);
#define WAS     ShiftInTime("was ",    2, 1,Toggle_HetWasIs,2);
#define MVIJF   ShiftInTime("vijf ",   3, 1,              1,3);
#define MTIEN   ShiftInTime("tien ",   4, 1,              1,4);
#define KWART   ShiftInTime("kwart ",  5, 1,              1,5);
#define VOOR    ShiftInTime("voor ",   6, 1,              1,6);
#define OVER    ShiftInTime("over ",   7, 1,              1,7);

#define PRECIES ShiftInTime("precies ",8, 2,              1,0);
#define HALF    ShiftInTime("half ",   9, 2,              1,1);
#define ELF     ShiftInTime("elf ",   10, 2,              1,2);
#define VIJF    ShiftInTime("vijf ",  11, 2,              1,3);
#define TWEE    ShiftInTime("twee ",  12, 2,              1,4);
#define EEN     ShiftInTime("een ",   13, 2,              1,5);
#define VIER    ShiftInTime("vier ",  14, 2,              1,6);
#define TIEN    ShiftInTime("tien ",  15, 2,              1,7);

#define TWAALF  ShiftInTime("twaalf ",16, 3,              1,0);
#define DRIE    ShiftInTime("drie ",  17, 3,              1,1);
#define NEGEN   ShiftInTime("negen ", 18, 3,              1,2);
#define ACHT    ShiftInTime("acht ",  19, 3,              1,3);
#define ZES     ShiftInTime("zes ",   20, 3,              1,4);
#define ZEVEN   ShiftInTime("zeven ", 21, 3,              1,5);
#define UUR     ShiftInTime("uur ",   22, 3,              1,6);
#define EDSOFT  ShiftInTime("Edsoft ",23, 3, ToggleEdsoft  ,7);
#endif //NL
#ifdef UK

#define HET     ShiftInTime("It ",     0, 1,Toggle_HetWasIs,0);
#define IS      ShiftInTime("is ",     1, 1,Toggle_HetWasIs,0);
#define WAS     ShiftInTime("was ",    2, 1,Toggle_HetWasIs,0);
#define PRECIES ShiftInTime("exact "  ,3, 1,              1,3);
#define HALF    ShiftInTime("half ",   4, 1,              1,4);
#define TWINTIG ShiftInTime("twenty ", 5, 1,              1,5);
#define MVIJF   ShiftInTime("five ",   6, 1,              1,6);
#define KWART   ShiftInTime("quarter ",7, 1,              1,7);

#define MTIEN   ShiftInTime("ten ",    8, 2,              1,0);
#define OVER    ShiftInTime("past ",   9, 2,              1,1);
#define ZES     ShiftInTime("six ",   10, 2,              1,2);
#define TWEE    ShiftInTime("two ",   11, 2,              1,3);
#define VIJF    ShiftInTime("five ",  12, 2,              1,4);
#define TWAALF  ShiftInTime("twelve ",13, 2,              1,5);
#define TIEN    ShiftInTime("ten ",   14, 2,              1,6);
#define ELF     ShiftInTime("eleven ",15, 2,              1,7);

#define VIER    ShiftInTime("four ",  16, 3,              1,0);
#define NEGEN   ShiftInTime("nine ",  17, 3,              1,1);
#define DRIE    ShiftInTime("three ", 18, 3,              1,2);
#define ACHT    ShiftInTime("eight ", 19, 3,              1,3);
#define ZEVEN   ShiftInTime("seven ", 20, 3,              1,4);
#define EEN     ShiftInTime("one ",   21, 3,              1,5);
#define UUR     ShiftInTime("O'clock",22, 3,              1,6);
#define VOOR    ShiftInTime("to "     23, 3, ToggleEdsoft  ,7);
#endif //UK


//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 

// Digital hardware constants ATMEGA 328 ----
enum DigitalPinAssignments {
  BT_TX        = 0,               // BT_TX connect to RX op de BT module
  BT_RX        = 1,               // BT_RX connect to TX op de BT module
  encoderPinB  = 2,               // Left (labeled CLK on decoder)no interrupt pin  
  encoderPinA  = 3,               // Right (labeled DT on decoder)on interrupt  pin
  clearButton  = 4,               // Switch (labeled SW on decoder)
  PWMpin       = 5,               // Pin that controle PWM signal on BC327 transistor to dim light
  Empty06      = 6,               // Empty06
  Empty07      = 7,               // Empty07
  Empty08      = 8,               // Empty08
  Empty09      = 9,               // Empty09
  LEDDataPin   = 10,              // blauw HC595
  LEDStrobePin = 11,              // groen HC595
  LEDClockPin  = 12,              // geel  HC595
  secondsPin   = 13};
                                  // Analogue hardware constants ----
enum AnaloguePinAssignments {
  EmptyA0       = 0,              // A0
  EmptyA1       = 1,              // A1
  PhotoCellPin  = 2,              // LDR pin
  OneWirePin    = 3,              // 
  SDA_pin       = 4,              // SDA pin
  SCL_pin       = 5};             // SCL pin

//--------------------------------------------
// KY-040 ROTARY
//--------------------------------------------                    
Encoder myEnc(encoderPinA, encoderPinB);    // Use digital pin  for encoder
unsigned long Looptime = 0;
unsigned long RotaryPressTimer  = 0;
byte          NoofRotaryPressed = 0;
//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
const byte MAXBRIGHTNESS    = 70;           // Factor (%) to dim LED intensity with. Between 1% and 250%
const byte LOWBRIGHTNESS    = 5;            // Lower limit of Brightness ( 0 - 255)   
bool  LEDsAreOff            = false;        // If true LEDs are off except time display
byte  BrightnessCalcFromLDR = 32;           // Initial brightness value The intensity send to the LEDs (0-255)
int   Previous_LDR_read     = 512;          // The actual reading from the LDR + 4x this value /5
int   OutPhotocell;                         // Stores reading of photocell;
int   MinPhotocell          = 1024;         // Stores minimum reading of photocell;
int   MaxPhotocell          = 1;            // Stores maximum reading of photocell;
int   ToggleEdsoft          = 1;

//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 50
static unsigned long msTick;                // Millisecond ticks 
int    count; 
byte   Isecond, Iminute, Ihour , Iday, Imonth, Iyear; 
byte   Display1 = 0 , Display2 = 0, Display3 = 0;
byte   lastday  = 0,lastminute = 0, lasthour = 0, sayhour  = 0;
byte   Toggle_HetWasIs      = 1;            // Turn On/Off HetIsWas lights
int    Toggle_HetWasIsUit   = 0;            // Turn off HetIsWas after 10 sec
byte   ChangeTime           = false;
byte   ChangeLightIntensity = false;
byte   SecPulse             = 0;            // Give a pulse to the Isecond led
byte   Demo                 = false;
byte   Zelftest             = false;

//--------------------------------------------
// Menu
//--------------------------------------------  
//0        1         2         3         4         5
//12345678901234567890123456789012345678901234567890
// do not make the strings longer than the menu[][nn]
const char menu[][41] PROGMEM =  {
 "Word clock",
 "Enter time as hhmmss or hhmm",
 "A Normal display",
 "B Suppress Het Is Was in display",
 "E Suppress Het Is Was after 10 seconds",
 "C Reset settings",
 "D D15122017 is 15 December 2017",
 "L (L5) Min light intensity (0-255)",
 "M (M70) Max light intensity (1-250)",
 "N (N2208) Turn OFF LEDs between Nhhhh",
 "I For this info",
 "R Reset to default settings",
 "S Self test",
 "X Demo mode",
 "Ed Nieuwenhuys Mei 2021" };

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;    //RTC_DS1307 RTC; 
        #else
RTC_Millis RTCklok;   
        #endif  //
DateTime Inow;


//----------------------------------------
// Common
//----------------------------------------
char sptext[MAXTEXT+2];                                               // For common print use                                      
struct EEPROMstorage {                                                // Data storage in EEPROM to maintain them after power loss
  byte LightReducer;
  byte LowerBrightness;
  byte TurnOffLEDsAtHH;
  byte TurnOnLEDsAtHH;
} Mem ;
                                          
  // End Definitions  ---------------------------------------------------------
                               
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
 SerialCheck();
 RotaryEncoderCheck(); 
 if (Demo)          Demomode();
 else if (Zelftest) Selftest(); 
 else               EverySecondCheck();     
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                                                     // Initialise the hardware // initialize the appropriate pins as outputs:
 Serial.begin(9600);                                                  // Setup the serial port to 9600 baud 
 Tekstprintln("***\nSerial started\nInstalled:");
 Wire.begin();                                                        // Start the wire communication I2C
                                  #ifdef  MOD_DS3231
  RTCklok.begin();                                                     // Start the DS3231 RTC-module
                                  #endif //MOD_DS3231
 pinMode(LEDClockPin,  OUTPUT); 
 pinMode(LEDDataPin,   OUTPUT); 
 pinMode(LEDStrobePin, OUTPUT); 
 pinMode(PWMpin,       OUTPUT);
 pinMode(secondsPin,   OUTPUT );
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary"); 
 myEnc.write(0);                                                    // Clear Rotary encode buffer
 analogWrite(PWMpin, BrightnessCalcFromLDR);                        // The duty cycle: between 0 (lights off) and 255 (light full on).    
 GetTijd(1);                                                        // Get the time and print it to serial                      
 DateTime now = RTCklok.now();                                      // Get the time from the RTC
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
   {
    Tekstprintln("RTC Updating");                                   // Following line sets the RTC to the date & time this sketch was compiled
    RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
   } 
 EEPROM.get(0,Mem);                                                 // Get the data from EEPROM
 Mem.LightReducer    = constrain(Mem.LightReducer,   1, 250);       // and constrain the values
 Mem.LowerBrightness = constrain(Mem.LowerBrightness,1, 250);       // 
 Mem.TurnOffLEDsAtHH = min(Mem.TurnOffLEDsAtHH, 23);                //
 Mem.TurnOnLEDsAtHH  = min(Mem.TurnOnLEDsAtHH,  23);                //
 EEPROM.put(0,Mem);                                                 // Update EEPROM if some data are out of the constrains                                          
 SWversion();                                                       // Display the version number of the software
// Selftest();                                                      // Play the selftest
 Previous_LDR_read = analogRead(PhotoCellPin);                      // to have a start value
 MinPhotocell      = Previous_LDR_read;                             // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                             // Stores maximum reading of photocell;
 Looptime     = millis();                                           // Used in KY-040 rotary
 msTick       = Looptime; 
}
// --------------------------- END SETUP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 unsigned int i;
 PrintLine(40);
 for (i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)   {strcpy_P(sptext, menu[i]);            Tekstprintln(sptext);}
 PrintLine(40);
 sprintf(sptext,"  Brightness Min: %3d bits Max: %3d%%",Mem.LowerBrightness, Mem.LightReducer); Tekstprintln(sptext);
 sprintf(sptext,"    LDR read Min:%4d bits Max: %3d bits",MinPhotocell, MaxPhotocell);          Tekstprintln(sptext); 
 sprintf(sptext,"LEDs off between: %02d - %02d",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH);       Tekstprintln(sptext);
 sprintf(sptext,"Software: %s",VERSION);                                                        Tekstprintln(sptext); 
 GetTijd(1);  
 PrintLine(40);
}
void PrintLine(byte Lengte)
{
 for (int n=0; n<Lengte; n++) {Serial.print(F("_"));} Serial.println(); 
}

//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 Mem.LightReducer     = MAXBRIGHTNESS;                              // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.LowerBrightness  = LOWBRIGHTNESS;                              // Lower limit of Brightness ( 0 - 255)
 Mem.TurnOffLEDsAtHH  = Mem.TurnOnLEDsAtHH = 0;                     // Reset Off On time
 EEPROM.put(0,Mem);                                                 // Update EEPROM if some data are out of the constrains                                                 
 Previous_LDR_read    = analogRead(PhotoCellPin);                   // To have a start value
 MinPhotocell         = Previous_LDR_read;                          // Stores minimum reading of photocell;
 MaxPhotocell         = Previous_LDR_read;                          // Stores maximum reading of photocell;
 ChangeTime           = false;
 ChangeLightIntensity = false;
 Demo                 = false;
 Zelftest             = false;
 Selftest();                                                        // Play the selftest
 GetTijd(0);                                                        // Get the time and store it in the proper variables
 SWversion();                                                       // Display the version number of the software
 Displaytime();
}

//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{ 
  uint32_t ms = millis() - msTick;                                  // A Digitalwrite() is very time consuming. 
  static bool Dpin;                                                 // Only write once to improve program speed in the loop()
  if ( ms >1 && Dpin) {Dpin = LOW; digitalWrite(secondsPin,LOW);}   // Turn OFF the second on pin 13 
  if ( ms >999)                                                     // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                               // second++; 
   digitalWrite(secondsPin,HIGH);                                   // turn ON the second on pin 13
   Dpin = HIGH;
   ++SecPulse;                                                      // second routine in function DimLeds
   GetTijd(0);                                                      // synchronize time with RTC clock
   if ((Toggle_HetWasIsUit == 2) && (Isecond > 10)) Toggle_HetWasIs = 0; 
    else Toggle_HetWasIs = 1;                                       // HET IS WAS is On
   if(Isecond % 30 == 0) DimLeds(true);                             // Led Intensity Control + seconds tick print every 30 seconds   
    else                 DimLeds(false);
  if ((Toggle_HetWasIsUit == 2) && (Isecond == 11))Displaytime();   // turn Leds OFF on second == 11
  if(Iminute == 0 && Isecond <9)
   { 
    ToggleEdsoft = Isecond % 2;                                     // ToggleEdsoft becomes 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light on pin 24
    Serial.println(ToggleEdsoft);
    Displaytime();
   }
  if (Iminute != lastminute) EveryMinuteUpdate();  
  }
 }
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
  lastminute = Iminute;
  Displaytime();
  Print_RTC_tijd();
  if(Ihour != lasthour) EveryHourUpdate();
 }
//--------------------------------------------
// CLOCK Update routine done every hour
//--------------------------------------------
void EveryHourUpdate(void)
{
 GetTijd(0); 
 if(Ihour == Mem.TurnOffLEDsAtHH) LEDsAreOff = true;                // Is it time to turn off the LEDs?
 if(Ihour == Mem.TurnOnLEDsAtHH)  LEDsAreOff = false;               // Or on?
 lasthour = Ihour;
 if (Iday != lastday) EveryDayUpdate(); 
}

//--------------------------------------------
// CLOCK Update routine done every day
//--------------------------------------------
void EveryDayUpdate(void)
{
 lastday = Iday;                                                    // Not much to do at the moment
 MinPhotocell = 999;                                                // Stores minimum reading of photocell;
 MaxPhotocell = 1;                                                  // Stores maximum reading of photocell;
}
//--------------------------------------------
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
  if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                               // second++; 
   digitalWrite(secondsPin,HIGH);                                   // turn ON the second on pin 13
   ++SecPulse;                                                      // second routine in function DimLeds
   if( ++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
    if(    Ihour >24)   Ihour = 0;
   DimLeds(false);
   Displaytime();
   Tekstprintln("");
  }
}

//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char const *tekst)
{
 Serial.print(tekst);    
}
void Tekstprintln(char const *tekst)
{
 Serial.println(tekst);    
}
 //--------------------------------------------
// CLOCK check for input from devices
// This fubction is called from the loop()
//--------------------------------------------
void InputDevicesCheck(void)
{
 SerialCheck();    
 RotaryEncoderCheck(); 
}

//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String SerialString;
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<123) SerialString += c;                            // Allow input from Space - Del
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString
 SerialString = "";
}

//------------------------ KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
// Due to AC power spikes the rotary pins on the ATMEGA can detect an erroneous signal
// causing the time to change. Therefore the shaft has to be pressed before settings 
// can be changed for a period of 60 seconds.
//--------------------------------------------
void RotaryEncoderCheck(void)
{
 long encoderPos = myEnc.read();
 if ( (unsigned long) (millis() - RotaryPressTimer) > 60000)        // After 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        Tekstprintln(PSTR("Changing time is over"));
        NoofRotaryPressed = 0;
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }  
 if (ChangeTime || ChangeLightIntensity)                            // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos && ( (millis() - Looptime) > 200))               // If rotary turned avoid debounce within 0.05 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos >0)                                            // Increase  MINUTES of light intensity
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(0.05); }    // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                             // Change hours
              {
               if( ++Ihour >23) { Ihour = 0; }
              }      
           if (NoofRotaryPressed == 2)                             // Change minutes
              { 
               Isecond = 0;
               if( ++Iminute >59) { Iminute = 0; if( ++Ihour >23) { Ihour = 0; } }   
              }
           } 
        }    
      if (encoderPos <0)                                           // Increase the HOURS
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-0.05); }   // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                             // Change hours
            {
             if( Ihour-- ==0) { Ihour = 23; }
            }      
           if (NoofRotaryPressed == 2)                             // Change minutes
            { 
             Isecond = 0;
             if( Iminute-- == 0) { Iminute = 59; if( Ihour-- == 0) { Ihour = 23; } }   
            }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
      myEnc.write(0);                                               // Set encoder pos back to 0
      Looptime = millis();       
     }                                                
   }
 if (digitalRead(clearButton) == LOW )                              // Set the time by pressing rotary button
   { 
    delay(200);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    RotaryPressTimer      = millis();                               // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >3 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                      // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true;  BlinkUUR(3, 70);       Displaytime();                  break; // Change the hours
       case 2:  ChangeTime = true;  BlinkHETISWAS(3, 70);  Displaytime();                  break; // Change the hours   
       case 3:  ChangeLightIntensity = true;  Display1=Display2=Display3=255; Laatzien();  break; // Turn on all LEDs and change intensity 
       default: NoofRotaryPressed = 0; 
                ChangeTime            = false;
                ChangeLightIntensity  = false;  
                Selftest();        
                break; // Reset all settings
      }
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed);   
    Toggle_HetWasIs = 1;                                            // Make sure HET IS WAS is on
    myEnc.write(0);
    Looptime = millis();       
   }
  myEnc.write(0);
 }

//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{
  GetTijd(1);             //Prints time in Serial monitor
  LedsOff(); 
#ifdef NL 
  HET;     Laatzien();  IS;      Laatzien();  WAS;     Laatzien();
  MVIJF;   Laatzien();  MTIEN;   Laatzien();
  KWART;   Laatzien();  VOOR;    Laatzien();
  OVER;    Laatzien();  PRECIES; Laatzien(); 
  HALF;    Laatzien();  ELF;     Laatzien();  
  VIJF;    Laatzien();  TWEE;    Laatzien();  
  EEN;     Laatzien();  VIER;    Laatzien();
  TIEN;    Laatzien();  TWAALF;  Laatzien();
  DRIE;    Laatzien();  NEGEN;   Laatzien(); 
  ACHT;    Laatzien();  ZES;     Laatzien(); 
  ZEVEN;   Laatzien();  UUR;     Laatzien();
  EDSOFT;  Laatzien();
#endif //NL
#ifdef UK
  HET;     Laatzien();  IS;      Laatzien();  WAS;     Laatzien();
  PRECIES; Laatzien();  HALF;    Laatzien();
  TWINTIG; Laatzien();  MVIJF;   Laatzien();
  KWART;   Laatzien();  MTIEN;   Laatzien();
  OVER;    Laatzien();  VOOR;    Laatzien();
  ZES;     Laatzien();  TWEE;    Laatzien(); 
  VIJF;    Laatzien();  TWAALF;  Laatzien();
  TIEN;    Laatzien();  ELF;     Laatzien(); 
  VIER;    Laatzien();  NEGEN;   Laatzien(); 
  DRIE;    Laatzien();  ACHT;    Laatzien();
  ZEVEN;   Laatzien();  EEN;     Laatzien();
  UUR;     Laatzien();
  #endif //UK  

  for(int i=0; i<2; i++)
  {
   Display1=255;   Display2=255;   Display3=255;  Laatzien();
   Display1=0;     Display2=0;     Display3=0;    Laatzien();
  }  
  Play_Lights();     
  Displaytime();
}
// -------------------------- END Selftest   
//--------------------------- Time functions --------------------------

//--------------------------------------------
// CLOCK set the LED's for displaying
//--------------------------------------------
void Displaytime(void)
{
 LedsOff();                                  // start by clearing the display to a known state
 HET;                                        // HET light is always on
 switch (Iminute)
 {
#ifdef NL
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
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
#endif //NL

#ifdef UK
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
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
  case 21: IS;  TWINTIG; OVER;  break;
  case 22: 
  case 23: WAS; TWINTIG; OVER; break;
  case 24: 
  case 25: 
  case 26: IS;  TWINTIG; MVIJF; OVER; break;
  case 27: 
  case 28: WAS; TWINTIG; MVIJF; OVER; break;
  case 29: IS;  HALF; OVER; break;
  case 30: IS;  PRECIES; HALF; OVER; break;
  case 31: IS;  HALF; OVER; break;
  case 32: 
  case 33: WAS; HALF; OVER; break;
  case 34: 
  case 35: 
  case 36: IS;  TWINTIG; MVIJF; VOOR; break;
  case 37: 
  case 38: WAS; TWINTIG; MVIJF; VOOR; break;
  case 39: 
  case 40: 
  case 41: IS;  TWINTIG; VOOR;  break;
  case 42: 
  case 43: WAS; TWINTIG; VOOR;  break;
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
#endif //UK

}
// if (Ihour >=0 && Ihour <12) digitalWrite(DCF_LED_Pin,0); else digitalWrite(DCF_LED_Pin,1);
                                        #ifdef NL
 sayhour = Ihour;
 if (Iminute > 18 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif //NL
                                        #ifdef UK
 sayhour = Ihour;
 if (Iminute > 33 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif //UK
                                        #ifdef FR
 sayhour = Ihour;                                   // French saying still not perfect
 if (Iminute > 33 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif //FR

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
 switch (Iminute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }

 if(Iminute == 0 && Isecond <9) 
 { 
  ToggleEdsoft = Isecond % 2;         // ToggleEdsoft bocomes 0 or 1 and turn on and off the first 8 seconds at minute 0 the Edsoft light on pin 24
  EDSOFT;
 } 
// Tekstprintln("");
 WriteLEDs();
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTCklok.now();
 Ihour =   Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}

//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = min(Ihour  , 24);
 Iminute = min(Iminute, 59); 
 Isecond = min(Isecond, 59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                               // synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from DS3231 Time module
//--------------------------------------------
int Get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & 0b01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else {  temp3231 = -273; }   
  return (temp3231);
}

// ------------------- End  Time functions 

// --------------------Light functions -----------------------------------
//--------------------------------------------
//  LED load the shiftbits in the LED display buffer
//--------------------------------------------
void ShiftInTime(const char TimeText[10], byte num , byte Displaynr, byte Shiftbit, byte ShiftIn)
{ 
 Tekstprint(TimeText);                           // Print the time in text 
 num = num;                                      // Not used
 switch (Displaynr)
  {
 case 1:
        Display1 |= Shiftbit << ShiftIn;
        break;
 case 2:
        Display2 |= Shiftbit << ShiftIn;
        break;
 case 3:
        Display3 |= Shiftbit << ShiftIn;
        break; 
 }     
}

//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void){  Display1=0;  Display2=0;  Display3=0; }

//--------------------------------------------
//  LED Turn On the LED's
//  Write the actual values to the hardware 
//--------------------------------------------
void WriteLEDs(void) 
{                                                                                
 digitalWrite(LEDStrobePin,LOW);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display3);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display2);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display1);
 digitalWrite(LEDStrobePin,HIGH);
 delay(2);
}
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Laatzien()
{ 
 WriteLEDs();  
 delay(200);  
 LedsOff();
}

//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------                                                                                           
void DimLeds(byte print) 
{                                                                                                       
 int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;               // Read lightsensor 0 - 1024 bits 
 Previous_LDR_read = LDR_read;
 OutPhotocell = (int)((Mem.LightReducer * sqrt(63.5*LDR_read))/100);                  // 0-255 bits.  Linear --> hyperbolic with sqrt
 MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
 MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
 BrightnessCalcFromLDR = (byte)constrain(OutPhotocell, Mem.LowerBrightness, 255);// filter out of strange results 
 if(print)
  {
   sprintf(sptext,"Sensor:%3d",(analogRead(PhotoCellPin)));  Tekstprint(sptext);
   sprintf(sptext," Min:%3d",MinPhotocell);                  Tekstprint(sptext);
   sprintf(sptext," Max:%3d",MaxPhotocell);                  Tekstprint(sptext);
   sprintf(sptext," Out:%3d",OutPhotocell);                  Tekstprint(sptext);
   sprintf(sptext,"=%2d%%",(100*BrightnessCalcFromLDR/255)); Tekstprint(sptext);
   sprintf(sptext," Temp:%2dC ", Get3231Temp()-2);           Tekstprint(sptext);
   Print_tijd(); 
  }
 if(LEDsAreOff) BrightnessCalcFromLDR = 0; 
 analogWrite(PWMpin, BrightnessCalcFromLDR);  // write PWM
}

//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
 Display1=255;   Display2=255;   Display3=255; Laatzien();
 for (int n=255 ; n>=0; n--) { analogWrite(PWMpin, n); delay(2);}    // The duty cycle: between 0 (lights off) and 255 (light full on).
 for (int n=0 ; n<=255; n++) { analogWrite(PWMpin, n); delay(2);}  
 LedsOff();
}
//--------------------------------------------
//  Blink HOUR
//--------------------------------------------
void BlinkUUR(int NoofBlinks, int Delayms)
{
for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); UUR; Laatzien(); delay(Delayms);} 
}
//--------------------------------------------
//  Blink HET IS WAS
//--------------------------------------------
void BlinkHETISWAS (int NoofBlinks, int Delayms)
{
for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); HET; IS; WAS; Laatzien(); delay(Delayms);} 
}
//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(int amount)
{
 int value = Mem.LightReducer + amount;                              // Prevent byte overflow by making it an integer before adding
 Mem.LightReducer = min(value, 255);          // May not be larger than 255
 sprintf(sptext,"Max brightness: %3d%%",Mem.LightReducer);
 Tekstprintln(sptext);
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(byte waarde)
{
 Mem.LowerBrightness = min(waarde, 255);             // Range between 0 and 255
 sprintf(sptext,"Lower brightness: %3d bits", Mem.LowerBrightness);
 Tekstprintln(sptext);
}

// --------------------End Light functions 
//--------------------------------------------
//  CLOCK Constrain a string with integers
// The value between the first and last character in a string is returned between the  low and up bounderies
//--------------------------------------------
int SConstrainInt(String s,byte first,byte last,int low,int up){return constrain(s.substring(first, last).toInt(), low, up);}
int SConstrainInt(String s,byte first,          int low,int up){return constrain(s.substring(first).toInt(), low, up);}
//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 //Serial.println(InputString);
 InputString.trim();                                                 // Remove trailing spaces
 if (InputString.length()>10) return;                                // If string is too long for some reason
 if (InputString.length()< 1) return;                                // If string is empty for some reason
 if (InputString[0] > 64 && InputString[0] <123)                     // If the first charater is a letter
  {
  sprintf(sptext,"**** Length fault ****");                          // Default message placed in sptext
  switch (InputString[0]) 
   { 
    case 'A':
    case 'a':   
             Toggle_HetWasIsUit = 0; Toggle_HetWasIs = 1;             // All tekst displayed  
             sprintf(sptext,"All tekst displayed");
             break;
    case 'B':
    case 'b':    
             Toggle_HetWasIsUit = 1; Toggle_HetWasIs = 0;             // Het Is Was turned off
             sprintf(sptext,"Het Is Was turned off");
             break;
    case 'E':
    case 'e':    
            Toggle_HetWasIsUit = 2; Toggle_HetWasIs = 0;              // Het Is Was Off after 10 sec
            Play_Lights();                     
            sprintf(sptext,"Het Is Was Off after 10 sec");                          
            break;
    case 'C':
    case 'c':
             if(InputString.length() == 1)
               {
                Mem.LightReducer    = MAXBRIGHTNESS;
                Mem.LowerBrightness = LOWBRIGHTNESS;
                Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
                EEPROM.put(0,Mem);                                                    // Update EEPROM  
                sprintf(sptext,"\nData were cleared");    
               }
              if(InputString.length() == 3)
               {
                for (unsigned int i=0 ; i<EEPROM.length(); i++) { EEPROM.write(i, 0); }
                Tekstprintln("EEPROM data were erased"); 
                setup();
               }
               break;
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              int Jaar;
              Iday   = (byte) SConstrainInt(InputString,1,3,0,31);
              Imonth = (byte) SConstrainInt(InputString,3,5, 0, 12); 
              Jaar   =        SConstrainInt(InputString,5,9, 2000, 3000); 
              RTCklok.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
             }
            break;
    case 'I':
    case 'i': 
            if (InputString.length() == 1)
            {  
             SWversion();
            }
            break;
    case 'L':                                                         // Lowest value for Brightness
    case 'l':
             if (InputString.length() < 5)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                sprintf(sptext,"Lower brightness changed to: %d bits",Mem.LowerBrightness);
                EEPROM.put(0,Mem);                                                    // Update EEPROM     
               }
             break;  
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':
    Serial.println(InputString);
            if (InputString.length() < 5)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Max brightness changed to: %d%%",Mem.LightReducer);
                EEPROM.put(0,Mem);                                                    // Update EEPROM     
               }
              break;
    case 'N':
    case 'n':
             if (InputString.length() == 1 )         Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
             if (InputString.length() == 5 )
              {
               Mem.TurnOffLEDsAtHH = SConstrainInt(InputString,1,3,0,23);
               Mem.TurnOnLEDsAtHH  = SConstrainInt(InputString,3,5,0,23); 
               sprintf(sptext,"LEDs are OFF between %2d:00 and %2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
              }
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );
                DimLeds(true);                    // Turn the LEDs on or off                
               }
              break;                                                              
    case 'R':
    case 'r':
            if (InputString.length() == 1)
              {
               Reset();                                               // Reset all settings 
               Tekstprintln("Reset to default settings"); 
              }
            break;
    case 'S':
    case 's':
             if (InputString.length() == 1)
               {   
                Zelftest = 1 - Zelftest; 
                sprintf(sptext,"Zelftest: %d",Zelftest);
                Displaytime();                                          // Turn on the LEDs with proper time
               }                                
             break; 
    case 'T':
    case 't':
            if(InputString.length() >= 7)  // T125500
              {              
              Ihour   = (byte) SConstrainInt(InputString,1,3,0,23);
              Iminute = (byte) SConstrainInt(InputString,3,5,0,59); 
              Isecond = (byte) SConstrainInt(InputString,5,7,0,59); 
              sprintf(sptext,"Time set");
              SetRTCTime();
              }
              break;         
    case 'X':
    case 'x':    
             Demo = 1 - Demo;                                          // Toggle Demo mode
             sprintf(sptext,"Demo mode:%s", Demo?"ON":"OFF" );
             break;        
     default:
             break;
    }                                          
  }
 else if (InputString.length() == 6 )    // for compatibility
   {
    Ihour   = (byte) SConstrainInt(InputString,0,2,0,23);
    Iminute = (byte) SConstrainInt(InputString,2,4,0,59); 
    Isecond = (byte) SConstrainInt(InputString,4,6,0,59);
    sprintf(sptext,"Time set");  
    SetRTCTime();
   }
 Tekstprintln(sptext);
 Displaytime();
}

//********************************************************************************************
