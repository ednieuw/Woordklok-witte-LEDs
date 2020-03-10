// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 chip on the PCB board that controls the 24 LED strips of the Word Clock
Attached to this PCB are 
- RCT DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05
- DCF77 module DCF-2
- FM Stereo Radio Module RDA5807M RRD-102V2.0  
A 74HC595 ULN2803APG combination regulates the LEDs by shifting in bits into the 74HC595 LED are turn On or Off
A FT232RL 5.5V FTDI USB to TTL Serial Module can be attached to program te ATMEGA and read the serial port
The HC05 Bluetooth module is use to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module
The FM-module can be used to read the RDS-time from radio station or to play FM-radio station from within the clock

 Title ..: Woordklok
 Author .: Ed Nieuwenhuys
 Changes.: 0.27-->0.40 read clock every second instead of every hour"
 Changes.: 0.27-->0.40 Encoder turn writes time directly to clock"
 Changes.: 0.42 toggle HetWasIs"
 Changes.: 0.43 Added PWM on Pin5 to control dimming "
 Changes.: 0.48 Minor changes"
 Changes.: 0.49 rotary encoder improved"
 Changes.: 0.50 Stable and organised coding"
 Changes.: 0.51 (SQRT (long)analog read *  63.5) for less lineair dimming"
 Changes.: 0.52 Changed rotary pins"
 Changes.: 0.54 Coding voor Klok 3 en 5"
 Changes.: 0.60 Nieuwe display format. Hetiswas na 10 sec uit "
 Changes.: 0.61 Pinchanges"
 Changes.: 0.63 Ebben klok No13"
 Changes.: 0.64 Light reducer added"
 Changes.: 0.65 Programma aangepast voor standaard front en KY-040 rotary-besturing ingebouwd"
 Changes.: 0.66 Random tijd when no RTC signal"
 Changes.: 0.67 Source code cleaning"
 Changes.: 0.69 Bluetooth added"
 Changes.: 0.70 Temperature added"
 Changes.: 0.76 Rotary delay 200 ---> 0 ms"
 Changes.: 0.78 BT Lightintensity"
 Changes.: 0.79 FM-radio time and serial/bluetooth input commands added"
 Changes.: 0.81 DCF77 receiver included"
 Changes.: 0.84 #ifdef included statements in source file, optimisation of code"
 Changes.: 0.86 English display added"
 Changes.: 0.87 French display added ---> Failed text does not fit in 11 x 11 character matrix. 
 Changes.: 0.90 Dutch-English version"    myEnc.write(0); at end of rotary function
 */
// ===============================================================================================================================
//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------

//#define FMRADIOMOD  // in development time retrieval works. Needs automatic optimal sender search function
#define BLUETOOTHMOD
#define DCFMOD
#define ROTARYMOD
//--------------------------------------------
// ARDUINO Definition of installed language word clock
//--------------------------------------------
 #define NL
// #define UK
// #define DE
// #define FR        // in development
//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <TimeLib.h>
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>     // for Bluetooth communication
                     #endif BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>
                     #endif ROTARYMOD
                     #ifdef DCFMOD
#include "DCF77.h"
                     #endif DCFMOD


#ifdef NL
#define HET     Tekstprint("Het ");      Display1=Display1 | (Toggle_HetWasIs<<0)
#define IS      Tekstprint("is ");       Display1=Display1 | (Toggle_HetWasIs<<1)
#define WAS     Tekstprint("was ");      Display1=Display1 | (Toggle_HetWasIs<<2)
#define MVIJF   Tekstprint("vijf ");     Display1=Display1 | (1<<3)
#define MTIEN   Tekstprint("tien ");     Display1=Display1 | (1<<4) 
#define KWART   Tekstprint("kwart ");    Display1=Display1 | (1<<5)
#define VOOR    Tekstprint("voor ");     Display1=Display1 | (1<<6)
#define OVER    Tekstprint("over ");     Display1=Display1 | (1<<7)

#define PRECIES Tekstprint("precies ");  Display2=Display2 | (1<<0)
#define HALF    Tekstprint("half ");     Display2=Display2 | (1<<1)
#define ELF     Tekstprint("elf ");      Display2=Display2 | (1<<2)
#define VIJF    Tekstprint("vijf ");     Display2=Display2 | (1<<3)
#define TWEE    Tekstprint("twee ");     Display2=Display2 | (1<<4)
#define EEN     Tekstprint("een ");      Display2=Display2 | (1<<5)
#define VIER    Tekstprint("vier ");     Display2=Display2 | (1<<6)
#define TIEN    Tekstprint("tien ");     Display2=Display2 | (1<<7)

#define TWAALF  Tekstprint("twaalf ");   Display3=Display3 | (1<<0)
#define DRIE    Tekstprint("drie ");     Display3=Display3 | (1<<1)
#define NEGEN   Tekstprint("negen ");    Display3=Display3 | (1<<2)
#define ACHT    Tekstprint("acht ");     Display3=Display3 | (1<<3)
#define ZES     Tekstprint("zes ");      Display3=Display3 | (1<<4)
#define ZEVEN   Tekstprint("zeven ");    Display3=Display3 | (1<<5)
#define UUR     Tekstprint("uur ");      Display3=Display3 | (1<<6)
#define EDSOFT  Tekstprint("Edsoft ");   Display3=Display3 | (1<<7)
#endif NL

#ifdef UK
#define HET     Tekstprint("It ");       Display1=Display1 | (Toggle_HetWasIs<<0)
#define IS      Tekstprint("is ");       Display1=Display1 | (Toggle_HetWasIs<<1)
#define WAS     Tekstprint("was ");      Display1=Display1 | (Toggle_HetWasIs<<2)
#define PRECIES Tekstprint("exact ");    Display1=Display1 | (1<<3)
#define HALF    Tekstprint("half ");     Display1=Display1 | (1<<4) 
#define TWINTIG Tekstprint("twenty ");   Display1=Display1 | (1<<5)
#define MVIJF   Tekstprint("five ");     Display1=Display1 | (1<<6)
#define KWART   Tekstprint("quarter ");  Display1=Display1 | (1<<7)

#define MTIEN   Tekstprint("ten ");      Display2=Display2 | (1<<0)
#define OVER    Tekstprint("past ");     Display2=Display2 | (1<<1)
#define ZES     Tekstprint("six ");      Display2=Display2 | (1<<2)
#define TWEE    Tekstprint("two ");      Display2=Display2 | (1<<3)
#define VIJF    Tekstprint("five ");     Display2=Display2 | (1<<4)
#define TWAALF  Tekstprint("twelve ");   Display2=Display2 | (1<<5)
#define TIEN    Tekstprint("ten ");      Display2=Display2 | (1<<6)
#define ELF     Tekstprint("eleven ");   Display2=Display2 | (1<<7)

#define VIER   Tekstprint("four ");      Display3=Display3 | (1<<0)
#define NEGEN  Tekstprint("nine ");      Display3=Display3 | (1<<1)
#define DRIE   Tekstprint("three ");     Display3=Display3 | (1<<2)
#define ACHT   Tekstprint("eight ");     Display3=Display3 | (1<<3)
#define ZEVEN  Tekstprint("seven ");     Display3=Display3 | (1<<4)
#define EEN    Tekstprint("one ");       Display3=Display3 | (1<<5)
#define UUR    Tekstprint("O'clock ");   Display3=Display3 | (1<<6)
#define VOOR   Tekstprint("to ");    Display3=Display3 | (1<<7)
#endif UK
#ifdef FR
#define HET     Tekstprint("Il ");       Display1=Display1 | (Toggle_HetWasIs<<0)
#define IS      Tekstprint("est ");      Display1=Display1 | (Toggle_HetWasIs<<1)
#define WAS     Tekstprint("etait ");    Display1=Display1 | (Toggle_HetWasIs<<2)
#define ELF     Tekstprint("onze ");     Display1=Display1 | (1<<3)
#define VIJF    Tekstprint("cing ");     Display1=Display1 | (1<<4) 
#define TWEE    Tekstprint("deux ");     Display1=Display1 | (1<<5)
#define DRIE    Tekstprint("trois ");    Display1=Display1 | (1<<6)
#define ZES     Tekstprint("six ");      Display1=Display1 | (1<<7)

#define VIER    Tekstprint("quatre ");   Display2=Display2 | (1<<0)
#define NTWAALF Tekstprint("minuit ");   Display2=Display2 | (1<<1)
#define TIEN    Tekstprint("dix ");      Display2=Display2 | (1<<2)
#define NEGEN   Tekstprint("neuf ");     Display2=Display2 | (1<<3)
#define TWAALF  Tekstprint("midi ");     Display2=Display2 | (1<<4)
#define ACHT    Tekstprint("huit ");     Display2=Display2 | (1<<5)
#define ZEVEN   Tekstprint("sept ");     Display2=Display2 | (1<<6)
#define EEN     Tekstprint("une ");      Display2=Display2 | (1<<7)

#define UUR     Tekstprint("heures ");   Display3=Display3 | (1<<0)
#define OVER    Tekstprint("et ");       Display3=Display3 | (1<<1)
#define VOOR    Tekstprint("moins ");    Display3=Display3 | (1<<2)
#define EN      Tekstprint("le ");       Display3=Display3 | (1<<3)
#define KWART   Tekstprint("quart ");    Display3=Display3 | (1<<4)
#define MTIEN   Tekstprint("dix ");      Display3=Display3 | (1<<5)
#define TWINTIG Tekstprint("vingt ");    Display3=Display3 | (1<<6)
#define MVIJF   Tekstprint("cing ");     Display3=Display3 | (1<<7)
#define HALF    Tekstprint("demie ");    TIEN; TWINTIG;
#endif FR

//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 
                                  // Digital hardware constants ----
enum DigitalPinAssignments {

  DCF_PIN     =  2,               // DCFPulse on interrupt  pin
  encoderPinA  = 3,               // right (labeled DT on decoder)on interrupt  pin
  clearButton  = 4,               // switch (labeled SW on decoder)
  PWMpin       = 5,               // Pin that controle PWM signal on BC327 transistor to dim light
  BT_RX        = 6,               // Bluetooth RX
  BT_TX        = 7,               // Bluetooth TX
  encoderPinB  = 8,               // left (labeled CLK on decoder)no interrupt pin  
  DCF_LED_Pin  = 9,               // define pin voor AM PM Led
  LEDDataPin   = 10,              // blauw HC595
  LEDStrobePin = 11,              // groen HC595
  LEDClockPin  = 12,              // geel  HC595
  secondsPin   = 13};
                                  // Analogue hardware constants ----
enum AnaloguePinAssignments {
 PhotoCellPin  = 2,               // LDR pin
 EmptyA3       = 3,               //
 SDA_pin       = 4,               // SDA pin
 SCL_pin       = 5};              // SCL pin
//--------------------------------------------
// LED
//--------------------------------------------
byte BrightnessCalcFromLDR = 200; // BRIGHTNESS 0 - 255
//--------------------------------------------
// KY-040 ROTARY
//--------------------------------------------                                  
Encoder myEnc(encoderPinA, encoderPinB);              // Use digital pin  for encoder
long Looptime = 0;
unsigned long RotaryPressTimer = 0;
//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
float LightReducer = 0.80 ;       // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
byte LowerBrightness = 10;        // Lower limit of Brightness ( 0 - 255)
int OutPhotocell;                 // stores reading of photocell;
int MinPhotocell = 1024;          // stores minimum reading of photocell;
int MaxPhotocell = 1;             // stores maximum reading of photocell;
//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
static unsigned long msTick;   // the number of millisecond ticks since we last incremented the second counter
int  count; 
int  Delaytime = 200;
byte Display1 = 0, Display2 = 0, Display3 = 0;
byte Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte lastminute = 0, lasthour = 0, sayhour = 0;
byte Toggle_HetWasIs    = 1;      // Turn On/Off HetIsWas lights
byte Toggle_HetWasIsUit = 0;      // Turn off HetIsWas after 10 sec
byte SecPulse           = 0;      // give a pulse to the Isecond led
byte Demo = false;
String SerialString;
//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
RTC_DS3231 RTC;    //RTC_DS1307 RTC;  
DateTime Inow;
//--------------------------------------------
// BLUETOOTH
//--------------------------------------------                                     
#ifdef BLUETOOTHMOD               // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);    // RX, TX
String BluetoothString;
#endif BLUETOOTHMOD                                
//--------------------------------------------
// RDA5807 FM-RADIO
//-------------------------------------------- 
#ifdef FMRADIOMOD                 // FM radio -----------------------
byte  RadioUur;                   // reading from  RDS FM-radio 
byte  RadioMinuut;                // reading from  RDS FM-radio                          
float fini = 103.50; //91.60; // 103.50; //98.10;               // Start frequency
int   ftun;                       // Selected frequency 
float Freq_lower_bandwith = 87.00;// lower Band limit 
float Freq_tuned;                 //
int   RDA5807_adrs = 0x10;        // I2C-Address RDA Chip for sequential  Access
int   RDA5807_adrr = 0x11;        // I2C-Address RDA Chip for random      Access
int   RDA5807_adrt = 0x60;        // I2C-Address RDA Chip for TEA5767like Access
int   sidx = 0;                   // Counter of frequency array
int   vol  = 0;                   // Volume
int   rssi = 0;                   // Signal-Level
unsigned int auRDS[32];
unsigned int auRDA5807_Reg[32];
unsigned int aui_RDA5807_Reg[32];
unsigned int aui_buf[8];
unsigned int auRDA5807_Regdef[10] ={
                                    0x0758,  // 00 defaultid
                                    0x0000,  // 01 not used
                                    0xD009,  // 02 DHIZ,DMUTE,BASS, POWERUPENABLE,RDS
                                    0x0000,  // 03
                                    0x1400,  // 04 DE ? SOFTMUTE  
                                    0x84D0,  // 05 INT_MODE, SEEKTH=0110,????, Volume=0
                                    0x4000,  // 06 OPENMODE=01
                                    0x0000,  // 07 unused ?
                                    0x0000,  // 08 unused ?
                                    0x0000   // 09 unused ?
                                  };
#endif FMRADIOMOD                // END FM radio ------------------------
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
#ifdef DCFMOD                     // DCF77 ------------------------------
#define DCF_INTERRUPT 0           // DCF Interrupt number associated with DCF_PIN
time_t time;
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW);
#endif DCFMOD 
                                  // End Definitions  ---------------------------------------------------------
                               
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
SerialCheck();

if(Demo)  Demomode();
else
 { 
                          #ifdef FMRADIOMOD  
  FMradioCheck();  
                          #endif FMRADIOMOD
  EverySecondCheck();
                          #ifdef DCFMOD
  DCF77Check();

                          #endif DCFMOD
                          #ifdef BLUETOOTHMOD 
  BluetoothCheck(); 
                          #endif BLUETOOTHMOD

  EveryMinuteUpdate();
                          #ifdef ROTARYMOD
  RotaryEncoderCheck(); 
                          #endif ROTARYMOD
 }
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                              // initialise the hardware // initialize the appropriate pins as outputs:
 pinMode(LEDClockPin,  OUTPUT); 
 pinMode(LEDDataPin,   OUTPUT); 
 pinMode(LEDStrobePin, OUTPUT); 
 pinMode(PWMpin,       OUTPUT);
 pinMode(DCF_LED_Pin,  OUTPUT);
 pinMode(secondsPin,   OUTPUT );
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP);
 pinMode(DCF_PIN,      INPUT_PULLUP);
 Serial.begin(9600);                          // setup the serial port to 9600 baud 
                          #ifdef BLUETOOTHMOD 
 Bluetooth.begin(9600);                       // setup the Bluetooth port to 9600 baud 
                          #endif BLUETOOTHMOD
                          #ifdef FMRADIOMOD 
 Setup_FMradio();                             // start the FM-radio
                          #endif FMRADIOMOD 
                          #ifdef DCFMOD
 DCF.Start();                                 // start the DCF-module
                          #endif DCFMOD
 Wire.begin();                                // start the wire communication I2C
 RTC.begin();                                 // start the RTC-module
 analogWrite(PWMpin, BrightnessCalcFromLDR);  // the duty cycle: between 0 (lights off) and 255 (light full on).
 DateTime now = RTC.now();                    // Get the time from the RTC
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
 {
  Serial.println("RTC is older than compile time! Updating");      // following line sets the RTC to the date & time this sketch was compiled
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__))); 
 } 
  if (EEPROM.read(0) <10 || EEPROM.read(0) > 100) 
   EEPROM.write(0,(int)(LightReducer * 100));                      // default intensity for this clock
  if (EEPROM.read(1) <10 || EEPROM.read(1) > 100) 
   EEPROM.write(1, LowerBrightness);                               // default Lower Brightness for this clock
 LightReducer = ((float) EEPROM.read(0) / 100);                    // store it is the work variable
 LowerBrightness = EEPROM.read(1);                                 // store it is the work variable
 Looptime = millis();                                              // Used in KY-040 rotary
 msTick = millis(); 
 SWversion();                                                      // Display the version number of the software
 Selftest();                                                       // Play the selftest
 GetTijd(0);                                                       // Get the time and print it to serial
}
// --------------------------- END SETUP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<90;n++) {Serial.print(F("="));} Serial.println();
 Serial.println(F("Title ..........: Woordklok zwartwit met Rotary KY-040, Temp, Bluetooth, FM, DCF77"));
                 #ifdef BLUETOOTHMOD 
 Bluetooth.println(F("Woordklok 2017 "));
 Bluetooth.println(F("Enter time as:hhmm (1321) or hhmmss (132145)"));
 Bluetooth.println(F("Enter A for normal display"));
 Bluetooth.println(F("Enter B to suspress Het Is Was in display"));
 Bluetooth.println(F("Enter C to suspress Het Is Was after 10 seconds every minute change in display"));
 Bluetooth.println(F("Enter D for Demo mode"));
 Bluetooth.println(F("Enter Mnn (M90) to reduce light intensity to 90% range 5% - 100%"));
 Bluetooth.println(F("Enter Lnn (L5) to set lowest intensity during darkness "));
 Bluetooth.println(F("Enter I for info"));
                 #endif BLUETOOTHMOD
 Serial.println(F("Author    ......: Ed Nieuwenhuys"));
 Serial.println(F("Version.........: 0.90 Juli-2016"));
 Serial.print(F(  "LightReducer....: ")); Serial.print(LightReducer*100); Serial.println(F("%"));
 Serial.print(F(  "LowerBrightness.: ")); Serial.println(LowerBrightness);
  for (int n=0; n<90;n++) {Serial.print(F("="));} Serial.println();
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
    if(     Ihour >24)  Ihour = 0;
   DimLeds(false);
   Displaytime();
  }
}

//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char tekst[])
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
 Bluetooth.print(tekst);  
                          #endif BLUETOOTHMOD
}
void Tekstprintln(char tekst[])
{
 Serial.println(tekst);    
                          #ifdef BLUETOOTHMOD
 Bluetooth.println(tekst);
                          #endif BLUETOOTHMOD
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
  if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                               // second++; 
   digitalWrite(secondsPin,HIGH);                                   // turn ON the second on pin 13
   ++SecPulse;                                                      // second routine in function DimLeds
   GetTijd(0);                                                      // synchronize time with RTC clock
   if ((Toggle_HetWasIsUit == 2) && (Isecond > 10)) Toggle_HetWasIs = 0; 
    else Toggle_HetWasIs = 1;                                       // HET IS WAS is On
   if(Isecond % 30 == 0) DimLeds(true);                             //Led Intensity Control + seconds tick print every 30 seconds   
    else                 DimLeds(false);
  if ((Toggle_HetWasIsUit == 2) && (Isecond == 11))Displaytime();   // turn Leds OFF on second == 11
  }
 }
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                                         //show time every minute
  { 
   lastminute = Iminute;
   Displaytime();
   Print_RTC_tijd();
                            #ifdef BLUETOOTHMOD  
   Print_BTtijd();                                                  // print time to Bluetooth
                            #endif BLUETOOTHMOD
  } 
 if (Ihour != lasthour) {lasthour = Ihour;}
 }
                            #ifdef BLUETOOTHMOD
//--------------------------------------------
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 while (Bluetooth.available())
  {
   delay(3);  
   char c = Bluetooth.read();
   if (c>31 && c<128) BluetoothString += c;    
  }
 if (BluetoothString.length()>0)  ReworkInputString(BluetoothString); // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD
                           
                           #ifdef DCFMOD
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
  time_t DCFtime = DCF.getTime();                                   // Check if new DCF77 time is available
  if (DCFtime!=0)
   {
    Serial.print(F("Time is updated ----->  "));
    setTime(DCFtime); 
    RTC.adjust(DCFtime);
    digitalClockDisplay(); 
   } 
  digitalWrite(DCF_LED_Pin,1-digitalRead(DCF_PIN));                // write inverted DCF pulse to LED on board
} 
                           #endif DCFMOD 
//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<128) SerialString += c;                            // allow input from Space - Del
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
}

//------------------------ KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
                           #ifdef ROTARYMOD
void RotaryEncoderCheck(void)
{
// If button pressed, 60 sec after start ATMEGA, then there are 60 seconds to adjust the light intensity. 
// RotaryPressTimer is the time in millisec after start ATMEGA 
 long encoderPos = myEnc.read();
 if ( (encoderPos) && ( (millis() - Looptime) >200))                  // if rotary turned debounce 0.2 sec
  {   
   Serial.print(F("--------> Index:"));   Serial.println(encoderPos);
   if  (encoderPos >0)                                                // increase the MINUTES
    {
     if ( millis() > 60000 && (millis() - RotaryPressTimer) < 60000)
         { WriteLightReducer(0.05); }                                 // If time < 60 sec then adjust light intensity factor
     else 
     {
      if( ++Iminute >59) { Iminute = 0; Isecond = 0;  }
      SetRTCTime();  
     }     
     myEnc.write(0);
     Looptime = millis();                                             // Set encoder pos back to 0
    }
   if  (encoderPos <0)                                                // increase the HOURS
    {
     if (millis() > 60000 &&  (millis() - RotaryPressTimer) < 60000) 
         { WriteLightReducer(-0.05); }    // If time < 60 sec then adjust light intensity factor
     else
      { 
      if( ++Ihour >23) { Ihour = 0; }
      SetRTCTime();   
      }
     myEnc.write(0);
     Looptime = millis();                                             // Set encoder pos back to 0      
    }                                                
  }
 if (digitalRead(clearButton) == LOW )                                // set the time by pressing rotary button
  { 
    delay(200);
    RotaryPressTimer =  millis();                                     // If time < 60 sec then adjust light intensity factor
    Toggle_HetWasIsUit++;
    if (Toggle_HetWasIsUit >= 3)  { Toggle_HetWasIsUit = 0 ; }
    if (Toggle_HetWasIsUit == 0)  { Toggle_HetWasIs = 1;} // On
    if (Toggle_HetWasIsUit == 1)  { Toggle_HetWasIs = 0;} // Off
    if (Toggle_HetWasIsUit == 2)  { Toggle_HetWasIs = 0; Play_Lights(); } // Off after 10 sec
    Serial.print(F("Toggle_HetWasIsUit: "));   Serial.println(Toggle_HetWasIsUit);
    Serial.print(F("Toggle_HetWasIs: "));      Serial.println(Toggle_HetWasIs);    
    Displaytime();
    myEnc.write(0);
    Looptime = millis(); 
   }
   myEnc.write(0);
 }
                           #endif ROTARYMOD

//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{
  GetTijd(1);             //Prints time in Serial monitor
  LedsOff(); 
#ifdef NL 
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
#endif NL
#ifdef UK
  HET;     Laatzien();
  IS;      Laatzien();
  WAS;     Laatzien();
  PRECIES; Laatzien();
  HALF;    Laatzien();
  TWINTIG; Laatzien();
  MVIJF;   Laatzien();
  KWART;   Laatzien();
  MTIEN;   Laatzien();
  OVER;    Laatzien();  
  VOOR;    Laatzien();
  ZES;     Laatzien();   
  TWEE;    Laatzien(); 
  VIJF;    Laatzien();
  TWAALF;  Laatzien();
  TIEN;    Laatzien();
  ELF;     Laatzien(); 
  VIER;    Laatzien();
  NEGEN;   Laatzien(); 
  DRIE;    Laatzien();
  ACHT;    Laatzien();
  ZEVEN;   Laatzien();
  EEN;     Laatzien();
  UUR;     Laatzien();
#endif UK  
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
#endif NL

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
#endif UK
 
}
// if (Ihour >=0 && Ihour <12) digitalWrite(DCF_LED_Pin,0); else digitalWrite(DCF_LED_Pin,1);
                                        #ifdef NL
 sayhour = Ihour;
 if (Iminute > 18 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif NL
                                        #ifdef UK
 sayhour = Ihour;
 if (Iminute > 33 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif UK
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
 Tekstprintln("");
 WriteLEDs();
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTC.now();
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
 Inow = RTC.now();
 if (Inow.hour() < 10) Serial.print(F("0"));
 Serial.print(Inow.hour(), DEC);
 Serial.print(F(":"));
 if (Inow.minute() < 10) Serial.print(F("0"));
 Serial.print(Inow.minute(), DEC);
 Serial.print(F(":"));
 if (Inow.second() < 10) Serial.print(F("0"));
 Serial.print(Inow.second(), DEC);
 Serial.print(F("  "));
 Serial.print(Inow.day(), DEC);
 Serial.print(F("/"));
 Serial.print(Inow.month(), DEC);
 Serial.print(F("/"));
 Serial.println(Inow.year(), DEC); 
}

//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 if (Ihour < 10) Serial.print(F("0"));
 Serial.print(Ihour);
 Serial.print(F(":"));
 if (Iminute < 10) Serial.print(F("0"));
 Serial.print(Iminute);
 Serial.print(F(":"));
 if (Isecond < 10) Serial.print(F("0"));
 Serial.println(Isecond);
}

//--------------------------------------------
// BLUETOOTH utility function prints time to BT
//--------------------------------------------
                         #ifdef BLUETOOTHMOD
void Print_BTtijd(void)
{
 Bluetooth.print(F("T: "));
 if (Ihour < 10) Bluetooth.print(F("0"));
 Bluetooth.print(Ihour);
 Bluetooth.print(F(":"));
 if (Iminute < 10) Bluetooth.print(F("0"));
 Bluetooth.print(Iminute);
 Bluetooth.print(F(":"));
 if (Isecond < 10) Bluetooth.print(F("0"));
 Bluetooth.print(Isecond);
 Bluetooth.print(F(" I:"));         
 Bluetooth.print(( int) (BrightnessCalcFromLDR/2.55)); 
 Bluetooth.print(F("% LDR:"));
 Bluetooth.print(analogRead(PhotoCellPin)/10); 
 Bluetooth.print(F("% T:")); 
 Bluetooth.print(get3231Temp());
 Bluetooth.println(F("C")); 
}
                         #endif BLUETOOTHMOD
                         #ifdef DCFMOD
//--------------------------------------------
// DCF77 utility function prints DCF time
//--------------------------------------------
void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(F(" "));
  Serial.print(day());
  Serial.print(F(" "));
  Serial.print(month());
  Serial.print(F(" "));
  Serial.print(year()); 
  Serial.println(); 
}
//--------------------------------------------
// DCF77 utility function prints preceding colon and leading 0
//--------------------------------------------
void printDigits(int digits)
{
  Serial.print(F(":"));
  if(digits < 10)
    Serial.print(F("0"));
  Serial.print(digits);
}
                         #endif DCFMOD
//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 RTC.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                               // synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          //2's complement int portion
    tLSB = Wire.read();                          //fraction portion 
    temp3231 = (tMSB & B01111111);               //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    //only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else {  temp3231 = -273; }   
  return (temp3231);
}

// ------------------- End  Time functions 

// --------------------Light functions -----------------------------------
//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void){  Display1=0;  Display2=0;  Display3=0; }

//--------------------------------------------
//  LED Turn On the LED's
//--------------------------------------------
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
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Laatzien()
{ 
 WriteLEDs();  
 delay(Delaytime);  
 LedsOff();
}
//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{                                                                                                       
 if (SecPulse) 
 {
  int LDR_read = analogRead(PhotoCellPin);      // Read lightsensor
  OutPhotocell = (int) (LightReducer * sqrt( (float) 63.5 * constrain(LDR_read,10,1023))); // filter out of strange results and set minimum brightness
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
  if(print)
  {
   Serial.print(F("SensorRead: "));       Serial.print(LDR_read);
   Serial.print(F("  LightReducer: "));   Serial.print((int) (LightReducer * 100)); 
   Serial.print(F("%  Min: "));           Serial.print(MinPhotocell);  
   Serial.print(F("  Max: "));            Serial.print(MaxPhotocell);      
   Serial.print(F("  Now: "));            Serial.print(OutPhotocell);  
  }
  BrightnessCalcFromLDR = constrain(OutPhotocell, LowerBrightness , 255);                   // filter out of strange results
  if(print)
  {
   Serial.print(F(" LI Now: "));         Serial.print(( int) (BrightnessCalcFromLDR/2.55)); 
   Serial.print(F("%"));    
   Serial.print(F(" Temp: ")); Serial.print(get3231Temp());   Serial.print(F("C "));  
   Print_tijd();
  }
  analogWrite(PWMpin, BrightnessCalcFromLDR);  // write PWM 
 }
 SecPulse = 0;
}
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
 Display1=255;   Display2=255;   Display3=255; Laatzien();
 for (int n=255 ; n>=0; n--) { analogWrite(PWMpin, n); delay(2);}    // the duty cycle: between 0 (lights off) and 255 (light full on).
 for (int n=0 ; n<=255; n++) { analogWrite(PWMpin, n); delay(2);}  
 LedsOff();
}

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(float amount)
{
 LightReducer += amount; 
 WriteLightReducerEeprom(LightReducer);
}
//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void WriteLightReducerEeprom(float value)
{
 LightReducer = value;
 if (LightReducer < 0.00 ) LightReducer = 0.05;
 if (LightReducer > 2.50   ) LightReducer = 2.50;                    // May not be larger than 2.55 (value*100 = stored as byte 
 EEPROM.write(0, (int) (LightReducer * 100));                        // Store the value (0-250) in permanent EEPROM memory at address 0
// Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
 Serial.print(F("LightReducer: ")); Serial.print(LightReducer * 100); Serial.println("%");
}
// --------------------End Light functions 

//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 float ff;
 Serial.println(InputString);
  if ( InputString[0] > 64 )
 {
  int val = InputString[0];
  int FMfreq;
  switch (val)
   {
    case 'A':
    case 'a':   
             Toggle_HetWasIsUit = 0; Toggle_HetWasIs = 1;             // All tekst displayed  
             break;
    case 'B':
    case 'b':    
             Toggle_HetWasIsUit = 1; Toggle_HetWasIs = 0;             // Het Is Was turned off
             Serial.println(F("Het Is Was turned off"));
             break;
    case 'C':
    case 'c':    
            Toggle_HetWasIsUit = 2; Toggle_HetWasIs = 0;              // Het Is Was Off after 10 sec
            Play_Lights();                                               
            break;
    case 'D':
    case 'd':    
            Demo = 1 - Demo;                                          // toggle Demo mode
            Play_Lights();
            GetTijd(0);  
            Displaytime();
            break;
                     #ifdef FMRADIO                        
    case 'F':
    case 'f':
            //set FM frequency
             temp = InputString.substring(1);
             FMfreq = temp.toInt();
             if (FMfreq < 8750 ) FMfreq = 8750;
             if (FMfreq > 10800) FMfreq = 10800;   
             RDA5807_setFreq((float) FMfreq/100);           
             break;
                     #endif FMRADIO
    case 'L':                                                         // Lowest value for Brightness
    case 'l':    
             temp = InputString.substring(1);
             LowerBrightness = temp.toInt();
             if (LowerBrightness < 1 )  LowerBrightness = 1;
             if (LowerBrightness > 250) LowerBrightness = 250;
             EEPROM.write(1, LowerBrightness);                        // default Lower Brightness for this clock
             break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':    
             temp = InputString.substring(1);
             ff = (float)(temp.toInt()) / 100;
             WriteLightReducerEeprom(ff);
             break;
    case 'I':
    case 'i':   
            SWversion();
            Display1=255;   Display2=255;   Display3=255;  Laatzien();
            Display1=0;     Display2=0;     Display3=0;    Laatzien();
            break;
                     #ifdef FMRADIO
    case 'R':
    case 'r':
            RDA5807_Report();
            break;
    case 'S':
    case 's':
            RDA5807_ReadStatus();            
            break;
    case 'T':
    case 't':    
            RDA5807_RDS_Dump();                                          
            break;             
                     #endif FMRADIO       
    default:
            break;
   }
   Displaytime();
                     #ifdef BLUETOOTHMOD
   Bluetooth.print(F(  "LightReducer   : ")); Bluetooth.print(LightReducer * 100); Bluetooth.println("%");
   Bluetooth.print(F(  "LowerBrightness: ")); Bluetooth.println(LowerBrightness);
                     #endif BLUETOOTHMOD
   InputString = "";
 }
 else if (InputString.length() > 3 && InputString.length() <7 )
 {
  temp = InputString.substring(0,2);   
  Ihour = temp.toInt(); 
  if (InputString.length() > 3) { temp = InputString.substring(2,4); Iminute = temp.toInt(); }
  if (InputString.length() > 5) { temp = InputString.substring(4,6); Isecond = temp.toInt(); }
  SetRTCTime();
 }
 InputString = "";
 temp = "";
}


                       #ifdef FMRADIO
//----------------------------------------- FM radio -------------------------------------------- 
//--------------------------------------------
// RDA5807 Check if RDS data are available
//--------------------------------------------
void FMradioCheck(void)                               
{
 int uur, minuut, ofs;
 if (!Serial.available())
  {
   RDA5807_ReadW(4);                              // Read RDS-Data as 4 Word to Array
   if ((auRDS[1] & 0xF000) == 0x4000)
    {
      uur    = (16 * (auRDS[2] & 0x0001) + ((auRDS[3] & 0xF000)>>12));
      minuut = (auRDS[3] & 0x0FC0)>>6;
      ofs    = (auRDS[3] & 0x003F);
      uur   += (ofs / 2);
 //Serial.print(F("<"));     
    }
  if (uur<24 && RadioUur != uur && RadioMinuut != minuut) // to avoid a 100 ms delay. Otherwise same time is retrieved many times
    { 
     if (uur < 10)       { Serial.print(F(" ")); } 
     Serial.print(uur);    Serial.print(F("u:"));
     if (minuut < 10)    { Serial.print(F("0")); } 
     Serial.print(minuut); Serial.println(F("m"));
     RadioUur = uur;
     RadioMinuut = minuut;
//     delay(80); 
    }
   }
}

//--------------------------------------------
// RDA5807 Setup_FMradio
//--------------------------------------------
void  Setup_FMradio(void)
 {
  RDA5807_PowerOn();
  RDA5807_Reset();
  RDA5807_setFreq(fini);
 }  

//--------------------------------------------
// RDA5807 Reset Chip to Default Configuration
//--------------------------------------------
int RDA5807_Reset()
{
  Serial.println(F("RESET RDA5807"));
  for(int i = 0;i < 7; i++) {auRDA5807_Reg[i] = auRDA5807_Regdef[i];}
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x0002;   // Enable SoftReset
  int ret = RDA5807_Write();
  auRDA5807_Reg[2] = auRDA5807_Reg[2] & 0xFFFB;   // Disable SoftReset
  return ret;
}

//----------------------------------------
// RDA5807 Power Off
//----------------------------------------
int RDA5807_PowerOff()
{
  RDA5807_setVol(0);
  Serial.println("Power OFF RDA5807");
  aui_RDA5807_Reg[2]=0x0001;   // all bits off
  return RDA5807_Write();
  auRDA5807_Reg[2] =auRDA5807_Regdef[2];       // Reset to Default Value
}

//----------------------------------------
// RDA5807 Power On
//----------------------------------------
int RDA5807_PowerOn()
{
  Serial.println(F("Power ON RDA5807"));
  auRDA5807_Reg[3] = auRDA5807_Reg[3] | 0x010;   // Enable Tuning
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x001;   // Enable PowerOn
  int ret = RDA5807_Write();
  auRDA5807_Reg[3] = auRDA5807_Reg[3] & 0xFFEF;  // Disable Tuning
  return ret;
}

//----------------------------------------
// RDA5807 Seek up  to next Station
//----------------------------------------
int RDA5807_SeekUp()
{
  Serial.println(F("SeekUp"));
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x0300;   // Enable Seekup
  RDA5807_Write();
  auRDA5807_Reg[2] = auRDA5807_Reg[2] & 0xFCFF;   // Disable Seekup
  return 0;
}

//----------------------------------------
// RDA5807 Seek down  to next Station
//----------------------------------------
int RDA5807_SeekDown()
{

  Serial.println(F("SeekDown"));
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x0100;   // Enable SeekDown(default)
  RDA5807_Write();
  auRDA5807_Reg[2] = auRDA5807_Reg[2] & 0xFCFF;   // Disable Seek 
  return 0;
}

//----------------------------------------
// RDA5807 Tune Radio to defined Frequency
//----------------------------------------
int RDA5807_setFreq(float mhz)
{
  ftun = mhz * 100.0; 
  Freq_tuned = mhz;
  int Chnumber = (int)(( 0.01 + mhz - Freq_lower_bandwith ) / 0.1);
  Serial.print(F("Frequency: "));
  Serial.print(ftun);
  Serial.print(F(" Channel: "));
  Serial.println(Chnumber);
  Chnumber = Chnumber & 0x03FF;
  auRDA5807_Reg[3] = Chnumber * 64 + 0x10;     // Channel + TUNE-Bit + Band=00(87-108) + Space=00(100kHz)
  Wire.beginTransmission(RDA5807_adrs);
  Wire_write16(0xD009);
  Wire_write16(auRDA5807_Reg[3]);
  Wire.endTransmission(); 
  return 0;
}

//----------------------------------------
// RDA5807 Set Volume
//----------------------------------------
int RDA5807_setVol(int setvol)
{
  vol = setvol;
  if (vol > 15) {vol = 15; Serial.println(F("Vol already maximal")); return 1; }
  if (vol < 0)  {vol = 0;  Serial.println(F("Vol already minimal")); return 1; }
  Serial.print(F("Volume="));     Serial.println(vol);
  auRDA5807_Reg[5] = (auRDA5807_Reg[5] & 0xFFF0)| vol;   // Set New Volume
  RDA5807_WriteReg(5);
  return 0;
}

//----------------------------------------
// Write 16Bit To I2C / Two Wire Interface
//----------------------------------------
void Wire_write16(unsigned int val)
{
 // if (b_debug) { Serial_print16h(val);}
  Wire.write(val >> 8); Wire.write(val & 0xFF);
}

//------------------------------------------
// Serial Print 16Bit Number in HEX as hh:ll
//------------------------------------------
void Serial_print16h(unsigned int uval)
{
  byte b_high,b_low;
  b_high = uval >> 8; b_low = uval & 0xFF;
  if (b_high < 0x10){ Serial.write('0');} Serial.print(b_high,HEX); Serial.write(':');
  if (b_low  < 0x10){ Serial.write('0');} Serial.print(b_low ,HEX); 
}

//----------------------------------------
// RDA5807 Set all Configuration Registers
//----------------------------------------
int RDA5807_Write()
{
  Wire.beginTransmission(RDA5807_adrs);
  for ( int i = 2; i < 7; i++) { Wire_write16(auRDA5807_Reg[i]);}
  return Wire.endTransmission();
}
//----------------------------------------
// RDA5807 Set one Configuration Registers
//----------------------------------------
int RDA5807_WriteReg(int reg)
{
  Wire.beginTransmission(RDA5807_adrr);
  Wire.write(reg); 
  Wire_write16(auRDA5807_Reg[reg]);
  return Wire.endTransmission();
}

//---------------------------------------------
// RDA5807 Read Special Data Registers as Word
//---------------------------------------------
void RDA5807_ReadW(int cnt)
{
   Wire.beginTransmission(RDA5807_adrr);            // Device 0x11 for random access
   Wire.write(0x0C);                                // Start at Register 0x0C
   Wire.endTransmission(0);                         // restart condition
   Wire.requestFrom(RDA5807_adrr,2*cnt, 1);         // Retransmit device address with READ, followed by 8 bytes
   for (int i = 0; i < cnt; i++)                    // Loop for Read data    
   {auRDS[i] = 256 * Wire.read() + Wire.read();}    // Read Data into Array of Unsigned Ints
   Wire.endTransmission();                  
} 

//----------------------------------------
// RDA5807 Read and Show all Status Registers
//----------------------------------------
int RDA5807_ReadStatus()
{
  int Chnumber = -1;
  unsigned int aubuf[8];
  memset (aubuf, 0, 8);
  Serial.println(F("Info Status RDA5807:"));
  Serial.println(F("Reg | 0Ah | 0Bh | 0Ch | 0Dh | 0Eh | 0Fh |"));
  Serial.print(F("    |"));
  Wire.requestFrom(RDA5807_adrs, 12); 
  for (int i = 0; i < 6; i++)  { aubuf[i] = 256 * Wire.read () + Wire.read(); }
  Wire.endTransmission();
  for (int i = 0; i < 6; i++)  { Serial_print16h(aubuf[i]); Serial.print("|"); }
  Serial.println();
  Chnumber = (aubuf[0] & 0x03FF); 
  Freq_tuned = Freq_lower_bandwith + Chnumber * 0.10;
  rssi = aubuf[1] >> 10;
  Serial.print(F("RDS Data:    ")); if ((aubuf[0] & 0x8000)==0){ Serial.println(F("NO"));}           else {Serial.println(F("NEW data"));}
  Serial.print(F("SEEK Ready:  ")); if ((aubuf[0] & 0x4000)==0){ Serial.println(F("no"));}           else {Serial.println(F("OK"));}
  Serial.print(F("SEEK Fail:   ")); if ((aubuf[0] & 0x2000)==0){ Serial.println(F("no, Succces!"));} else {Serial.println(F("FAILED"));}
  Serial.print(F("RDS Sync:    ")); if ((aubuf[0] & 0x1000)==0){ Serial.println(F("no"));}           else {Serial.println(F("OK"));}
  Serial.print(F("RDS Block:   ")); if ((aubuf[0] & 0x0800)==0){ Serial.println(F("no"));}           else {Serial.println(F("Block E"));}
  Serial.print(F("Stationmode: ")); if ((aubuf[0] & 0x0400)==0){ Serial.println(F("Mono  "));}       else {Serial.println(F("Stereo"));} 
  Serial.print(F("Channel Nr:  ")); Serial.print(Chnumber); Serial.print(F(" = "));
  Serial.print(Freq_tuned);         Serial.println(F(" MHz"));
  Serial.print(F("SignalLevel: ")); Serial.println(rssi);
  return 0;
}

//----------------------------------------
// RDA5807 Report all available Stations
//----------------------------------------
int RDA5807_Report()
{
  Freq_tuned = Freq_lower_bandwith;
  int cnt_stations = 0;
  int cnt_stereo = 0;
  int cnt_rds = 0;
  int Rssi = 0;
//auRDA5807_Reg[3] =  0x10;  //Set channelnumber 0
//RDA5807_setFreq(87.50);
  Serial.println(F("Sender Report:"));
   for(int Chnumber = 0; Chnumber <= 210; Chnumber++)
  {
    auRDA5807_Reg[3] = 64 * Chnumber + 0x10; 
    Wire.beginTransmission(RDA5807_adrs);
    Wire_write16(0xD009);
    Wire_write16(auRDA5807_Reg[3]);
    Wire.endTransmission();
    delay(300);                           //give de radio some time to settle
    RDA5807_Status();
  }
}

//----------------------------------------
// RDA5807 Show Status
//----------------------------------------
void RDA5807_Status(void)
{
  int Chnumber;
  Wire.requestFrom (RDA5807_adrs, 16); 
  for (int i = 0; i < 8; i++) { auRDA5807_Reg[0x0A + i] = 256 * Wire.read () + Wire.read(); }
  Wire.endTransmission();
  Chnumber = auRDA5807_Reg[0x0A] & 0x03FF;
  rssi = auRDA5807_Reg[0x0B] >> 10;
  Freq_tuned = Freq_lower_bandwith + (Chnumber ) * 0.1;
//  if ( (auRDA5807_Reg[0x0A] & 0x8000) && (auRDA5807_Reg[0x0A] & 0x0400)        )  // if RDS and stereo in station
 if ((auRDA5807_Reg[0x0A] & 0x0400) )                    // if Stereo in station
  {
   if (Freq_tuned <= 99.99){Serial.print(" ");}
   Serial.print(Freq_tuned);
   Serial.print(F(" MHz"));
   Serial.print(F(" Ch=")); if (Chnumber < 10){Serial.print(F(" "));} if (Chnumber < 100) { Serial.print(F(" ")); } Serial.print(Chnumber);
   Serial.print(F(" PI=")); Serial_printuih(auRDA5807_Reg[0x0C]);             // RDS Block A contains Station ID
   if ((auRDA5807_Reg[0x0A] & 0x0400) == 0)    { Serial.print(F(" Mono  "));} else { Serial.print(F(" Stereo"));}
   if ((auRDA5807_Reg[0x0A] & 0x8000) == 0)    { Serial.print(F(" ---"));   } else { Serial.print(F(" RDS"));   }
   Serial.print(F(" Sig= "));   if (rssi < 10) { Serial.print(F(" "));      } else  Serial.print(rssi);  Serial.print(F(" "));
   for(int i = 0; i < rssi - 5; i++) { Serial.print(F("*")); }
   Serial.println();
  }
}

//----------------------------------------
// RDA5807 Show Status
//----------------------------------------
void RDA5807_Get_RSSI()
{
  Wire.requestFrom (RDA5807_adrs, 16); 
  for (int i = 0; i < 8; i++) { auRDA5807_Reg[0x0A + i] = 256 * Wire.read () + Wire.read(); }
  Wire.endTransmission();
  rssi = auRDA5807_Reg[0x0B] >> 10;
}

//----------------------------------------
// SerialPrint 16Bit Number in HEX as hhll
//----------------------------------------
void Serial_printuih(unsigned int val)
{
  if (val < 0xF)   Serial.print(F("0"));                 // if less 2 Digit
  if (val < 0xFF)  Serial.print(F("0"));                 // if less 3 Digit
  if (val < 0xFFF) Serial.print(F("0"));                 // if less 4 Digit
  Serial.print(val,HEX);
  Serial.print(F(" "));
}

//----------------------------------------
// RDA5807 Radio Data System Dump Infos
//----------------------------------------
int RDA5807_RDS_Dump()
{
  Serial.println(" PI |GTxx|Asci");
  while(Serial.available()==0)
  {
    RDA5807_ReadW(4);                           // Read RDS-Data as 4 Word to Array               
    if((auRDS[1] & 0xF000)==0x2000)
    { 
//      Serial_printuih(auRDS[0]);                 // Block A  PI
//      Serial_printuih(auRDS[1]);                 // Block B  GT(5Bit)T(1Bit) PTY(5Bit)POS(5)Bit
//      Serial_printuih(auRDS[2]);
//      Serial_printuih(auRDS[3]);
//      int x = 16 + 4*(auRDS[1] & 0x000F);        
      for (int i=2;i<4;i++)  
      { 
        Serial.write(auRDS[i]>>8);               // Block C/D Ascii Code
        Serial.write(auRDS[i]&0xFF);             // 2 * 2 Byte
      }
    }
    if ((auRDS[1] & 0xF000)==0x4000)
    {
      int i_hh =(16*(auRDS[2] & 0x0001)+((auRDS[3] & 0xF000)>>12));
      int i_mm =(auRDS[3] & 0x0FC0)>>6;
      int i_ofs=(auRDS[3] & 0x003F);
      i_hh=i_hh+(i_ofs/2);
      if (i_hh <10){Serial.write(' ');} Serial.print(i_hh);  Serial.write(':');
      if (i_mm <10){Serial.write('0');} Serial.print(i_mm);  Serial.write(' ');
    }
   if ((auRDS[1]& 0xF000)==0x400)
   { 
    Serial.print(F("RDS CT: ")); for (int i=0;i<4;i++){ Serial_print16h(auRDS[i]); Serial.write(' | ');}  Serial.println();
    }
    delay(80);
    Serial.println();
  }
  return  0;
}

//----------------------------------------
// RDA5807 Radio Data System Dump Infos
//----------------------------------------
int RDA5807_RDS_DumpCT()
{
  int          i_gt,i_gab,i_pty,i_t,i_pos,i_hh,i_mm,i_ofs;
  RDA5807_Status();
  Serial.println(F(" PI |GTxx|Asci      GT  T PTY POS HH:mm Offset"));
  while(Serial.available()==0)
  {
    RDA5807_ReadW(4);                              // Read RDS-Data as 4 Word to Array
    i_gt = auRDS[1] >>12;
    if ((auRDS[1] & 0x0800)==0){i_gab='A';} else {i_gab='B';}
    i_t  =(auRDS[1] & 0x0400)>10;
    i_pty=(auRDS[1] & 0x03FF)>>5;
    i_pos=(auRDS[1] & 0x000F);
    i_hh =(16*(auRDS[2] & 0x0001)+((auRDS[3] & 0xF000)>>12));
    i_mm =(auRDS[3] & 0x0FC0)>>6;
    i_ofs=(auRDS[3] & 0x003F);
    i_hh=i_hh+(i_ofs/2);
    if (i_gt==4)
    {
    Serial_printuih(auRDS[0]);       // Block A  PI
    Serial_printuih(auRDS[1]);       // Block B  GT(4Bit) A/B(1Bit) T(1Bit) PTY(5Bit)POS(5)Bit
    Serial_printuih(auRDS[2]);
    Serial_printuih(auRDS[3]);
    if (i_gt <10){Serial.write(' ');} Serial.print(i_gt);  Serial.write(i_gab); Serial.write(' ');
    if (i_t  <10){Serial.write(' ');} Serial.print(i_t);   Serial.write(' ');
    if (i_pty<10){Serial.write(' ');} Serial.print(i_pty); Serial.print("  ");
    if (i_pos<10){Serial.write(' ');} Serial.print(i_pos); Serial.write(" ");
    if (i_hh <10){Serial.write(' ');} Serial.print(i_hh);  Serial.write(':');
    if (i_mm <10){Serial.write('0');} Serial.print(i_mm);  Serial.write(' ');
    Serial.print(i_ofs);
    Serial.println();
    }
    delay(80);
  }
  return  0;
}
//                                ------------------ End FM-radio
                          #endif FMRADIO
//********************************************************************************************

