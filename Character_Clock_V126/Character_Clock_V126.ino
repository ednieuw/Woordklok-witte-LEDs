// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 ot ARMEGA1284 chip on the PCB board that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05
- DCF77 module DCF-2
- FM Stereo Radio Module RDA5807M RRD-102V2.0  
- Red_MAX7219_8-Digit_LED_Display
- I2C LCD display
A 74HC595 ULN2803APG combination regulates the LEDs by shifting in bits into the 74HC595 LED are turn On or Off
A FT232RL 5.5V FTDI USB to TTL Serial Module can be attached to program te ATMEGA and read the serial port
The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module
The FM-module can be used to read the RDS-time from radio station or to play FM-radio station from within the clock

 Author .: Ed Nieuwenhuys
 Changes V107: Rotary bug, "noise on AC line", solved by changing rotary press/rotate options
 Changes V108: Changed time settings in rotary for Kees Soeters. Time text in ShiftInTime function instead from array
 Changes V109; Added features of Character_Colour_Clock_V067 like KEYPADs Menu structure, EEPROM struct
                 Solved all compiler warning. UseDCF turns off LED on board when turned Off. Use DCF added to menu
                 Changed constrain(mem,0,250) -->= min(mem,250)
 Changes V110: Added OneWire keypad 3x1 and 3x4
 Changes V111: Bugfixes 
 Changes V112; Clear NVRAM in menu option C. Store more menu text in PROGMEM  
 Changes V113; Added option W in menu; Test LDR every second. 
 Changes V114: DCF printing in DCFcheck()   
 Changes V115; Klok Merijn No12
 Changes V116: Rotary and KEYPAD entries updated. RTCklok.getTemperature()); in Dimleds(). DCF_count added
 Changes V117: Debugged RotaryEncoderCheck(void)
 Changes V117a & V120:  To turn On/Off LEDs  removed from DimLeds if(LEDsAreOff) BrightnessCalcFromLDR = 0;
                 and added in Displaytime() if(LEDsAreOff) {  LedsOff(); } 
 Changes V118; -1: Compileert voor Nano connect. 
                -2: bleSerial.beginAndSetupBLE("Woordklok nRF52")on RP2040
                -3 On Arduino BLE 33 IOT
                -4 On Arduino MKR1010  -- werkt nog niet
 Changes V119: nothing
 Changes V120: To turn On/Off LEDs  removed from DimLeds if(LEDsAreOff) BrightnessCalcFromLDR = 0;
                 and added in Displaytime() if(LEDsAreOff) {  LedsOff(); }
 Changes V121: Removed FM-radio   <--> DEAD END               
 Changes V122: From V117a  
                 Removed FMRADIOMOD   DS1820   LCDMOD MAX7219_8DIGIT ATMEAGA 1284 support
 Changes V123: Opgeruimd en Ulrich's klok toegevoegd
 Changes V124; Removed Selftest from Loop and added to one run Menu option S 
 Changes V125: Bij NLUV klok:  if (Iminute > 18 )  sayhour = Ihour+1;     if (Iminute > 19 )  sayhour = Ihour+1;    
 Changes V126: Twee fouten gevonden. Uitvallen van het UUR een minuut lang. De andere was een tijd die een minuut fout stond                    
 */
// ===============================================================================================================================
//--------------------------------------------
// ARDUINO Definition of installed modules
// Define the modules installed in the clock by removing the // before the #define
//--------------------------------------------
#define MOD_DS3231                  // The DS3231 module is installed, if not defined the Arduino internal clock is used
#define BLUETOOTHMOD                // Bluetooth or NTP ESP32 module connected to BT_RX BT_TX (pin 6 and 7)
//#define DCF77MOD                  // DCF77 installed
//#define ROTARYMOD                 // Use a rotary encode
//#define ONEWIREKEYPAD3x1          // Use a 3x1 keypad with one wire   
//#define ONEWIREKEYPAD3x4          // Use a 3x4 keypad with one wire
//#define FOURWIREKEYPAD            // 4 buttons and 4 wires (Do NOT use with Bluetooth on pin 6 & 7 or change connection pins) 

//--------------------------------------------
// ARDUINO Definition of installed language word clock
// Define only one definition!
//--------------------------------------------
// #define NL
 #define NLUV
// #define UK

//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <Wire.h>
#include <EEPROM.h>
#include <RTClib.h>
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>                 // for Bluetooth communication
                     #endif //BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>
                     #endif //ROTARYMOD
                     #ifdef DCF77MOD
#include <DCF77.h>
#include <TimeLib.h>
                     #endif //DCF77MOD

                     #ifdef NLUV
                // Ulrich Vertessen Clock display lay-out
#define HET     ShiftInTime("Het ",    0, 1,Toggle_HetWasIs,0);
#define IS      ShiftInTime("is ",     1, 1,Toggle_HetWasIs,1);  // Niet in klok aangesloten
#define WAS     ShiftInTime("was ",    2, 1,Toggle_HetWasIs,2);  // Niet in klok aangesloten
#define EDSOFT  ShiftInTime("Edsoft ", 2, 1, ToggleEdsoft  ,2);  // Niet in klok aangesloten
#define MVIJF   ShiftInTime("vijf ",   3, 1,              1,3);
#define MTIEN   ShiftInTime("tien ",   4, 1,              1,4);
#define KWART   ShiftInTime("kwart ",  5, 1,              1,5);
#define VOOR    ShiftInTime("voor ",   6, 1,              1,6);
#define MOVER   ShiftInTime("over ",   7, 1,              1,7);

#define OVER    ShiftInTime("over ",   8, 2,              1,0);
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
#define MVOOR   ShiftInTime("voor ",  23, 3,              1,7);         

                     #endif //NLUV
            // new lay-out   
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
#if defined(__AVR_ATmega328P__) 
// Digital hardware constants ATMEGA 328 ----
enum DigitalPinAssignments {
  DCF_PIN      = 2,               // DCFPulse on interrupt  pin
                                      #ifdef ROTARYMOD  
  encoderPinA  = 3,               // right (labeled DT on decoder)on interrupt  pin
  clearButton  = 4,               // switch (labeled SW on decoder)
  encoderPinB  = 8,               // left (labeled CLK on decoder)no interrupt pin  
                                      #endif //ROTARYMOD  
  PWMpin       = 5,               // Pin that controle PWM signal on BC327 transistor to dim light
                                      #ifdef BLUETOOTHMOD  
  BT_RX        = 6,               // BT_RX connect to TXD op de BT module
  BT_TX        = 7,               // BT_TX connect to RXD op de BT module
                                      #endif // BLUETOOTHMOD  
                                    #ifdef FOURWIREKEYPAD   
  Button2      = 5,               // Plus 
  Button4      = 6,               // Select mode / start
  Button1      = 7,               // Minus 
  Button3      = 8,               // Select mode / start
                                   #endif // FOURWIREKEYPAD    
  DCF_LED_Pin  = 9,               // define LED pin voor DCF pulse
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
# endif

//--------------------------------------------                                                //
// KY-040 ROTARY
//-------------------------------------------- 
                                  #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);    // Use digital pin  for encoder
                                  #endif //ROTARYMOD      
unsigned long Looptime          = 0;
unsigned long RotaryPressTimer  = 0;
byte          NoofRotaryPressed = 0;
//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
const byte MAXBRIGHTNESS    = 120;           // Factor (%) to dim LED intensity with. Between 1% and 250%
const byte LOWBRIGHTNESS    = 20;           // Lower limit of Brightness ( 0 - 255)   
bool  LEDsAreOff            = false;        // If true LEDs are off except time display
byte  BrightnessCalcFromLDR = 32;           // Initial brightness value The intensity send to the LEDs (0-255)
byte  Menumem[24];                          // Storage of DCF or LDR readings per hour
byte  TestLDR               = false;        // If true LDR inf0 is printed every second in serial monitor
int   Previous_LDR_read     = 512;          // The actual reading from the LDR + 4x this value /5
int   OutPhotocell;                         // Stores reading of photocell;
int   MinPhotocell          = 1023;         // Stores minimum reading of photocell;
int   MaxPhotocell          = 1;            // Stores maximum reading of photocell;
int   ToggleEdsoft          = 1;
uint32_t SumLDRreadshour    = 0;
uint32_t NoofLDRreadshour   = 0;

//--------------------------------------------                                                //
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 50
static unsigned long msTick;                                                                  // Millisecond ticks 
int    count; 
byte   Isecond, Iminute, Ihour , Iday, Imonth, Iyear; 
byte   Display1 = 0 , Display2 = 0, Display3 = 0;
byte   lastday  = 0,lastminute = 0, lasthour = 0, sayhour  = 0;
int    Delaytime            = 500;
byte   Toggle_HetWasIs      = 1;                                                              // Turn On/Off HetIsWas lights
int    Toggle_HetWasIsUit   = 0;                                                              // Turn off HetIsWas after 10 sec
byte   ChangeTime           = false;
byte   ChangeLightIntensity = false;
byte   SecPulse             = 0;                                                              // Give a pulse to the Isecond led
byte   Demo                 = false;
byte   hbval                = 128;
byte   hbdelta              = 8;
String SerialString;

//                                                                                            //
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
                                  #endif  //MOD_DS3231
DateTime Inow;

//--------------------------------------------                                                //
// BLUETOOTH
//--------------------------------------------                                     
                                  #ifdef BLUETOOTHMOD
SoftwareSerial Bluetooth(BT_RX, BT_TX);     // RX <=> TXD on BT module, TX <=> RXD on BT module
                                  #endif // BLUETOOTHMOD  
//--------------------------------------------                                                //
// DCF-2 DCF77 MODULE
//--------------------------------------------
byte DCF_signal = 50;                                                                         // Is a proper time received?
byte DCF_counts = 0;                                                                          // Valid DCF receptions per hour
bool SeeDCFsignalInDisplay = false;                                                           // If ON then the display line HET IS WAS will show the DCF77-signal received
bool UseDCF                = true;                                                            // Use the DCF-receiver or not

                                  #ifdef DCF77MOD                                             // DCF77 ------------------------------
#if defined(__AVR_ATmega328P__) 
#define DCF_INTERRUPT 0                                                                       // DCF Interrupt number associated with DCF_PIN
#endif
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW);
                                  #endif //DCF77MOD                      

//--------------------------------------------                                                //
// Common
//----------------------------------------
//                                                                                            //
char sptext[MAXTEXT+2];                                                                       // For common print use   
uint16_t  MilliSecondValue  = 10;                                                             // The duration of a second  minus 1 ms. Used in Demo mode
struct EEPROMstorage {                                                                        // Data storage in EEPROM to maintain them after power loss
  byte LightReducer;
  byte LowerBrightness;
  byte TurnOffLEDsAtHH;
  byte TurnOnLEDsAtHH;
  unsigned int  DCForLDR;
} Mem ; 
                    // End Definitions  ---------------------------------------------------------
                               
//--------------------------------------------                                                //
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
if (Demo)          Demomode(); 
else               EverySecondCheck();                                                        // Let the second led tick and run the clock program
InputDevicesCheck();                                                                          // Check input from input devices
}  
//--------------------------------------------                                                //
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                                                                             // Initialise the hardware // initialize the appropriate pins as outputs:
 Serial.begin(9600);                                                                          // Setup the serial port to 9600 baud 
 Tekstprintln("***\nSerial started\nInstalled:");
 Wire.begin();                                                                                // Start the wire communication I2C
                                  #ifdef  MOD_DS3231
 RTCklok.begin();                                                                             // Start the DS3231 RTC-module
 Tekstprintln("DS3231 RTC");
                                  #endif //MOD_DS3231
 pinMode(LEDClockPin,  OUTPUT); 
 pinMode(LEDDataPin,   OUTPUT); 
 pinMode(LEDStrobePin, OUTPUT); 
 pinMode(PWMpin,       OUTPUT);
 pinMode(secondsPin,   OUTPUT );
                                  #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary"); 
 myEnc.write(0);                                                                              // Clear Rotary encode buffer
                                  #endif //ROTARYMOD  
                                  #ifdef FOURWIREKEYPAD   
  pinMode(Button1,  INPUT_PULLUP);
  pinMode(Button2,  INPUT_PULLUP);
  pinMode(Button3,  INPUT_PULLUP);
  pinMode(Button4,  INPUT_PULLUP);  
 Tekstprintln("Four wire keypad");                                                            // The one wire keypad is enabled
                                  #endif // FOURWIREKEYPAD                                 
                                  #ifdef KEYPAD3x1
 // keypad.addEventListener(keypadEvent);                                                     // No interrupt needed. Loop is fast enought. Start the 3x1 keypad
 Tekstprintln("3*1 keypad");  
 digitalWrite(COLPIN,LOW);
                                  #endif //KEYPAD3x1
                                  #ifdef ONEWIREKEYPAD3x4  
 Tekstprintln("Onewire keypad");                                                              // The one wire keypad is enabled
                                  #endif //ONEWIREKEYPAD3x4

                                  #ifdef BLUETOOTHMOD   
 Bluetooth.begin(9600);                                                                       // Setup the Bluetooth port to 9600 baud 
 Tekstprintln("Bluetooth");
                                  #endif //BLUETOOTHMOD
                                  #ifdef DCF77MOD         
 pinMode(DCF_LED_Pin,  OUTPUT);
 pinMode(DCF_PIN,INPUT_PULLUP); 
 DCF.Start();                                                                                 // Start the DCF-module
 Tekstprintln("DCF77");
                                  #endif //DCF77MOD
 analogWrite(PWMpin, BrightnessCalcFromLDR);                                                  // The duty cycle: between 0 (lights off) and 255 (light full on).    
 GetTijd(1);                                                                                  // Get the time and print it to serial                
 DateTime now = RTCklok.now();                                                               // Get the time from the RTC
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
   {
    Tekstprintln("RTC Updating");                                                             // Following line sets the RTC to the date & time this sketch was compiled
    RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
   } 
 EEPROM.get(0,Mem);                                                                           // Get the data from EEPROM
 Mem.LightReducer    = constrain(Mem.LightReducer,   1, 250);                                 // 
 Mem.LowerBrightness = constrain(Mem.LowerBrightness,1, 250);                                 // 
 Mem.TurnOffLEDsAtHH = min(Mem.TurnOffLEDsAtHH, 23);                                          //
 Mem.TurnOnLEDsAtHH  = min(Mem.TurnOnLEDsAtHH,  23);                                          //
 Mem.DCForLDR        = min(Mem.DCForLDR,  1);                                                 //
 EEPROM.put(0,Mem);                                                                           // Update EEPROM if some data are out of the constrains
 SWversion();                                                                                 // Display the version number of the software
// Selftest();                                                                                // Play the selftest
 Previous_LDR_read = analogRead(PhotoCellPin);                                                // to have a start value
 MinPhotocell      = Previous_LDR_read;                                                       // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                                                       // Stores maximum reading of photocell;
 GetTijd(0);                                                                                  // Get the time (to be sure time is correct) and do not print it to serial 
 lasthour = Ihour;
 while (Serial.available())  char c = Serial.read();                                          // Flush the serial buffer. When a Bluetooth module is connected to RX/TX pin 0&1 garbage can set the time to 00:00:00
 TestLDR           = false;
 Looptime          = millis();                                                                // Used in KY-040 rotary
 msTick            = Looptime; 
}
//                                                                                            //
// --------------------------- END SETUP     
//--------------------------------------------                                                //
// Menu definition
// Keep chars per line a few chars less than nn in array definition menu[][nn]
//--------------------------------------------  
//0        1         2         3         4         5
//12345678901234567890123456789012345678901234567890
const char menu[][33] PROGMEM =  {
 "Woordklok",
 "Enter time as hhmmss or hhmm",
 "A Normal display",
 "B HET IS WAS always off",
 "C HET IS WAS off after 10 secs",
 "D D15122017 is 15 Dec 2017",
                                  #ifdef DCF77MOD   
 "F DCF or LDR counts/h",
 "G Display DCF-signal",
 "H Use DCF-receiver",
                                  #endif //DCF77MOD   
 "I For this info",
 "L Min light intensity (0-255)",
 "M Max light intensity (1-250)",
 "N LEDs OFF between Nhhhh",
 "O LEDs ON/OFF",
 "R Reset to default settings",
 "S Self test",
 "W Test LDR every second",
 "X Demo mode (Xmsec)",
 "Ed Nieuwenhuys July 2024" };                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
//--------------------------------------------                                                //
// CLOCK Version info
//--------------------------------------------
// Storing strings in PROGMEM saves 
// global variable memory
//--------------------------------------------
//0        1         2         3
//123456789012345678901234567890
const char SWVER[][28] PROGMEM =  {
 "LDR average bits per hour",
 "DCF receptions per hour",
 "Brightness Min  : ",
 "LDR read Min    :",
 "LEDs off between: ",
 "DCF-receiver is : ",
 "Software: "};

//--------------------------------------------                                                //
// CLOCK Menu and Version info
//--------------------------------------------
void SWversion(void) 
{ 
 unsigned int i;
 #define FILENAAM (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
 PrintLine(40);
 for (i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)   {strcpy_P(sptext, menu[i]);                Tekstprintln(sptext);}
 PrintLine(47);
 sprintf(sptext,"%s",strcpy_P(sptext, SWVER[Mem.DCForLDR]));                                       Tekstprintln(sptext);
 for (i=0;i<12;i++) { sprintf(sptext," %02d ",i );        Tekstprint(sptext); }                    Tekstprintln("");
 for (i=0;i<12;i++) { sprintf(sptext,"%03d ",Menumem[i]); Tekstprint(sptext); }                    Tekstprintln("");
 for (i=12;i<24;i++){ sprintf(sptext,"%03d ",Menumem[i]); Tekstprint(sptext); }                    Tekstprintln("");
 PrintLine(47);
 sprintf(sptext,"%s%3d bits Max: %3d%%",strcpy_P(sptext, SWVER[2]),Mem.LowerBrightness, Mem.LightReducer); Tekstprintln(sptext);
 sprintf(sptext,"%s%4d bits Max: %3d bits",strcpy_P(sptext, SWVER[3]),MinPhotocell, MaxPhotocell);  Tekstprintln(sptext); 
 sprintf(sptext,"%s%02d - %02d",strcpy_P(sptext,SWVER[4]),Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH);  Tekstprintln(sptext);
                                   #ifdef DCF77MOD   
 sprintf(sptext,"%s%s  Signal:%3d%%",strcpy_P(sptext,SWVER[5]),UseDCF ? "On" : "Off",DCF_signal);   Tekstprintln(sptext);
                                   #endif // DCF77MOD   
// sprintf(sptext,"%s%s",strcpy_P(sptext,SWVER[6]),VERSION);                                          Tekstprintln(sptext); 
 sprintf(sptext,"Software: %s",FILENAAM);                                                           Tekstprintln(sptext);  //VERSION); 

 GetTijd(1);  
 PrintLine(47);
}
//--------------------------------------------                                                //
// CLOCK Print line with underscores
//--------------------------------------------
void PrintLine(byte Lengte)
{
 sptext[0] = 0;
 for (int n=0; n<Lengte; n++) strcat(sptext,"_");
 Tekstprintln(sptext);      
}
//--------------------------------------------                                                //
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 Mem.LightReducer     = MAXBRIGHTNESS;                                                        // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.LowerBrightness  = LOWBRIGHTNESS;                                                        // Lower limit of Brightness ( 0 - 255)
 Mem.TurnOffLEDsAtHH  = Mem.TurnOnLEDsAtHH = 0;                                               // Reset Off On time
 Mem.DCForLDR         = 0;                                                                    // LDR reading in menu
 EEPROM.put(0,Mem);                                                                           // Update EEPROM if some data are out of the constrains                                                 
 Previous_LDR_read    = analogRead(PhotoCellPin);                                             // To have a start value
 MinPhotocell         = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
 MaxPhotocell         = Previous_LDR_read;                                                    // Stores maximum reading of photocell;
 TestLDR              = false;                                                                // If true LDR display is printed every second
 NoofRotaryPressed    = 0; 
 SeeDCFsignalInDisplay= false;
 ChangeTime           = false;
 ChangeLightIntensity = false;
 Demo                 = false;
 for (int i=0;i<24;i++) Menumem[i] = 1;                                                       // Reset LDR/DCF readings
 Selftest();                                                                                  // Play the selftest
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 Displaytime();
}
//                                                                                            //
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{ 
  uint32_t ms = millis() - msTick;                                                            // A Digitalwrite() is very time consuming. 
  static bool Dpin;                                                                           // Only write once to improve program speed in the loop()
  if ( ms >1 && Dpin) {Dpin = LOW; digitalWrite(secondsPin,LOW);}                             // Turn OFF the second on pin 13 
  if ( ms >999)                                                                               // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                                                         // second++; 
   digitalWrite(secondsPin,HIGH);                                                             // turn ON the second on pin 13
   Dpin = HIGH;
   ++SecPulse;                                                                                // second routine in function DimLeds
   GetTijd(0);                                                                                // synchronize time with RTC clock
   if ((Toggle_HetWasIsUit == 2) && (Isecond > 10)) Toggle_HetWasIs = 0; 
      else Toggle_HetWasIs = 1;                                                               // HET IS WAS is On
   if(Isecond % 30 == 0) DimLeds(true);                                                       // Led Intensity Control + seconds tick print every 30 seconds   
      else               DimLeds(TestLDR);
  if ((Toggle_HetWasIsUit == 2) && (Isecond == 11))Displaytime();                             // turn Leds OFF on second == 11
  if(Iminute == 0 && Isecond <9)
   { 
    ToggleEdsoft = Isecond % 2;                                                               // ToggleEdsoft becomes 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light on pin 24
    Serial.println(ToggleEdsoft);
    Displaytime();
    Print_RTC_tijd();
   }
  if (Iminute != lastminute) EveryMinuteUpdate();  
  }
 }
//--------------------------------------------                                                //
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
  lastminute = Iminute;
  Displaytime();
  Print_RTC_tijd();
                                    #ifdef DCF77MOD  
  DCF_signal--;
  DCF_signal = constrain((byte) DCF_signal,1,99); 
                                    #endif // DCF77MOD  
  if(Ihour != lasthour) EveryHourUpdate();
 }
//--------------------------------------------                                                //
// CLOCK Update routine done every hour
//--------------------------------------------
void EveryHourUpdate(void)
{
 GetTijd(0); 
 if(Ihour == Mem.TurnOffLEDsAtHH) LEDsAreOff = true;                                          // Is it time to turn off the LEDs?
 if(Ihour == Mem.TurnOnLEDsAtHH)  LEDsAreOff = false;                                         // Or on?
                                    #ifdef DCF77MOD    
 Menumem[lasthour] =  DCF_counts;                                                             // Valid DCF receptions
 DCF_counts        = 0;                                                                       // Valid DCF receptions per hour
 
                                    #else
 Menumem[lasthour] = SumLDRreadshour / NoofLDRreadshour;                                      // Update the average LDR readings per hour
                                    # endif  //DCF77MOD
 SumLDRreadshour  = 0;
 NoofLDRreadshour = 0;
 lasthour = Ihour;
 if (Iday != lastday) EveryDayUpdate(); 
}
//--------------------------------------------                                                //
// CLOCK Update routine done every day
//--------------------------------------------
void EveryDayUpdate(void)
{
 lastday = Iday;                                                                              // Not much to do at the moment
 MinPhotocell = 999;                                                                          // Stores minimum reading of photocell;
 MaxPhotocell = 1;                                                                            // Stores maximum reading of photocell;
}
//--------------------------------------------                                                //
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char const *tekst)
{
 Serial.print(tekst);    
                                  #ifdef BLUETOOTHMOD
 Bluetooth.print(tekst);  
                                  #endif //BLUETOOTHMOD
}
void Tekstprintln(char const *tekst)
{
 Serial.println(tekst);    
                                  #ifdef BLUETOOTHMOD
 Bluetooth.println(tekst);
                                  #endif //BLUETOOTHMOD
}
//--------------------------------------------                                                //
// CLOCK check for input from devices
// This fubction is called from with the loop()
//------------------------------------------------------------------------------
//                                                                                            //
void InputDevicesCheck(void)
{
 SerialCheck();
                                  #ifdef DCF77MOD
 if(UseDCF)  DCF77Check();
                                  #endif  //DCF77MOD
                                  #ifdef BLUETOOTHMOD   
 BluetoothCheck(); 
                                  #endif  //BLUETOOTHMOD                                 
                                  #ifdef ROTARYMOD      
 RotaryEncoderCheck(); 
                                  #endif  //ROTARYMOD
                                  #ifdef KEYPAD3x4   
 Keypad3x4Check(); 
                                  #endif  //KEYPAD3x4
                                  #ifdef KEYPAD3x1   
 Keypad3x1Check(); 
                                  #endif  //KEYPAD3x1
                                  #ifdef ONEWIREKEYPAD3x4   
 OnewireKeypad3x4Check(); 
                                  #endif  //ONEWIREKEYPAD3x4
                                  #ifdef ONEWIREKEYPAD3x1   
 OnewireKeypad3x1Check(); 
                                  #endif  //ONEWIREKEYPAD3x1
                                  #ifdef FOURWIREKEYPAD 
 FourwireKeypadCheck();
                                  #endif //FOURWIREKEYPAD                        
                                  #ifdef HC12MOD
 HC12Check();
                                  #endif  //HC12MOD 
}
//--------------------------------------------                                                //
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
 if ( millis() - msTick == 10)   digitalWrite(secondsPin,LOW);                                // Turn OFF the second on pin 13
 if ( millis() - msTick >= MilliSecondValue)                                                  // Flash the onboard Pin 13 Led so we know something is happening
 {    
  msTick = millis();                                                                          // second++; 
  digitalWrite(secondsPin,HIGH);                                                              // Turn ON the second on pin 13
  //Isecond = 60-Iminute;
  if( ++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
  if( Ihour >12) Ihour = 0;                                                                   // If hour is after 12 o'clock 
  LedsOff(); 
  DimLeds(false);
  Displaytime();
  Tekstprintln("");
  digitalWrite(secondsPin,LOW);                                                              // Turn ON the second on pin 13
  SerialCheck();
 }
}
//--------------------------------------------                                                //
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String SerialString = "";
 while (Serial.available())
  {
   char c = Serial.read();  delay(3); 
   if (c>31 && c<127) SerialString += c;                                                      // allow input from Space - Del
   else c = 0;
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);                            // Rework ReworkInputString();
 SerialString = "";
}
                                  #ifdef HC12MOD
//--------------------------------------------                                                //
// CLOCK check for HC-12 input
//--------------------------------------------
                        
void HC12Check(void)
{
 String HC12String = ""; 
 HC12String.reserve(64);
 HC12String ="";
 HC12.listen();                                                                               // When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (HC12.available()>0)                                                                   // If HC-12 has data
   {       
    char c = HC12.read();     
    if (c>31 && c<127)  HC12String += c;
    else c = 0;                                                                               // Allow only input from Space - Del
    delay(3);
   }
 HC12String += "\n";
 if (HC12String.length()>0) 
   {
    Serial.print("Received HC-12: "); Serial.println(HC12String);
    ReworkInputString(HC12String);
    }                  
}                         
                                  #endif  //HC12MOD
                                  #ifdef BLUETOOTHMOD
//--------------------------------------------                                                //
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 unsigned long looptimeBT = millis();                                                         // Avoid an hangup in this loop 
 String BluetoothString;
 
 while (Bluetooth.available() and (millis() - looptimeBT < 10000) )
  {
   delay(3); 
   char c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<123) BluetoothString += c;
//   else c = 0;     // delete a CR
  }
 if (BluetoothString.length()>0) ReworkInputString(BluetoothString);                          // Rework ReworkInputString
 BluetoothString = "";
}
                                 #endif //BLUETOOTHMOD
                                 #ifdef DCF77MOD
//--------------------------------------------                                                //
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 time_t DCFtime = DCF.getTime();                                                              // Check if new DCF77 time is available
 if (DCFtime!=0)                                                                              // If a valid time/date is received ...
  {
   DCF_signal+=2;                                                                             // Add 2 to the counter. NB 1 is subtracted every minute 
   DCF_counts++;                                                                              // Valid DCF receptions per hour
   setTime(DCFtime);                                                                          // Update the internal time of the Arduino (Used by the DCF library) RTC time
   Inow = RTCklok.now();                                                                      // Update the internal time in the variabele of this program
   Tekstprint("DCF OK: ");
   if(abs(DCFtime - Inow.unixtime()) > 2)                                                     // If the time differs more than a second -> update
      {
       RTCklok.adjust(DCFtime);                                                               // Update the  RTC time
       Tekstprintln("Time NOK updated");
       Print_RTC_tijd();
      }
   else Tekstprintln("Time OK");
  }
 bool LHbit = digitalRead(DCF_PIN);
 if (UseDCF)  digitalWrite(DCF_LED_Pin, 1 - LHbit );                                          // Write inverted DCF pulse to LED on board 
 if (SeeDCFsignalInDisplay)
  {
   Toggle_HetWasIs = LHbit;
   Display1 |= (Toggle_HetWasIs<<0);                                                          // Turn off the  HET IS WAS LEDs
   Displaytime();
  }
  DCF_signal = min(DCF_signal, 100);                                                          // Keep DCF_signal counter <=100
} 
                                  #endif //DCF77MOD                            
                                  #ifdef ONEWIREKEYPAD3x4
//--------------------------------------------                                                //
// KEYPAD check for Onewire Keypad input
//--------------------------------------------
void OnewireKeypad3x4Check(void)
{
 byte keyvalue;
 char Key;
 int sensorValue = analogRead(OneWirePin); // read the value from the sensor:
 switch(sensorValue)
  {
    case   0 ... 100:  keyvalue = 13; break;   // noise
    case 101 ... 132:  keyvalue = 12; Key = '*'; break;   // * 
    case 133 ... 154:  keyvalue =  0; Key = '0'; break;   // 0 
    case 155 ... 216:  keyvalue = 11; Key = '#'; break;   // # 
    case 217 ... 281:  keyvalue =  7; Key = '7'; break;   // 7 
    case 282 ... 318:  keyvalue =  4; Key = '4'; break;   // 4 
    case 319 ... 349:  keyvalue =  1; Key = '1'; break;   // 1 
    case 350 ... 390:  keyvalue =  8; Key = '8'; break;   // 8 
    case 391 ... 463:  keyvalue =  5; Key = '5'; break;   // 5 
    case 464 ... 519:  keyvalue =  2; Key = '2'; break;   // 2 
    case 520 ... 619:  keyvalue =  9; Key = '9'; break;   // 9 
    case 620 ... 848:  keyvalue =  6; Key = '6'; break;   // 6 
    case 849 ... 1023: keyvalue =  3; Key = '3'; break;   // 3
  }
 if(keyvalue<13) { Serial.println(Key); delay(300); }
  if (Key == 12)   // *                                                                       // Pressing a * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString ="";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                                                      // Turn all LEDs green
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>=0 && Key<10))
   {
    delay(20); 
    KeypadString += Key;                                                                      // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                                          // Turn all LEDs red
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if (KeypadString.length()>5)                                                                 // If six numbers are entered rework this to a time hhmmss
   {       
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Settings reset"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                                                        // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }    
   }
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000) ) 
   {  
    KeyInputactivated = false;                                                                // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
}
                                  #endif  //ONEWIREKEYPAD3x4  
                                  #ifdef ONEWIREKEYPAD3x1
//--------------------------------------------                                                //
// KEYPAD check for Onewire Keypad input with 5V and 1.1, 4.7, 4.7, 4.7 kOhm resistors
//--------------------------------------------
void OnewireKeypad3x1Check(void)
{
 char keyvalue, Key;
 int sensorValue = analogRead(OneWirePin);                                                    // Read the value from the sensor:
 switch(sensorValue)
   {
    case   0 ... 385:  keyvalue = 99;            break;                                       // Noise
    case 386 ... 635:  keyvalue = -1; Key = 'G'; break;                                       // G 
    case 636 ... 910:  keyvalue =  0; Key = 'Y'; break;                                       // Y 
    case 911 ... 1024: keyvalue =  1; Key = 'R'; break;                                       // R 
   }
 if(keyvalue<2) 
    { 
     Serial.print(sensorValue); Serial.println(Key); 
     if (Key == 'R') ProcessKeyPressTurn(1);                                                  // Pressing Red increases hour or minute. 
     if (Key == 'G') ProcessKeyPressTurn(-1);                                                 // Pressing Green decreases hour or minute. 
     if (Key == 'Y') ProcessKeyPressTurn(0);                                                  // Pressing Yellow activates the keyboard input. 
     delay(200);     
    }
}
                                  #endif //ONEWIREKEYPAD3x1
                                  #ifdef FOURWIREKEYPAD 
//--------------------------------------------                                                //
// KEYPAD 4 buttons and four wires
//--------------------------------------------
void FourwireKeypadCheck(void)
{
if ((digitalRead(Button1) == LOW) || (digitalRead(Button2) == LOW) 
||  (digitalRead(Button3) == LOW) || (digitalRead(Button4) == LOW)) delay(150);               // Avoid spike

 if (digitalRead(Button1) == LOW) ProcessKeyPressTurn(-1);    // - 1 
// if (digitalRead(Button3) == LOW) ProcessKeyPressTurn( 0);    // Select / start             // Do not use button3
 if (digitalRead(Button2) == LOW) ProcessKeyPressTurn( 1);    // + 1 
 if (digitalRead(Button4) == LOW) ProcessKeyPressTurn( 0);    // Select / start                                                        
}
                                  #endif //FOURWIREKEYPAD 
                                  #ifdef KEYPAD3x4
//--------------------------------------------                                                //
// KEYPAD check for Keypad input
//--------------------------------------------                          
void Keypad3x4Check(void)
{ 
 char Key = 0;
 Keypad3x4.tick(); 
 while(Keypad3x4.available())
  {
   keypadEvent e = Keypad3x4.read();  
   if(e.bit.EVENT == KEY_JUST_PRESSED)
     {                                                                                        // Serial.println(F(" pressed"));  
      delay(20);
     }  
   else  if(e.bit.EVENT == KEY_JUST_RELEASED) 
     {
      Key = (char) e.bit.KEY;                                                                 // Serial.print(Key);  Serial.println(F(" released"));
      Keypad3x4.clear();
      delay(20);
     }
   }
 if (Key == 42)   // *                                                                        // Pressing * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString = "";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                                                      // Turn all LEDs green
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>47 && Key<58))
   {
    delay(20); 
    KeypadString += Key;                                                                      // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                                          // Turn all LEDs red
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if(KeypadString.length()>5)                                                                  // If six numbers are entered rework this to a time hhmmss
   {  
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock setting resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                                                        // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }
   }
  if (Key == 35)   // #                                                                       // Pressing # changes palettes. 
   { 
    KeypadString ="";
    Mem.DisplayChoice++;
    Mem.DisplayChoice = min(Mem.DisplayChoice, 9);
    Displaytime();
   }
   
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000)                                 // Stop keyboard entry after 30 seconds
   {  
    KeyInputactivated = false;                                                                // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
} 
                                  #endif  //KEYPAD3x4   
//                                                                                            //
                                  #ifdef KEYPAD3x1
//--------------------------------------------                                                //
// KEYPAD check for Keypad input
//--------------------------------------------                           
void Keypad3x1Check(void)
{ 
 char Key = 0;
 Keypad3x1.tick(); 
 while(Keypad3x1.available())
  {
   keypadEvent e = Keypad3x1.read();  
   Serial.print((char)e.bit.KEY);
   Key = (char) e.bit.KEY;   
   if(e.bit.EVENT == KEY_JUST_PRESSED) {Serial.println(F(" pressed"));}  
   if(e.bit.EVENT == KEY_JUST_RELEASED){Serial.println(F(" released"));}
   }
 if (Key == 'G') ProcessKeyPressTurn(-1);                                                     // Pressing Yellow activates the keyboard input. 
 if (Key == 'Y') ProcessKeyPressTurn( 0);                                                     // Pressing Yellow activates the keyboard input. 
 if (Key == 'R') ProcessKeyPressTurn( 1);                                                     // Pressing Yellow activates the keyboard input. 
 Keypad3x1.clear();                                                                           // and clear the keyboard buffer
 delay(200);
} 
                                 #endif  //KEYPAD3x1      
                                 #ifdef ROTARYMOD
//--------------------------------------------                                                //
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
void RotaryEncoderCheck(void)
{
 int ActionPress = 999;
 if (digitalRead(clearButton) == LOW )          ProcessKeyPressTurn(0);                       // Set the time by pressing rotary button
 else if (ChangeTime)    
  {   
   ActionPress = myEnc.read();                                                                // If the knob is turned store the direction (-1 or 1)
   if (ActionPress == 0) {  ActionPress = 999;  ProcessKeyPressTurn(ActionPress);  }          // Sent 999 = nop 
   if (ActionPress == 1 || ActionPress == -1 )  ProcessKeyPressTurn(ActionPress);             // Process the ActionPress
  } 
 myEnc.write(0);                                                                              // Set encoder pos back to 0
}
                                  #endif  //ROTARYMOD  

//--------------------------------------------                                                //
// CLOCK
// KY-040 or Membrane 3x1 processing input
// encoderPos < 1 left minus 
// encoderPos = 0 attention and selection choice
// encoderPos > 1 right plus
//--------------------------------------------
void ProcessKeyPressTurn(int encoderPos)
{
if ((unsigned long) (millis() - RotaryPressTimer) > 60000)                                    // After 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        Tekstprintln("<-- Changing time is over -->");
        NoofRotaryPressed = 0;
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }  
 if (ChangeTime || ChangeLightIntensity)                                                      // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos!=999 && ( (millis() - Looptime) > 250))                                   // If rotary turned avoid debounce within 0.25 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos == 1)                                                                     // Increase  
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }                                  // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                                                        // Change hours
              {if( ++Ihour >23) { Ihour = 0; } }      
           if (NoofRotaryPressed == 2)                                                        // Change minutes
              {  Isecond = 0;
               if( ++Iminute >59) { Iminute = 0; if( ++Ihour >23) { Ihour = 0; } }   }
           } 
        }    
      if (encoderPos == -1)                                                                   // Decrease
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }    // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                                                        // Change hours
            { if( Ihour-- ==0) { Ihour = 23; } }      
           if (NoofRotaryPressed == 2)                                                        // Change minutes
            { Isecond = 0;
             if( Iminute-- == 0) { Iminute = 59; if( Ihour-- == 0) { Ihour = 23; } }  }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
      Looptime = millis();       
     }                                                
   }
 if (encoderPos == 0 )                                                                        // Set the time by pressing rotary button
   { 
    delay(250);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    SeeDCFsignalInDisplay = false;                                                            // Shows the DCF-signal NOT in the display
    RotaryPressTimer      = millis();                                                         // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >6 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                                                // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true;     BlinkUUR(3, 20);             break;                    // Change the hours
       case 2:  ChangeTime = true;     BlinkHETISWAS(3, 20);        break;                    // Change the hours        
       case 3:  ChangeLightIntensity = true;  
                Display1 = Display2 = Display3 = 255; Laatzien();   break;                    // Turn on all LEDs and change intensity 
       case 4:  SeeDCFsignalInDisplay = true;                       break;                    // Shows the DCF-signal in the display                               
       case 5:                                                      break;
       case 6:  Reset();                                            break;                    // Reset all settings                                                                  
      default:                                                      break;                     
      }
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed);   
    Looptime = millis();     
    Displaytime();                                                                            // Turn on the LEDs with proper time
   }
 }

//--------------------------------------------                                                //
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{
  GetTijd(1);                                                                                 // Prints time in Serial monitor
  LedsOff(); 
                #ifdef NLUV
  HET;     Laatzien(); IS;      Laatzien(); MVIJF;   Laatzien(); 
  MTIEN;   Laatzien(); MVOOR;   Laatzien(); MOVER;   Laatzien();
  KWART;   Laatzien(); HALF;    Laatzien(); OVER;    Laatzien(); 
  VOOR;    Laatzien(); EEN;     Laatzien(); TWEE;    Laatzien();  
  DRIE;    Laatzien(); VIER;    Laatzien(); VIJF;    Laatzien(); 
  ZES;     Laatzien(); ZEVEN;   Laatzien(); NEGEN;   Laatzien(); 
  ACHT;    Laatzien(); TIEN;    Laatzien(); ELF;     Laatzien();  
  TWAALF;  Laatzien(); UUR;     Laatzien(); EDSOFT;  Laatzien();
                #endif //NLUV

#ifdef NL 
  HET;     Laatzien(); IS;      Laatzien(); WAS;     Laatzien();
  PRECIES; Laatzien(); MVIJF;   Laatzien(); KWART;   Laatzien(); 
  MTIEN;   Laatzien(); OVER;    Laatzien(); VOOR;    Laatzien(); 
  HALF;    Laatzien(); TWAALF;  Laatzien(); ELF;     Laatzien();  
  EEN;     Laatzien(); TIEN;    Laatzien(); TWEE;    Laatzien(); 
  NEGEN;   Laatzien(); DRIE;    Laatzien(); ACHT;    Laatzien(); 
  VIER;    Laatzien(); ZEVEN;   Laatzien(); VIJF;    Laatzien();  
  ZES;     Laatzien(); UUR;     Laatzien(); EDSOFT;  Laatzien();
#endif //NL
#ifdef UK
  HET;     Laatzien();  IS;      Laatzien();  WAS;     Laatzien();
  PRECIES; Laatzien();  HALF;    Laatzien();  TWINTIG; Laatzien();
  MVIJF;   Laatzien();  KWART;   Laatzien();  MTIEN;   Laatzien();
  OVER;    Laatzien();  VOOR;    Laatzien();  ZES;     Laatzien();  
  TWEE;    Laatzien();  VIJF;    Laatzien();  TWAALF;  Laatzien();
  TIEN;    Laatzien();  ELF;     Laatzien();  VIER;    Laatzien();
  NEGEN;   Laatzien();  DRIE;    Laatzien();  ACHT;    Laatzien();
  ZEVEN;   Laatzien();  EEN;     Laatzien();  UUR;     Laatzien();
  #endif //UK  

  for(int i=0; i<2; i++)
  {
   Display1=255;   Display2=255;   Display3=255;  Laatzien();
   Display1=0;     Display2=0;     Display3=0;    Laatzien();
  }  
  Play_Lights(); 
  LedsOff(); 
  DimLeds(false);
  Displaytime();    
}
//--------------------------------------------                                                //
// CLOCK set the LED's for displaying
//--------------------------------------------
void Displaytime(void)
{
 LedsOff(); 
 if(LEDsAreOff) { WriteLEDs(); return; }                                                                                  // Start by clearing the display to a known state

                             #ifdef NL
 HET;                                                                                         // HET light is always on
 switch (Iminute)
 {
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
 }
 
 sayhour = Ihour;
 if (Iminute > 18 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                            #endif //NL

                            #ifdef UK
 HET;                                                                                         // HET light is always on
 switch (Iminute)
 {                            
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
 }

 sayhour = Ihour;
 if (Iminute > 33 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif //UK
                                        #ifdef NLUV
 HET; IS;                                                                                       // HET light is always on
 switch (Iminute)
 {
  case  0: 
  case  1: 
  case  2: 
  case  3: 
  case  4: break;
  case  5: 
  case  6:   
  case  7: 
  case  8: 
  case  9: MVIJF; MOVER; break;
  case 10: 
  case 11: 
  case 12: 
  case 13: 
  case 14: MTIEN; MOVER; break;
  case 15: 
  case 16:
  case 17: 
  case 18: 
  case 19: KWART; OVER; break;  
  case 20:
  case 21: 
  case 22: 
  case 23: 
  case 24: MTIEN; MVOOR; HALF; break;
  case 25: 
  case 26: 
  case 27: 
  case 28: 
  case 29: MVIJF; MVOOR; HALF; break;
  case 30:
  case 31: 
  case 32: 
  case 33: 
  case 34: HALF; break;
  case 35: 
  case 36: 
  case 37: 
  case 38: 
  case 39: MVIJF; MOVER; HALF; break; 
  case 40: 
  case 41:
  case 42: 
  case 43: 
  case 44: MTIEN; MOVER; HALF; break;
  case 45: 
  case 46:
  case 47: 
  case 48: 
  case 49: KWART; VOOR; break; 
  case 50: 
  case 51: 
  case 52: 
  case 53: 
  case 54: MTIEN; VOOR;  break;
  case 55: 
  case 56:
  case 57: 
  case 58:
  case 59: MVIJF; VOOR; break;
 }


 
 sayhour = Ihour;
 if (Iminute > 19 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                       #endif NLUV
 switch (sayhour)
 {
  case 13:
  case  1: EEN; break;
  case 14:
  case  2: TWEE; break;
  case 15:
  case  3: DRIE; break;
  case 16:
  case  4: VIER; break;
  case 17:
  case  5: VIJF; break;
  case 18:
  case  6: ZES; break;
  case 19:
  case  7: ZEVEN; break;
  case 20:
  case  8: ACHT; break;
  case 21:
  case  9: NEGEN; break;
  case 22:
  case 10: TIEN; break;
  case 23:
  case 11: ELF; break;
  case 0:
  case 12: TWAALF; break;
 } 

                                        #if (defined NL || defined UK)
 switch (Iminute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }
                                        #endif //NL  UK
                                                               
                                        #ifdef NLUV
switch (Iminute)
 {
  case  0: 
  case  1: 
  case  2:
  case  3:    
  case  4: UUR;  break; 
 }
                                        #endif NLUV
 if(Iminute == 0 && Isecond <9) 
 { 
  ToggleEdsoft = Isecond % 2;         // ToggleEdsoft bocomes 0 or 1 and turn on and off the first 8 seconds at minute 0 the Edsoft light on pin 24
  EDSOFT;
 } 
// Tekstprintln("");
 WriteLEDs();
}
//--------------------------------------------                                                //
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

//--------------------------------------------                                                //
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
//--------------------------------------------                                                //
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}

//--------------------------------------------                                                //
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
//--------------------------------------------                                                //
//  LED load the shiftbits in the LED display buffer
//--------------------------------------------
void ShiftInTime(const char TimeText[10], byte num , byte Displaynr, byte Shiftbit, byte ShiftIn)
{  
 num = num;                                      // Not used
 if (!SeeDCFsignalInDisplay)   
  Tekstprint(TimeText);                          // Print the time in text 
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
//--------------------------------------------                                                //
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void){  Display1=0;  Display2=0;  Display3=0; }
//--------------------------------------------                                                //
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
//--------------------------------------------                                                //
//  LED Turn On en Off the LED's
//--------------------------------------------
void Laatzien()
{ 
 WriteLEDs();  
 delay(Delaytime);  
 LedsOff();
}
//                                                                                            //
//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------                                                                                           
void DimLeds(byte print) 
{                                                                                                       
 int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;                      // Read lightsensor 0 - 1023 bits 
 Previous_LDR_read = LDR_read;
 OutPhotocell = (int)((Mem.LightReducer * sqrt(63.5*LDR_read))/100);                         // 0-255 bits.  Linear --> hyperbolic with sqrt
 MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
 MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
 BrightnessCalcFromLDR = (byte)constrain(OutPhotocell, Mem.LowerBrightness, 255);            // filter out of strange results 
 SumLDRreadshour += LDR_read;                                                                // For statistics LDR readings per hour
 NoofLDRreadshour++;
 if(print)
  {
   sprintf(sptext,"Sensor:%3d",(analogRead(PhotoCellPin)));                Tekstprint(sptext);
   sprintf(sptext," Min:%3d",MinPhotocell);                                Tekstprint(sptext);
   sprintf(sptext," Max:%3d",MaxPhotocell);                                Tekstprint(sptext);
   sprintf(sptext," Out:%3d",OutPhotocell);                                Tekstprint(sptext);
   sprintf(sptext,"=%2d%%",(int)(100 * BrightnessCalcFromLDR / 255));      Tekstprint(sptext);
                        #ifdef MOD_DS3231
   sprintf(sptext," Temp:%2.0dC ", (int)RTCklok.getTemperature());              Tekstprint(sptext);   //Get3231Temp()-2);   
                        #endif //MOD_DS3231        
   Print_tijd(); 
  }
 if(LEDsAreOff) 
     {
      BrightnessCalcFromLDR = 0;
      LedsOff();
      return;
     }
 analogWrite(PWMpin, BrightnessCalcFromLDR);                                                  // write PWM
}

//--------------------------------------------                                                //
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
 Display1=255;   Display2=255;   Display3=255; Laatzien();
 for (int n=255 ; n>=0; n--) { analogWrite(PWMpin, n); delay(2);}                             // The duty cycle: between 0 (lights off) and 255 (light full on).
 for (int n=0 ; n<=255; n++) { analogWrite(PWMpin, n); delay(2);}  
 LedsOff();
}
//--------------------------------------------                                                //
//  Blink UUR
//--------------------------------------------
void BlinkUUR(int NoofBlinks, int Delayms)
{
for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); UUR; Laatzien(); delay(Delayms);}
}
//--------------------------------------------                                                //
//  Blink HET IS WAS
//--------------------------------------------
void BlinkHETISWAS (int NoofBlinks, int Delayms)
{
for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); HET; IS; WAS; Laatzien(); delay(Delayms);} 
}
//--------------------------------------------                                                //
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(int amount)
{
 int value = Mem.LightReducer + amount;                                                      // Prevent byte overflow by making it an integer before adding
 Mem.LightReducer = min(value, 255);                                                         // May not be larger than 255
 sprintf(sptext,"Max brightness: %3d%%",Mem.LightReducer);
 Tekstprintln(sptext);
}
//--------------------------------------------                                                //
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(byte waarde)
{
 Mem.LowerBrightness = min(waarde, 255);                                                      // Range between 0 and 255
 sprintf(sptext,"Min brightness: %3d bits", Mem.LowerBrightness);
 Tekstprintln(sptext);
}

//--------------------------------------------                                                //
//  CLOCK Constrain a string with integers
// The value between the first and last character in a string is returned between the  low and up bounderies
//--------------------------------------------
int SConstrainInt(String s,byte first,byte last,int low,int up){return constrain(s.substring(first, last).toInt(), low, up);}
int SConstrainInt(String s,byte first,          int low,int up){return constrain(s.substring(first).toInt(), low, up);}
//--------------------------------------------
//  CLOCK Input from Bluetooth, HC-12 or Serial
//  rework the string to a command
//--------------------------------------------                                                //
void ReworkInputString(String InputString)
{
// Serial.print(InputString); Serial.println(InputString.length());
 InputString.trim();                                                                          // Remove trailing spaces
 if (InputString.length()>10) return;                                                         // If string is too long for some reason
 if (InputString.length()< 1) return;                                                         // If string is empty for some reason
 if (InputString[0] > 64 && InputString[0] <123)                                              // If the first charater is a letter
  {
  sprintf(sptext,"**** Length fault ****");                                                   // Default message placed in sptext
  switch (InputString[0]) 
   { 
    case 'A':
    case 'a':   
             Toggle_HetWasIsUit = 0; Toggle_HetWasIs = 1;                                     // All tekst displayed  
             sprintf(sptext,"All tekst displayed");
             break;
    case 'B':
    case 'b':    
             Toggle_HetWasIsUit = 1; Toggle_HetWasIs = 0;                                     // Het Is Was turned off
             sprintf(sptext,"Het Is Was off");
             break;
    case 'C':
    case 'c':    
            Toggle_HetWasIsUit = 2; Toggle_HetWasIs = 0;                                      // Het Is Was Off after 10 sec
            Play_Lights();                     
            sprintf(sptext,"Het Is Was Off after 10 sec");                          
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
    case 'E':
    case 'e':
             if(InputString.length() == 2)
               {
                for (int i=0;i<24;i++) Menumem[i] = 0;                                        // Reset LDR readings 
                sprintf(sptext,"Menu counts erased");   
               }
              if(InputString.length() == 3)
               {
                for (unsigned int i=0 ; i<EEPROM.length(); i++) { EEPROM.write(i, 0); }
                Tekstprintln("EEPROM erased"); 
                strcpy(sptext,""); 
                EEPROM.put(0,Mem);                                                            // Update EEPROM     
               }
               break;
                                   #ifdef DCF77MOD                
    case 'F':                                                                                 // LDR or DCF measurement/h in menu
    case 'f':
             if(InputString.length() == 1)
               {
                Mem.DCForLDR = 1-Mem.DCForLDR;
                sprintf(sptext,"%s", Mem.DCForLDR?"DCF":"LDR" ); 
               }
             EEPROM.put(0,Mem);                                                               // Update EEPROM         
             break;
    case 'G':                                                                                 // Toggle DCF Signal on Display
    case 'g':
            if (InputString.length() == 1)
              {
               SeeDCFsignalInDisplay = 1 - SeeDCFsignalInDisplay;
               sprintf(sptext,"SeeDCFsignal: %s",SeeDCFsignalInDisplay ? "Yes" : "No ");
               }
             break;
    case 'H':                                                                                 // Turn DCF receiver ON/OFF
    case 'h':
            if (InputString.length() == 1)
              {
               UseDCF = 1 - UseDCF;
               sprintf(sptext,"Use DCF-receiver: %s",UseDCF ? "Yes" : "No");
               }            
             break; 
                                  #endif //DCF77MOD               
//                                                                                            //
    case 'I':
    case 'i': 
            if (InputString.length() == 1)
            {  
             SWversion();
            }
            break;
    case 'L':                                                                                 // Lowest value for Brightness
    case 'l':
             if (InputString.length() < 5)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                sprintf(sptext,"Min brightness: %d bits",Mem.LowerBrightness);
                EEPROM.put(0,Mem);                                                            // Update EEPROM     
               }
             break;  
    case 'M':                                                                                 // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':
             Serial.println(InputString);
             if (InputString.length() < 5)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Max brightness: %d%%",Mem.LightReducer);
                EEPROM.put(0,Mem);                                                           // Update EEPROM     
               }
              break;
    case 'N':
    case 'n':
             if (InputString.length() == 1 )         Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
             if (InputString.length() == 5 )
              {
               Mem.TurnOffLEDsAtHH = SConstrainInt(InputString,1,3,0,23);
               Mem.TurnOnLEDsAtHH  = SConstrainInt(InputString,3,5,0,23); 
               sprintf(sptext,"LEDs OFF %2d:00-%2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
                EEPROM.put(0,Mem); 
              }
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                DimLeds(true);                    // Turn the LEDs on or off  
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );     
               }
              break; 
                                                                 
    case 'R':
    case 'r':
            if (InputString.length() == 1)
              {
               Reset();                                                                       // Reset all settings 
               Tekstprintln("\n Reset done"); 
              }
            break;
    case 'S':
    case 's':
             if (InputString.length() == 1)
               {   
                sprintf(sptext,"Zelftest");
                Selftest();                                                                //
                Displaytime();                                                                // Turn on the LEDs with proper time
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
    case 'W':
    case 'w':
             if (InputString.length() >1) break;   
             TestLDR = !TestLDR;                                                              // If TestLDR = 1 LDR reading is printed every second instead every 30s
             sprintf(sptext,"TestLDR: %s",TestLDR? "On" : "Off");
             break;
    case 'X':
    case 'x':    
             MilliSecondValue = 10;                                                           // Clock runs now 100 times faster. 10 msec per second 
              if (InputString.length() >1 && InputString.length() < 6 )
                MilliSecondValue = InputString.substring(1,5).toInt();                
             Demo = 1 - Demo;                                                                 // Toggle Demo mode
             sprintf(sptext,"Demo mode: %d MillisecondTime=%d",Demo,MilliSecondValue);
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
//                                                                                            //                 
    case 'R':
    case 'r':
            RDA5807_Report();            break;
    case 'S':
    case 's':
            RDA5807_ReadStatus();       break;
    case 'T':
    case 't':    
            RDA5807_RDS_Dump();        break;             
                     #endif //FMRADIO        
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

/*
//--------------------------------------------
// DS3231 Get temperature from module   OBSOLETE
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int  temp3231;
 
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    // Temp registers (11h-12h) get updated automatically every 64s
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
  else   {temp3231 = -273; }  
  return (temp3231);
}
*/
