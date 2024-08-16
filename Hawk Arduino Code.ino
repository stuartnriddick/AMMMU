/*
  Snffer_V3_9600_LCD_SD

  // connection shown for an LCD 1602 and an Arduino Uno/Nano
  //                              
  //                              |----- 4 bit ----|
  // Lcd           E   RS   RW    D7   D6   D5    D4     
  // Arduino pin   2    3    4    5    6    7     8      
  // Port          D2   D3   D4   D5   D6   D7    Pin 8    

    16.05.2023 Ver 0_10 clean up mode changes between 4 bit and 8 bit
*/

const bool DEBUG_MODE = false ;   // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#include "Fifo.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>
// Select which LCD Arduino uses
LiquidCrystal_I2C lcd(0x27, 20, 4);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

Fifo queue( 50 ) ;
volatile uint16_t errorCount = 0 ;  // indicates Queue overflows if the queue is not cleared fast enough in the loop()
int led = 13;
int yy;
int Y20;
int mm;
int dd;
int h;
int m;
int s;
RTC_DS1307 rtc;

char sBuff[80] ; // for sprintf
String datout;
uint16_t expandedBuf = 0 ;
const int chipSelect = 10;

char filename[] = "00000000.TXT";
String dataout;


char * dumpBits16( uint16_t inB ) {
  // formatted output of uint16_t as bit pattern
  static char f16[ 17 ] = {0} ;
  for ( uint8_t i = 0 ; i < 16 ; i++ ) {
    f16[ 15 - i ] = bitRead( inB, i ) == true ? '1' : '0' ;
  }
  return &f16[0] ;
}


void readLcdBus1() {
  // ports D and pin 8 are put on queue when LCD enable is falling.
  // called from external interrupt
 
  uint16_t buf = PIND ;
   buf <<= 8 ;
   int inB = digitalRead(8);
   uint8_t bufB = bufB + inB;
   buf |= bufB;
  if ( ! queue.isfull() ) queue.push( buf ) ;
  else {
    errorCount ++ ; // queue overflow
  }
}

bool processQueue() {

  // takes an item off the queue, analyses it, reformats it and,
  // in the case of a 4 bit data stream, consolidates it
  // to form an 8bit packet. It returns true if the data item in 
  // global expandedBuf is
  // complete (ie 8bits have been accumulated) and false otherwise.

  static bool mode8bit = true ;
  static bool highOrder4bit = true ;  // only valid in 4 bit mode

  bool expandedBufIsValid = false ;  // function return value


  uint16_t qItem = queue.pop_locked() ;

  // bulk read of ports
  uint8_t pinD = ( qItem >> 8 ) & 0xFF ;
  uint8_t pinB = ( qItem  ) & 0xFF ;

  // restructure qItem for simpler processing
  // could be optimised
  // 8 low order bits of pinBuf are display pins D7 to D0

  uint16_t pinBuf = 0 ;

  bitWrite( pinBuf, 9  , bitRead( pinD, 3 ) ) ;   // RS
  bitWrite( pinBuf, 8  , bitRead( pinD, 4 ) ) ;   // RW
  bitWrite( pinBuf, 7  , bitRead( pinD, 5 ) ) ;   // D7
  bitWrite( pinBuf, 6  , bitRead( pinD, 6 ) ) ;   // D6
  bitWrite( pinBuf, 5  , bitRead( pinD, 7 ) ) ;   // D5
  bitWrite( pinBuf, 4  , bitRead( pinB, 0 ) ) ;   // D4
  bitWrite( pinBuf, 3  , bitRead( pinB, 1 ) ) ;   // D3
  bitWrite( pinBuf, 2  , bitRead( pinB, 2 ) ) ;   // D2
  bitWrite( pinBuf, 1  , bitRead( pinB, 3 ) ) ;   // D1
  bitWrite( pinBuf, 0  , bitRead( pinB, 4 ) ) ;   // D0

  if ( DEBUG_MODE ) {
    Serial.println( F("      RRDDDDDDDD") ) ;
    Serial.println( F("......SW76543210") ) ;
    Serial.println( dumpBits16( pinBuf ) ) ;
    Serial.println() ;
  }

  bool functionSet8bit = false ;
  bool functionSet4bit = false ;

  // 4 bit function set recognised only in 8 bit mode
  functionSet8bit =  !( ( pinBuf & 0b0000001011110000 ) ^ 0b0000000000110000 ) ;  // RS = 0 and D7..D4 = 0011
  functionSet4bit =  !( ( pinBuf & 0b0000001011110000 ) ^ 0b0000000000100000 ) ;  // RS = 0 and D7..D4 = 0001

  // here we are interested in two possible situations
  // 1, We are in 8 bit mode and a Function Set 4 bit is encountered in the high order nibble or
  // 2. We are in 4 bit mode and a Function Set 8 bit is encountered in the high order nibble.
  // In the first case we change the mode to 4 bit and synchronise to the next nibble which will be a high order nibble
  // and, according to the protocol, is another Function Set 4 bit command. The corresponding low order nibble will be a screen
  // lines parameter and font size parameter. 
  // In the second case we enter 8 bit mode then treat this as normal because nibble order synchronisation is not relevant.

  if ( functionSet8bit && mode8bit ) {
    if ( DEBUG_MODE ) Serial.println("8 bit initialisation detected" ) ;
    expandedBuf = pinBuf ;
    expandedBufIsValid = true ;
  }
  else if ( functionSet4bit && mode8bit ) {
    if ( DEBUG_MODE ) Serial.println("4 bit initialisation detected" ) ;
    mode8bit = false ;
    highOrder4bit = true ;  // resync
    expandedBuf = pinBuf ;
    expandedBufIsValid = true ;
  }
  else if ( ! mode8bit ) {
    // 4 bit mode
    if (  highOrder4bit ) {
      // part 1 nibble
      if ( functionSet8bit ) {
        // transition to 8 bit
        if ( DEBUG_MODE ) Serial.println("4 bit to 8 bit transition" ) ;
        mode8bit = true ;
        expandedBuf = pinBuf ;   
        expandedBufIsValid = true ;
      }
      else {
        expandedBuf = pinBuf ;    
        expandedBufIsValid = false ;
        highOrder4bit = false ;
      }
    }
    else {
      // part 2 nibble
      // join this with the first nibble
      expandedBuf = ( expandedBuf & 0b1111111111110000 ) | (( pinBuf & 0b0000000011110000 ) >> 4 ) ;
      expandedBufIsValid = true ;
      highOrder4bit = true ;
    }
  }
  else if ( mode8bit ) {
    expandedBuf = pinBuf ;
    expandedBufIsValid = true ;
  }

  return expandedBufIsValid ;
}


void setup()
{
   lcd.init();
   lcd.backlight();
  Serial.begin(9600);
  Serial.println( "Starting Sniffer SD " ) ;
  lcd.setCursor(0,0);
  lcd.print("Starting Sniffer SD");
   pinMode(8,INPUT);
  pinMode(2, INPUT_PULLUP ) ;
   pinMode(led, OUTPUT);  // LED
  pinMode(10, OUTPUT);   // SD
   delay(2000);
  lcd.clear();
   Wire.begin();
    // see if theSD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
     Serial.print("SD Not Recognised ");
     lcd.setCursor(0,0);
     lcd.print("SD Not Recognised ");
     digitalWrite(led, HIGH);   // LED ON indicating SD problem
    // don't do anything more:
    while (1);
     }
  Serial.println("SD Setup");
  lcd.println("SD Setup");
   if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
    lcd.setCursor(0,0);
   lcd.print("Couldn't find RTC");
   while (1);
   }
    lcd.setCursor(0,0);
    +lcd.print("RTC/SD Setup");
    Serial.println("RTC is set to PC time");
    delay(2000);
// Set rtc to PC time
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    lcd.setCursor(0,0);
     lcd.print("Start Sniffer V5 ");
   delay(1000);
  // LCD  "E" (arduino pin 2) is the Enable and we use the falling edge
  attachInterrupt( digitalPinToInterrupt( 2 ) , readLcdBus1 , FALLING ) ;
}


void loop()
{
  static uint32_t lastLoopAtMs = 0 ;

  if ( millis() - lastLoopAtMs > 3000 ) {
    // print out errors occasionally (if any )
    if ( errorCount != 0 ) {
      Serial.print( F("Q overflow error count= " )) ;
      Serial.println( errorCount ) ;
    }
    lastLoopAtMs = millis() ;
  }

  if ( ! queue.isempty_locked() ) {
    if ( processQueue() ) {
      // here if expandedBuf is ready to be read

      static char charBuffer[80 ] = {' '} ;      // for presenting a stream of characters obtained from the LCD data bus.
      static uint8_t charBufferIndex = 0 ;
      bool isChar = ( ( 0b00000011 & ( expandedBuf >> 8 )) == 0b10 )  ; // RS == 1 and R/W = 0 (crude test for a character)

      if ( isChar ) {
        char prCh ;
        uint8_t expBuf = expandedBuf & 0xFF  ;
        ( expBuf >= 0x20 && expBuf < 0x7F ) ? prCh = expBuf : prCh = '.' ;   // '.' => unrecognised
        snprintf(sBuff, 50, "control  data 0x%02X  0x%02X  \'%c\'", (expandedBuf >> 8) & 0xFF , expandedBuf & 0xFF , prCh  );
        if ( (uint16_t) (charBufferIndex + 1) < sizeof( charBuffer ) ) {
           dataout  += prCh;
          charBuffer[ charBufferIndex++ ] =  prCh ;   // just the data part
          charBuffer[ charBufferIndex ] = 0 ;  // clean ahead
        }
      }
      else {
        snprintf(sBuff, sizeof( sBuff) - 1, "control  data 0x%02X  0x%02X ", (expandedBuf >> 8) & 0xFF , expandedBuf & 0xFF );
        if ( charBufferIndex > 0 ) {
          // dump charBuffer
          if(dataout[1] != 0x50){  // Check for'P' 
          //Serial.println("This is the correct string");
           String dataString = "";  // Clear Output string 
           String timeString = "";     
            //  Read the time
           DateTime now = rtc.now();
            yy=now.year();
            Y20 = yy-2000;
            dataString += yy;
            dataString += " ";
            timeString += yy;
            timeString += " ";
            mm=now.month();
            if (mm < 10) { dataString += '0';}
            dataString += mm;
            dataString += " ";
            timeString += mm;
            timeString += " ";
            dd = now.day();
            if (dd < 10) { 
              dataString += '0';
              timeString += '0';}
            dataString += dd;
            dataString += " ";
            timeString += dd;
            timeString += " ";
            h = now.hour();
            if (h < 10) { 
              dataString += '0';
              timeString += '0';}
            dataString += h;
            dataString += " ";
            timeString += h;
            timeString += " ";
            m=now.minute();
            if (m < 10) { 
              dataString += '0';
              timeString += '0';}
            dataString += m;
            dataString += " ";
            timeString += m;
            timeString += " ";
            s=now.second();
            if (s < 10) { 
              dataString += '0';
              timeString += '0';}
            dataString += s;
            dataString += " ";
            timeString += s;
            timeString += " ";
          //  Load data
            dataString += charBuffer;
           
            
 // Write data to IIyymmdd.txt
         getFilename(filename);
        File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
        if (dataFile) {
          dataFile.println(dataString);
         dataFile.close(); 
         SMP_LED();  // flash sample LED
         SMP_LED(); 
    // print to the serial port and Bluetooth too:
          Serial.println(dataString);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(timeString);
          lcd.setCursor(0,1);
          lcd.println(charBuffer);            
                 }
          }
           dataout = "";
          charBufferIndex = 0 ;
        }
      }
     // Serial.println( sBuff ) ;
    }
    
  }

} // loop()
void SMP_LED()
{
 digitalWrite(led, HIGH);
 delay(50);
// Serial.println("Flashing LED");
 digitalWrite(led, LOW); 
 delay(50);
}
//  Read Sensor Ident
   
void getFilename(char *filename) {
   int nyy;
   filename[0] = 'M';
   filename[1] = 'M';
   nyy = yy - 2000;
   filename[2] = (nyy)/10 + '0';
   filename[3] = nyy%10 + '0';
   filename[4] = mm/10 + '0';
   filename[5] = mm%10 + '0';
   filename[6] = dd/10 + '0';
   filename[7] = dd%10 + '0';
   
   filename[8] = '.';
   filename[9] = 'T';
   filename[10] = 'X';
   filename[11] = 'T';
   return;
  }
  
