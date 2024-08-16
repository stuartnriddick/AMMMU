// Program  Logger_B

// The data from the sensor is retrieved every
// 2 seconds and printed out.
// Time is read and SD card initialised
// Data are logged to SD file LByymmdd.txt
// Data logged are TGS  and 3.3 volt reference 16 bit 

#include <stdlib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
#include <SoftwareSerial.h>
const int chipSelect = 10;

char ID[2];
int a;
int b;
int c;
int d;
int Port;

int count_in;
int led = 13;
int Dummy;
int yy;
int Y20;
int mm;
int dd;
int h;
int m;
int s;
int i;
int old_sec;
int Flag =0;

RTC_DS1307 rtc;
char filename[] = "00000000.txt";

void setup() {
  Serial.begin(9600);
   Wire.begin();
  rtc.begin();
  ads.begin();

  pinMode(A1,INPUT);
 
  pinMode(led, OUTPUT);  // LED
  pinMode(10, OUTPUT);   // SD
  
   Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                               
 
   ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
 
 
 
  delay(1000); 
  delay(10);
  rtc.begin();
   if (! rtc.begin()) {
   //   Serial.println("Couldn't find RTC");
      digitalWrite(led, HIGH);   // LED ON indicating RTC problem
    while (1);
  }  
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
// see if theSD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
   //  Serial.println("Card failed, or not present");
    
     digitalWrite(led, HIGH);   // LED ON indicating SD problem
    // don't do anything more:
    while (1);
  }
 // Serial.println("RTC Setup");
 // Serial.println("SD Setup");
  
 // Serial.println("Collecting Data");
  delay(1000);
  DateTime now =rtc.now();
 

}

void loop() {

   //  Data Sample Loop
  
   String dataString = "";  // Clear Output string
   
  //  Read the time
  DateTime now =rtc.now();
  
  yy=now.year();
  
  Y20 = yy-2000;
   dataString += yy;
   dataString += " ";
   mm=now.month();
   if (mm < 10) { dataString += '0';}
   dataString += mm;
   dataString += " ";
   dd = now.day();
   if (dd < 10) { dataString += '0';}
   dataString += dd;
   dataString += " ";
   h = now.hour();
   if (h < 10) { dataString += '0';}
   dataString += h;
   dataString += " ";
   m=now.minute();
   if (m < 10) { dataString += '0';}
   dataString += m;
   dataString += " ";
   s=now.second();
   if (s < 10) { dataString += '0';}
   dataString += s;
   dataString += " ";
   
 
   //dataString += " "; 

   //  Read TGS  and store
      
    //Read data and store it to variables hum and temp
    
     int16_t adc0, adc1;
     float volts0, volts1;
  
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  
    dataString += volts0;
    dataString += " ";
    dataString += volts1;
    dataString += " "; 
    
// Write data to LByymmdd.txt

   getFilename(filename);
   File dataFile = SD.open(filename, FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    Serial.println(dataString);
    dataFile.println(dataString);
    dataFile.close();
   // MQSerial.print(dataString);
   // MQSerial.print("\n\r"); 
     SMP_LED();  // flash sample LED
     SMP_LED(); 
    // print to the serial port and Bluetooth too:
    //   Serial.println(dataString);
    //    MONSerial.println(dataString);
 
  }   
    delay(2000); 
}    
    
void SMP_LED()
{
 digitalWrite(led, HIGH);
 delay(50);
// Serial.println("Flashing LED");
 digitalWrite(led, LOW); 
 delay(50);
}

void getFilename(char *filename) {
  int nyy;
   filename[0] = 'L';
   filename[1] = 'B';
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
//  Serial.println(filename);
    return;
  }
  
 
