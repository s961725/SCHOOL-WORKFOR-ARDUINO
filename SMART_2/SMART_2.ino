#include <SPI.h>
#include <SD.h>
#include <idDHT11.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "PMS5XXX.h"
#include <TinyGPS++.h>

#ifdef PMS5XXX_USE_SOFTWARE_SERIAL


TinyGPSPlus gps;

/* SD card attached to SPI bus as follows:
 ** MOSI - pin 11/51
 ** MISO - pin 12/50
 ** CLK - pin 13/52
 ** CS - pin 4/53*/
File myFile;
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 53;

//PMS5003
SoftwareSerial mySerial(10, 11); // MEAG TX:10, RX:11 UNO 6,7
PMS5XXX pm2_5(mySerial);
#else
PMS5XXX pm2_5(Serial3);
#endif

int sensorValue;

Adafruit_BMP280 bmp; // I2C

//DHT11
int idDHT11pin = 2; //Digital pin for comunications
int idDHT11intNumber = 0; //interrupt number (must be the one that use the previus defined pin (see table above)
//declaration
void dht11_wrapper(); // must be declared before the lib initialization
// Lib instantiate
idDHT11 DHT11(idDHT11pin,idDHT11intNumber,dht11_wrapper);

double p1 ;

double E,N,I = 0;//GPS 資料
int Y,M,D,H,T,S =0;

int ledPin=13;
void setup() {
 
  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(ledPin, OUTPUT);
 // Serial.println("idDHT11 Example program");
  //Serial.print("LIB version: ");
 // Serial.println(IDDHT11LIB_VERSION);
  // Serial.print("code update OK");
  
 // Serial.println(F("BMP280 test"));
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    //while (1);
  }

   delay(2000);
   p1 = bmp.readPressure()/100;
  Serial.println(p1);
  
//SD寫入檔案
  while (!Serial) {    // 等待串列埠連線
  }

  Serial.print("\nWaiting for SD card ready...");
   while (!SD.begin(53)) {
    Serial.println("Fail!");
    digitalWrite(ledPin, HIGH);
    return;
  }
  digitalWrite(ledPin, LOW);
 myFile = SD.open("card.txt", FILE_WRITE);
 myFile.println("   Date      Time      lat         lng     high(GPS)  O3  PM1 PM2.5 PM10 AQI RH     T      P(hpa)   high(p) runningtime");
 myFile.close(); 
}


void dht11_wrapper() {
  DHT11.isrCallback();
}


void loop() {
pm2_5.read();
int result = DHT11.acquireAndWait();
  switch (result)
  {
  case IDDHTLIB_OK: 
    Serial.println("OK"); 
    break;
  case IDDHTLIB_ERROR_CHECKSUM: 
    Serial.println("Error\n\r\tChecksum error"); 
    break;
  case IDDHTLIB_ERROR_ISR_TIMEOUT: 
    Serial.println("Error\n\r\tISR time out error"); 
    break;
  case IDDHTLIB_ERROR_RESPONSE_TIMEOUT: 
    Serial.println("Error\n\r\tResponse time out error"); 
    break;
  case IDDHTLIB_ERROR_DATA_TIMEOUT: 
    Serial.println("Error\n\r\tData time out error"); 
    break;
  case IDDHTLIB_ERROR_ACQUIRING: 
    Serial.println("Error\n\r\tAcquiring"); 
    break;
  case IDDHTLIB_ERROR_DELTA: 
    Serial.println("Error\n\r\tDelta time to small"); 
    break;
  case IDDHTLIB_ERROR_NOTSTARTED: 
    Serial.println("Error\n\r\tNot started"); 
    break;
  default: 
    Serial.println("Unknown error"); 
    break;
  }

  
  
  //SD卡 迴圈紀錄
    Serial.println("Success!");
  myFile = SD.open("card.txt", FILE_WRITE);       // 開啟檔案，一次僅能開啟一個檔案
  
  if (myFile) {                                   // 假使檔案開啟正常
    Serial.println("Write to card.txt...");
   digitalWrite(ledPin, LOW);
    
    
smartDelay(1000);
 if (gps.location.isUpdated())
 {
Y = gps.date.year(); // Year (2000+) (u16)  `
    M = gps.date.month(); // Month (1-12) (u8)

    D = gps.date.day(); // Day (1-31) (u8)
     
    H = gps.time.hour()+8; // Hour (0-23) (u8)
    T = gps.time.minute(); // Minute (0-59) (u8)
    S = gps.time.second(); // Second (0-59) (u8)
  
  }

    char DAY[10];
    char time1[10];
    sprintf(DAY,"%04d/%02d/%02d ",Y,M,D);
    myFile.print(DAY); // Year (2000+) (u16)
    sprintf(time1,"%02d:%02d:%02d ",H,T,S);
    myFile.print(time1);
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    myFile.print("    ");
   char data[5] ;
   sprintf(data,"%03d ",sensorValue) ;       
            myFile.print(data);
           
            myFile.print(pm2_5.pm1);//PM1
            myFile.print("  ");
            
            myFile.print(pm2_5.pm2d5);//PM2.5
            myFile.print("    ");
           
            myFile.print(pm2_5.pm10);//PM10
            myFile.print("   ");
           
            myFile.print(pm2_5.getAQI(pm2_5.pm2d5));
            myFile.print("  ");
           
            myFile.print(DHT11.getHumidity());
            myFile.print("  ");
           
            myFile.print(DHT11.getCelsius());
            myFile.print("  ");
            
            myFile.print(bmp.readPressure()/100);
            myFile.print("  ");
           
            myFile.print(bmp.readAltitude(p1)); // this should be adjusted to your local forcase
            myFile.print("       ");
           
            myFile.println(millis()/1000);
    
    myFile.close();                               // 關閉檔案
    Serial.println("Completed!");
  } else {
    digitalWrite(ledPin, HIGH);
    Serial.println("\n open file error ");    // 無法開啟時顯示錯誤
  }
 // Serial.println("================= Finished =====================");*/
  sensorValue = analogRead(0);


      
           Serial.print("O3:");
           Serial.print(sensorValue, DEC);
          // delay(100);
           Serial.print(",");
           Serial.print("PM1:");
           Serial.print(pm2_5.pm1);//PM1
           Serial.print(",");
           Serial.print("PM2.5:");
           Serial.print(pm2_5.pm2d5);//PM2.5
           Serial.print(",");
           Serial.print("PM10:");
           Serial.print(pm2_5.pm10);//PM10
           Serial.print(",");
           Serial.print("AQI=");
           Serial.print(pm2_5.getAQI(pm2_5.pm2d5));
           Serial.print(",");
           Serial.print("RH (%): ");
           Serial.print(DHT11.getHumidity(), 2);
           Serial.print(",");
           Serial.print("T (C): ");
           Serial.print(DHT11.getCelsius(), 2);
           Serial.print(",");
           Serial.print(F("P (hPa) = "));
           Serial.print(bmp.readPressure()/100);
           Serial.print(",");
           Serial.print(F("H (m) = "));
           Serial.print(bmp.readAltitude(p1)); // this should be adjusted to your local forcase
           Serial.print(" ");
           Serial.print(H);
           Serial.println(" ");
          

           
          
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      myFile.print('*');
    myFile.print(' ');
  }
  else
  {
    myFile.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      myFile.print(' ');
  }
  smartDelay(0);
}

