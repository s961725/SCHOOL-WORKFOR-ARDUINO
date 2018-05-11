#include <SPI.h>
#include <SD.h>
#include <idDHT11.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "PMS5XXX.h"
#include <TinyGPS++.h>

#ifdef PMS5XXX_USE_SOFTWARE_SERIAL

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

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


void setup() {
 
  Serial.begin(9600);
  Serial2.begin(9600);
  
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
  if (!SD.begin(53)) {
    Serial.println("Fail!");
    return;
  }
   myFile.println("開始記錄偵測值");  // 繼續寫在檔案後面

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
   
    
     // n=(int32_t)RX_buffer[0]*1000000+(int32_t)RX_buffer[1]*10000+(int32_t)RX_buffer[2]*100+RX_buffer[3];
      //因為BYTE會升為int16_t這會使資料不正確必須讓它變為innt32_t，避免系統自動行動而編成者無法預測的動作
     // m=(int32_t)RX_buffer[4]*1000000+(int32_t)RX_buffer[5]*10000+(int32_t)RX_buffer[6]*100+RX_buffer[7];
       // myFile.print("經度:");
        //myFile.print(n);//經度
       // myFile.print(",");
       // myFile.print("緯度:");
       // myFile.print(m);//緯度
       // myFile.print(",");

          //  myFile.print("高度:");
          //  myFile.print((int32_t)RX_buffer[8]*100+(int32_t)RX_buffer[9],DEC);//高度
          //  myFile.print(",");
          //  myFile.print("氣壓:");
          //  myFile.print((int32_t)RX_buffer[10]*100+(int32_t)RX_buffer[11],DEC);//氣壓
          //  myFile.println(",");
           // myFile.print("溫度:");
           // myFile.print((int32_t)RX_buffer[12],DEC);//溫度
           // myFile.print(",");
smartDelay(1000);
 if (gps.location.isUpdated())
 {
Y = gps.date.year(); // Year (2000+) (u16)  `
    M = gps.date.month(); // Month (1-12) (u8)

    D = gps.date.day(); // Day (1-31) (u8)
     
    H = gps.time.hour()+8; // Hour (0-23) (u8)
    T = gps.time.minute(); // Minute (0-59) (u8)
    S = gps.time.second(); // Second (0-59) (u8)
    E = gps.location.lat();
    N = gps.location.lng();
    I = gps.altitude.meters(); 
  }
else
myFile.println("GPS FAIL ");//沒有括弧所以不會繼續ELSE下去

    myFile.print("DATE=");
    myFile.print(Y); // Year (2000+) (u16)
    myFile.print(F("/"));
    myFile.print(M); // Month (1-12) (u8)
    myFile.print(F("/"));
    myFile.print(D); // Day (1-31) (u8)
     
    myFile.print(" TIME=");
    myFile.print(H); // Hour (0-23) (u8)
    myFile.print(F(":"));
    myFile.print(T); // Minute (0-59) (u8)
    myFile.print(F(":"));
    myFile.print(S); // Second (0-59) (u8)
    myFile.print(" LAT="); 
    myFile.print(E, 6);
    myFile.print(" LNG="); 
    myFile.print(N, 6);
    myFile.print(" HIGH=");
    myFile.print(I);
    myFile.print(" ");
            myFile.print(" O3:");
            myFile.print(sensorValue, DEC);
            myFile.print(",");
            myFile.print("PM1:");
            myFile.print(pm2_5.pm1);//PM1
            myFile.print(",");
            myFile.print("PM2.5:");
            myFile.print(pm2_5.pm2d5);//PM2.5
            myFile.print(",");
            myFile.print("PM10:");
            myFile.print(pm2_5.pm10);//PM10
            myFile.print(",");
            myFile.print("AQI=");
            myFile.print(pm2_5.getAQI(pm2_5.pm2d5));
            myFile.print(",");
            myFile.print("RH (%): ");
            myFile.print(DHT11.getHumidity(), 2);
            myFile.print(",");
            myFile.print("T (C): ");
            myFile.print(DHT11.getCelsius(), 2);
            myFile.print(",");
            myFile.print(F("P (hPa) = "));
            myFile.print(bmp.readPressure()/100);
            myFile.print(",");
            myFile.print(F("H (m) = "));
            myFile.print(bmp.readAltitude(p1)); // this should be adjusted to your local forcase
            myFile.print(" ");
            //delay(1000);
            myFile.print("running_time=");     
            myFile.println(millis()/1000);
    
    myFile.close();                               // 關閉檔案
    Serial.println("Completed!");
  } else {
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



