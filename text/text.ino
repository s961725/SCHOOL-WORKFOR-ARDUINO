#include <SPI.h>
#include <SD.h>
#include <idDHT11.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "PMS5XXX.h"
#include <TinyGPS++.h>
long m;
long n;
long p[13];
int pm1;
int pm2d5;
int pm10;

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



int MIN,SEC=0;
int sensorValue;

double E,N,I = 0;//GPS 資料
int Y,M,D,H,T,S =0;


void setup() {
 
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);

}
void loop() {
smartDelay(1000);
    Serial.println("Write to card.txt...");
    
 while(Serial3.available() > 0)
    gps.encode(Serial3.read());
      if (gps.date.isValid())
  Serial.print(gps.date.year());
  PM5();
  Serial.println(pm2d5);//PM1
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
}


void PM5()
{

  byte b3[32] = {0};
  int i = 0;
  int j = 0;
  int k = 0;
  while (Serial2.available() > 0) {
    b3[i] = Serial2.read();
    i++;
    if (i > 1 && (b3[0] != 0x42 || b3[1] != 0x4d))
      i = 0;
    if (i >= 32)break;
  }
  for (j = 0; j < 13; j++) {
    p[j] = b3[2 * j + 4] * 0x100 + b3[2 * j + 5];
  }
  for (j = 0; j < 30; j++) {
    k += b3[j];
  }
  j = b3[30] * 0x100 + b3[31];
  if (j > 0 && j == k) {
    pm1 = p[3];
    pm2d5 = p[4];
    pm10 = p[5];

  }
}

