
 #include <ELECHOUSE_CC1101.h>


#include <Wire.h>
#include <TinyGPS++.h>

#include <SFE_BMP180.h>

SFE_BMP180 pressure;



double baseline; // baseline pressure

long m;
long n;
long p[13];
    int pm1;
    int pm2d5;
    int pm10;
double T,P,p0,a;

#define size 20
TinyGPSPlus gps;


byte TX_buffer[size]={0};

void setup()
{
  Serial.begin(9600);
  ELECHOUSE_cc1101.Init();
  
       Serial2.begin(9600);
       Serial3.begin(9600);
       if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while(1); // Pause forever.
  }

  
  baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");  

}

void loop()
{
 
 if(Serial3.available() > 0){
    if (gps.encode(Serial3. read())){
      if (gps.location.isValid()){
 double a,P;
 P = getPressure();
 a = pressure.altitude(P,baseline);
 long p=P;
 long H=a;
     //傳出資料整理
     m=gps.location.lat()*1000000;
     n=gps.location.lng()*1000000;
     Serial.println(m);
    PM5();
     TX_buffer[3]=m%100;
     TX_buffer[2]=(m/100)%100;
     TX_buffer[1]=(m/10000)%100;
     TX_buffer[0]=(m/1000000)%1000;
     TX_buffer[7]=n%100;
     TX_buffer[6]=(n/100)%100;
     TX_buffer[5]=(n/10000)%100;
     TX_buffer[4]=(n/1000000)%1000;

      TX_buffer[8]=(H/100)%100;
      TX_buffer[9]=H%100;
      TX_buffer[10]=(p/100)%100;
      TX_buffer[11]=p%100;
      TX_buffer[12]=T;
     
      TX_buffer[13]=(pm1/100)%100;
      TX_buffer[14]=pm1%100;
      TX_buffer[15]=(pm2d5/100)%100;
      TX_buffer[16]=pm2d5%100;
      TX_buffer[17]=(pm10/100)%100;
      TX_buffer[18]=pm10%100;

 delay(1000);
    
   
   ELECHOUSE_cc1101.SendData(TX_buffer,size);
  delay(1);
       }
     }
   }
}

double getPressure()
{
  char status;

  status = pressure.startTemperature();
  if (status != 0)
  {


    delay(status);

    status = pressure.getTemperature(T);
    if (status != 0)
    {
    

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void PM5()
{
 
  byte b3[32] = {0};
  int i = 0;
  int j = 0;
  int k = 0;
  while(Serial2.available() > 0) {
    b3[i] =Serial2.read();
    i++;
    if (i > 1 && (b3[0] != 0x42 || b3[1] != 0x4d))
      i = 0;
      if(i>=32)break;
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

