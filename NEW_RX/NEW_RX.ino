 #include <ELECHOUSE_CC1101.h>
 #include <SoftwareSerial.h>
 SoftwareSerial mySerial(4,5);
 int pm1;
    int pm2d5;
    int pm10;
    long p[13];
    long n;
     long m;
     long temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,temp9;
     long h,H,t,T,p0,P0,p1,P1,p2,P2,P10,p10;
 void setup()
{
  Serial.begin(9600);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.SetReceive();
  mySerial.begin(9600);
}

byte RX_buffer[20]={0};
byte size,i,flag;

void loop()
{
  if(ELECHOUSE_cc1101.CheckReceiveFlag())
  {
    size=ELECHOUSE_cc1101.ReceiveData(RX_buffer);

      n=(int32_t)RX_buffer[0]*1000000+(int32_t)RX_buffer[1]*10000+(int32_t)RX_buffer[2]*100+RX_buffer[3];
      //因為BYTE會升為int16_t這會使資料不正確必須讓它變為innt32_t，避免系統自動行動而編成者無法預測的動作
      m=(int32_t)RX_buffer[4]*1000000+(int32_t)RX_buffer[5]*10000+(int32_t)RX_buffer[6]*100+RX_buffer[7];
      long N=n;
      long M=m;
      if (n>30000000){
        N=temp1;
        }
          if (m>300000000){
        M=temp2;
        }
        temp1=n;
        temp2=m;
      Serial.print(N);//經度
        Serial.print(",");
      Serial.print(M);//緯度
        Serial.print(",");

        
        if (RX_buffer[9]>100)//去除數值為負時的情況255
        {
          RX_buffer[9]=0;
          }
          h=(int32_t)RX_buffer[8]*100+(int32_t)RX_buffer[9];
          p0=(int32_t)RX_buffer[10]*100+(int32_t)RX_buffer[11];
          t=(int32_t)RX_buffer[12];
          p1=(int32_t)RX_buffer[13]*100+(int32_t)RX_buffer[14];
          p2=(int32_t)RX_buffer[15]*100+(int32_t)RX_buffer[16];
          p10=(int32_t)RX_buffer[17]*100+(int32_t)RX_buffer[18];
          
           H=h;
           P0=p0;
           T=t;
           P1=p1;
           P2=p2;
           P10=p10;
           
         if (h>300){
        H=temp3;
        }
        if (p0>30000){
        P0=temp4;
        }
        if (t>100){
        T=temp5;
        }
        if (p1>900){
        P1=temp6;
        }
        if (p2>900){
        P2=temp7;
        }
        if (p10>900){
        P10=temp8;
        }
          temp3=h;
          temp4=p0;
          temp5=t;
          temp6=p1;
          temp7=p2;
          temp8=p10;
        
          
            Serial.print(H,DEC);//高度
            Serial.print(",");
            Serial.print(P0,DEC);//氣壓
            Serial.print(",");
            Serial.print(T,DEC);//溫度
            Serial.print(",");
            Serial.print(P1,DEC);//PM1
            Serial.print(",");
            Serial.print(P2,DEC);//PM2.5
            Serial.print(",");
            Serial.print(P10,DEC);//PM10
            Serial.print(",");
             PM5();
          Serial.print(pm1);
          Serial.print(",");
          Serial.print(pm2d5);
          Serial.print(",");
          Serial.print(pm10);
     
    

    Serial.println("");
    ELECHOUSE_cc1101.SetReceive();
  }
}
void PM5()
{
 
  byte b3[32] = {0};
  int i = 0;
  int j = 0;
  int k = 0;
 mySerial.begin(9600);
  while(mySerial.available() > 0) {
    b3[i] =mySerial.read();
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

