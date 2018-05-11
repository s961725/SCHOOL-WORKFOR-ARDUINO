#define BME280_ADDRESS 0x76

double p0;
unsigned long int hum_raw, temp_raw, pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

#include <Wire.h>
#include <ELECHOUSE_CC1101.h>
#include <TinyGPS++.h>

#define size 20
TinyGPSPlus gps;
byte TX_buffer[size] = {0};

long m;
long n;
long p[13];
int pm1;
int pm2d5;
int pm10;

void setup() {

  Serial3.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  Serial.begin(9600);
  
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off
  uint8_t spi3w_en = 0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;

  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);
  readTrim();
   ELECHOUSE_cc1101.Init();
  for (int i = 0; i < 3; i++)
  {
    double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
    signed long int temp_cal;
    unsigned long int press_cal, hum_cal;
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    readData();
    press_cal = calibration_P(pres_raw);
    p0 = (double)press_cal / 100.0;
    Serial.println(p0);
   
  }
}

void loop() {
  if (Serial3.available() > 0) {
    if (gps.encode(Serial3. read())) {
      if (gps.location.isValid()) {

        m = gps.location.lat() * 1000000;
        n = gps.location.lng() * 1000000;
        Serial.print(m);
        
        double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
        signed long int temp_cal;
        unsigned long int press_cal, hum_cal;
        readData();
        temp_cal = calibration_T(temp_raw);
        press_cal = calibration_P(pres_raw);
        hum_cal = calibration_H(hum_raw);
        int T = (double)temp_cal / 100.0;
        double p = (double)press_cal / 100.0;
        int P9 = p;
       Serial.println(p);
        int H = 44330.0 * (1 - pow(p / p0, 1 / 5.255));

        PM5();

        //無線傳輸資料轉換
        //取GPS 資料
        TX_buffer[3] = m % 100;
        TX_buffer[2] = (m / 100) % 100;
        TX_buffer[1] = (m / 10000) % 100;
        TX_buffer[0] = (m / 1000000) % 1000;
        TX_buffer[7] = n % 100;
        TX_buffer[6] = (n / 100) % 100;
        TX_buffer[5] = (n / 10000) % 100;
        TX_buffer[4] = (n / 1000000) % 1000;
        //取得BMP28資料
        TX_buffer[8] = (H / 100) % 100;
        TX_buffer[9] = H % 100;
        TX_buffer[10] = (P9 / 100) % 100;
        TX_buffer[11] = P9 % 100;
        TX_buffer[12] = T;
        //取PMS5003資料
        TX_buffer[13] = (pm1 / 100) % 100;
        TX_buffer[14] = pm1 % 100;
        TX_buffer[15] = (pm2d5 / 100) % 100;
        TX_buffer[16] = pm2d5 % 100;
        TX_buffer[17] = (pm10 / 100) % 100;
        TX_buffer[18] = pm10 % 100;
        
        ELECHOUSE_cc1101.SendData(TX_buffer, size);
        delay(1000);
      }
    }
  }
}
void readTrim()
{
  uint8_t data[32], i = 0;
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xA1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  data[i] = Wire.read();
  i++;

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 7);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}
void writeReg(uint8_t reg_address, uint8_t data)
{
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}


void readData()
{
  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 8);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{

  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0;
  }
  P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000)
  {
    P = (P << 1) / ((unsigned long int) var1);
  }
  else
  {
    P = (P / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
            ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
                ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);
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
