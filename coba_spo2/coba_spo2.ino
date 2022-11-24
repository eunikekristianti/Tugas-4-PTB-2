#include <Wire.h>

#define I2CADDR 0x57

bool dataBaru = false;
bool withBPF  = false;
bool withPeak  = false;
bool withHR = false;
bool withSpO2 = false;
uint8_t counter;
float yi0, yi1, yi2, yi3, yi4;
float i1, i2, i3, i4;
float yr0, yr1, yr2, yr3, yr4;
float r1, r2, r3, r4;
float thia0, thia1;
float thib0, thib1;
float thra0, thra1;
float thrb0, thrb1;
float peak_ir = 0; 
float bottom_ir = 0;
float peak_r = 0;
float bottom_r = 0;
int beat = 0;
int count = 0;
int bpm = 0;
float rms_ir = 0;
float rms_r = 0;
float AC_IR = 0; 
float DC_IR = 0;
float AC_R = 0;
float DC_R = 0;
float R = 0;
float SpO2 = 0;
float sum_ir = 0;
float sum_r = 0;
float temp = 0;
float sum_dc_ir = 0;
float sum_dc_r = 0;

void registerWrite(uint8_t regaddr, uint8_t regdata) {
  Wire.beginTransmission(I2CADDR);
  Wire.write(regaddr);
  Wire.write(regdata);
  Wire.endTransmission();
}

uint8_t registerRead(uint8_t regaddr) {
  Wire.beginTransmission(I2CADDR);
  Wire.write(regaddr);
  Wire.endTransmission(0);
  Wire.requestFrom(I2CADDR, 1);
  return Wire.read();
}

void dataRead(uint8_t *FIFOdata) {
  Wire.beginTransmission(I2CADDR);
  Wire.write(0x05);
  Wire.endTransmission(0);
  Wire.requestFrom(I2CADDR, 4);
  FIFOdata[0] = Wire.read();
  FIFOdata[1] = Wire.read();
  FIFOdata[2] = Wire.read();
  FIFOdata[3] = Wire.read();
}

void adaDataBaru() {
  dataBaru = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(2, INPUT);
  registerWrite(0x06, B01000000); // Reset
  registerWrite(0x01, B00010000); // SpO2 Data Ready Interrupt
  registerWrite(0x06, B00000011); // SpO2 enabled mode
  registerWrite(0x07, B00000110); // 100 sps ADC, 1600 us pulse width
  registerWrite(0x09, B10010111); // 30.6 mA RED, 24.0 mA IR //arus Eunike
  attachInterrupt(digitalPinToInterrupt(2), adaDataBaru, FALLING );
  registerRead(0x00);
  counter = 0;
}

void loop() {
  if(Serial.available()) {
    byte command=Serial.read();
    if (command==102) {      // 'f'
      withBPF=1^withBPF;
    }
    else {
      if (command==104) { // 'h'
        withHR=1^withHR;
      }
      else {
        if (command==115) {  // 's'
          withSpO2=1^withSpO2; 
        }
      }
    }
  }
  
  uint8_t data[4];
  uint16_t ir, red;

  if (dataBaru) {
    count++;
    dataBaru = false;
    dataRead(data);
    ir  = (data[0] << 8) | data[1];
    red = (data[2] << 8) | data[3];
    if (counter < 4) {
      yi0 = 0; yi1 = 0; yi2 = 0; yi3 = 0; yi4 = 0; i1 = ir;  i2 = i1; i3 = i2; i4 = i3;
      yr0 = 0; yr1 = 0; yr2 = 0; yr3 = 0; yr4 = 0; r1 = red; r2 = r1; r3 = r2; r4 = r3;
      thia0 = 0; thia1 = 0; thib0 = 0; thib1 = 0; thra0 = 0; thra1 = 0; thrb0 = 0; thrb1 = 0;
      counter++;
    }
    else {  // y[n]= 3.6409*y[n-1] - 5.0194*y[n-2] + 3.1133*y[n-3] - 0.7349*y[n-4] - 0.0169*x[n] + 0.0338*x[n-2] - 0.0169*x[n-4]
      // persamaan LCCDE
      yi0 = 3.6409*yi1 - 5.0194*yi2 + 3.1133*yi3 - 0.7349*yi4 - 0.0169*ir + 0.0338*i2 - 0.0169*i4;
      yr0 = 3.6409*yr1 - 5.0194*yr2 + 3.1133*yr3 - 0.7349*yr4 - 0.0169*red + 0.0338*r2 - 0.0169*r4;
      // persamaan threshold HR 
      thia0 = 0.1*yi0 + 0.9*thia1;
      thia1 = thia0;
      if ((yi1 > thia1) and (yi1 > yi2) and (yi1 >= yi0)) {
        peak_ir = yi0;
        beat++;
      }

      // mencari peak dan bottom gelombang
      thib0 = 0.9*thib1 - 0.1*yi0;
      thib1 = thib0;
      if ((yi1 < thib1) and (yi1 < yi2) and (yi1 <= yi0)) {
        bottom_ir = yi0;
      }

      thra0 = 0.1*yr0 + 0.9*thra1;
      thra1 = thra0;
      if ((yr1 > thra1) and (yr1 > yr2) and (yr1 >= yr0)) {
        peak_r = yr0;
      }

      thrb0 = 0.9*thrb1 - 0.1*yr0;
      thrb1 = thrb0;
      if ((yr1 < thrb1) and (yr1 < yr2) and (yr1 <= yr0)) {
        bottom_r = yr0;
      }

      // mencari nilai R untuk SpO2
      AC_IR = peak_ir - bottom_ir;
      sum_ir = sum_ir + pow(AC_IR, 2);
      rms_ir = pow((sum_ir/count), 0.5);
      sum_dc_ir = sum_dc_ir + (AC_IR / 2);
      AC_R = peak_r - bottom_r;
      sum_r = sum_r + pow(AC_R, 2);
      rms_r = pow((sum_r/count), 0.5);
      sum_dc_r = sum_dc_r + (AC_R / 2);

      // shifting
      yi4 = yi3; i4 = i3; yi3 = yi2; i3 = i2; yi2 = yi1; i2 = i1; yi1 = yi0; i1 = ir;
      yr4 = yr3; r4 = r3; yr3 = yr2; r3 = r2; yr2 = yr1; r2 = r1; yr1 = yr0; r1 = red;    
    } 

    if (count == 500) {
      DC_IR = sum_dc_ir / count;
      DC_R = sum_dc_r / count;
      R = (rms_r/DC_R) / (rms_ir/DC_IR);
      SpO2 = 122 - (25*R);
      bpm = beat*12;
      count = 0;
      beat = 0;
      sum_r = 0;
      sum_ir = 0;
      sum_dc_ir = 0;
      sum_dc_r = 0;
    }
    
    if (withBPF) {
      Serial.print(yi0); Serial.print('\t'); Serial.println(yr0);
    }
    else {
      if (withHR) {
        Serial.print(bpm); Serial.print('\t'); Serial.println(bpm);
      }
      else {
        if (withSpO2) {
          Serial.print((int)SpO2); Serial.print('\t'); Serial.println((int)SpO2);
        }
        else {
          Serial.print(ir); Serial.print('\t'); Serial.println(red);
        }
      }
    }
  }
}
