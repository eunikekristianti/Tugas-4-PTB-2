#include <Wire.h>

#define I2CADDR 0x57

bool dataBaru = false;
uint8_t counter;
float yi0, yi1, yi2, yi3, yi4, i1, i2, i3, i4;
float yr0, yr1, yr2, yr3, yr4, r1, r2, r3, r4;

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
  registerWrite(0x06, B00000011); // SpO2 mode
  registerWrite(0x07, B00000111); // 100 sps ADC, 1600 us pulse width
  registerWrite(0x09, B10010111); // 30.6 mA RED, 24.0 mA IR //arus Eunike
  attachInterrupt(digitalPinToInterrupt(2), adaDataBaru, FALLING );
  registerRead(0x00);
  counter = 0;
}

void loop() {
  uint8_t data[4];
  uint16_t ir, red;

  if (dataBaru) {
    dataBaru = false;
    dataRead(data);
    ir  = (data[0] << 8) | data[1];
    red = (data[2] << 8) | data[3];
    // chebyshev 1 orde 4
    if (counter < 4) {
      yi0 = 0; yi1 = 0; yi2 = 0; yi3 = 0; yi4 = 0; i1 = ir;  i2 = i1; i3 = i2; i4 = i3;
      yr0 = 0; yr1 = 0; yr2 = 0; yr3 = 0; yr4 = 0; r1 = red; r2 = r1; r3 = r2; r4 = r3;
      counter++;
    }
    else {  // y[n]= 3.6409*y[n-1] - 5.0194*y[n-2] + 3.1133*y[n-3] - 0.7349*y[n-4] - 0.0169*x[n] + 0.0338*x[n-2] - 0.0169*x[n-4]
      yi0 = 3.6409*yi1 - 5.0194*yi2 + 3.1133*yi3 - 0.7349*yi4 - 0.0169*ir + 0.0338*i2 - 0.0169*i4;
      yi4 = yi3; i4 = i3; yi3 = yi2; i3 = i2; yi2 = yi1; i2 = i1; yi1 = yi0; i1 = ir;
      yr0 = 3.6409*yr1 - 5.0194*yr2 + 3.1133*yr3 - 0.7349*yr4 - 0.0169*red + 0.0338*r2 - 0.0169*r4;
      yr4 = yr3; r4 = r3; yr3 = yr2; r3 = r2; yr2 = yr1; r2 = r1; yr1 = yr0; r1 = red;
      Serial.print(yi0); Serial.print('\t'); Serial.println(yr0);
    } 
    /* chebyshev 2 orde 4
    if (counter < 4) {
      yi0 = 0; yi1 = 0; yi2 = 0; yi3 = 0; yi4 = 0; i1 = ir;  i2 = i1; i3 = i2; i4 = i3;
      yr0 = 0; yr1 = 0; yr2 = 0; yr3 = 0; yr4 = 0; r1 = red; r2 = r1; r3 = r2; r4 = r3;
      counter++;
    }
    else {  // y[n]= 3.6409*y[n-1] - 5.0194*y[n-2] + 3.1133*y[n-3] - 0.7349*y[n-4] - 0.0169*x[n] + 0.0338*x[n-2] - 0.0169*x[n-4]
      yi0 = 3.6608*yi1 - 5.0815*yi2 + 3.1780*yi3 - 0.7574*yi4 - 0.5027*ir + 1.9229*i1 - 2.8404*i2 + 1.9229*i3 - 0.5027*i4;
      yi4 = yi3; i4 = i3; yi3 = yi2; i3 = i2; yi2 = yi1; i2 = i1; yi1 = yi0; i1 = ir;
      yr0 = 3.6608*yr1 - 5.0815*yr2 + 3.1780*yr3 - 0.7574*yr4 - 0.5027*red + 1.9229*r1 - 2.8404*r2 + 1.9229*r3 - 0.5027*r4;
      yr4 = yr3; r4 = r3; yr3 = yr2; r3 = r2; yr2 = yr1; r2 = r1; yr1 = yr0; r1 = red;
      Serial.print(yi0); Serial.print('\t'); Serial.println(yr0);
    } */
  }
}
