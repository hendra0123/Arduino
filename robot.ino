#include <AFMotor.h>
// #include <iostream>
// #include <string>
// #include "MD_MAX72XX.h"
// #include <SPI.h>
#include <LedControl.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 12, 1);  // set the LCD address to 0x3F for a 16 chars and 2 line display

const int DIN_PIN = 51;
const int CS_PIN = 53;
const int CLK_PIN = 49;


const byte IMAGES[][8] = {
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b10000000,
    0b10000000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b11000000,
    0b11000000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b01100000,
    0b01100000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00110000,
    0b00110000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00011000,
    0b00011000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00001100,
    0b00001100,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00000110,
    0b00000110,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00000011,
    0b00000011,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00000001,
    0b00000001,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00000011,
    0b00000011,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00000110,
    0b00000110,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00011000,
    0b00011000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b00110000,
    0b00110000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b01100000,
    0b01100000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b11000000,
    0b11000000,
    0b00000000,
    0b00000000,
    0b11111111 },
  { 0b11111111,
    0b00000000,
    0b00000000,
    0b10000000,
    0b10000000,
    0b00000000,
    0b00000000,
    0b11111111 }
};
const int IMAGES_LEN = sizeof(IMAGES) / 8;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN);
unsigned long delaytime = 100;

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

int S0 = 34;
int S1 = 36;
int S2 = 38;
int S3 = 40;

int voltage;

int SIG_pin = A15;

//PID Control
double Error = 0.0;
double SumError = 0.0;
double LastError = 0.0;
double BasePWM = 50;
double Kp = 100;   // Proporsional
double Ki = 0.2;  // Integral
double Kd = 60;   // Diferensial
double Ts = 1;    // Time sampling
int NilaiPosisi;
int outPID;
int Kec_Max = 80;
int Kec_Min = 50;
double motorsp1;
double motorsp2;

void setup() {

  motor1.setSpeed(100);
  motor2.setSpeed(100);
  motor2.run(RELEASE);
  motor1.run(RELEASE);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  Serial.begin(9600);

  display.clearDisplay(0);
  display.shutdown(0, false);
  display.setIntensity(0, 10);

  // Leonardo: wait for serial port to connect
  while (!Serial) {
  }

  Serial.println();
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1);  // maybe unneeded?
    }            // end of good response
  }              // end of for loop
  Serial.println("Done.");
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
  // end of setup

  lcd.init();
  lcd.clear();
  lcd.backlight();  // Make sure backlight is on

  // Print a message on both lines of the LCD.


  // lcd.setCursor(2,1);   //Move cursor to character 2 on line 1
  // lcd.print("LCD Tutorial");
}

void displayImage(const byte* image) {
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      display.setLed(0, i, j, bitRead(image[i], 7 - j));
    }
  }
}

int i = 0;

void lineFollow() {
  // robotPosition();
  int SetPoint = 0;                // Setpoint yang diinginkan
  Error = SetPoint - NilaiPosisi;  // Error
  Serial.print("Nilai Posisi = ");
  Serial.println(NilaiPosisi);
  double DeltaError = Error - LastError;  // Delta Error (Selisih error sekarang e(t) dengan error sebelumya e(t-1))
  Serial.print("Delta Error = ");
  Serial.println(DeltaError);
  SumError += LastError;                // Akumulasi error
  double P = Kp * Error;                // Kontrol proporsional
  double I = Ki * SumError * Ts;        // Kontrol integral
  double D = ((Kd / Ts) * DeltaError);  // Kontrol derivative
  LastError = Error;                    // Error sebelumnya
  outPID = P + I + D;
  Serial.print("Out PID = ");
  Serial.println(outPID);       // Output PID
  motorsp1 = BasePWM - outPID;  // Motor Kiri
  motorsp2 = BasePWM + outPID;  // Motor Kanan

  Serial.print("Motor Sp1 = ");
  Serial.println(motorsp1);

  Serial.print("Motor Sp2 = ");
  Serial.println(motorsp2);
  /*** Pembatasan kecepatan ***/
  if (motorsp1 > Kec_Max) motorsp1 = Kec_Max;
  if (motorsp1 < Kec_Min) motorsp1 = Kec_Min;
  if (motorsp2 > Kec_Max) motorsp2 = Kec_Max;
  if (motorsp2 < Kec_Min) motorsp2 = Kec_Min;

  maju(motorsp1, motorsp2);
}

void loop() {

  // displayImage(IMAGES[i]);
  // if (++i >= IMAGES_LEN) {
  //   i = 0;
  // }
  // delay(500);


  String kode = "";
  // kode = readMux();

  for (int channel = 0; channel < 14; channel++) {
    float sense = readMux(channel);
    if (sense >= 4) {
      kode = kode + 1;

      //set led
      nyalaled(channel, true);

      switch (channel) {
        case 0:
        NilaiPosisi = NilaiPosisi - 2;
          break;
        case 1:
        NilaiPosisi = NilaiPosisi - 2;
          break;
        case 2:
        NilaiPosisi = NilaiPosisi - 2;
          break;
        case 3:
        NilaiPosisi = NilaiPosisi - 1;
          break;
        case 4:
        NilaiPosisi = NilaiPosisi - 1;
          break;
        case 5:
        NilaiPosisi = NilaiPosisi - 1;
          break;
        case 6:
        NilaiPosisi = NilaiPosisi - 0;
          break;
        case 7:
        NilaiPosisi = NilaiPosisi - 0;
          break;
        case 8:
        NilaiPosisi = NilaiPosisi + 1;
          break;
          case 9:
        NilaiPosisi = NilaiPosisi + 1;
          break;
          case 10:
        NilaiPosisi = NilaiPosisi + 1;
          break;
          case 11:
        NilaiPosisi = NilaiPosisi + 2;
          break;
          case 12:
        NilaiPosisi = NilaiPosisi + 2;
          break;
          case 13:
        NilaiPosisi = NilaiPosisi + 2;
          break;
      }

    } else {

      kode = kode + 0;
      nyalaled(channel, false);
    }
  }

  arah(kode);

  lcd.setCursor(2, 0);  //Set cursor to character 2 on line 0
  lcd.print(kode);
  // delay(1000);





  //         if(kode.equals("000000000001")) {
  //   NilaiPosisi = -16;

  //  } else if(kode.equals("000000000011")) {
  //   NilaiPosisi = -15;

  //  } else if(kode.equals("000000000111")) {
  //   NilaiPosisi = -14;

  //  } else if(kode.equals("000000000010")) {
  //   NilaiPosisi = -13;

  //  } else if(kode.equals("000000000110")) {
  //   NilaiPosisi = -12;

  //  } else if(kode.equals("000000001110")) {
  //   NilaiPosisi = -11;

  //  } else if(kode.equals("000000000100")) {
  //   NilaiPosisi = -10;

  //  } else if(kode.equals("000000001100")) {
  //   NilaiPosisi = -9;

  //  } else if(kode.equals("000000011100")) {
  //   NilaiPosisi = -8;

  //  } else if(kode.equals("000000001000")) {
  //   NilaiPosisi = -7;

  //  } else if(kode.equals("000000011000")) {
  //   NilaiPosisi = -6;

  //  } else if(kode.equals("000000111000")) {
  //   NilaiPosisi = -5;

  //  } else if(kode.equals("000000010000")) {
  //   NilaiPosisi = -4;

  //  } else if(kode.equals("000000110000")) {
  //   NilaiPosisi = -3;

  //  } else if(kode.equals("000001110000")) {
  //   NilaiPosisi = -2;

  //  } else if(kode.equals("000000100000")) {
  //   NilaiPosisi = -1;

  //  } else if(kode.equals("000001100000")) {
  //   NilaiPosisi = 0;

  //  } else if(kode.equals("000011100000")) {
  //   NilaiPosisi = 1;

  //  } else if(kode.equals("000001000000")) {
  //   NilaiPosisi = 2;

  //  } else if(kode.equals("000011000000")) {
  //   NilaiPosisi = 3;

  //  } else if(kode.equals("000111000000")) {
  //   NilaiPosisi = 4;

  //  } else if(kode.equals("000010000000")) {
  //   NilaiPosisi = 5;

  //  } else if(kode.equals("000110000000")) {
  //   NilaiPosisi = 6;

  //  } else if(kode.equals("001110000000")) {
  //   NilaiPosisi = 7;

  //  } else if(kode.equals("000100000000")) {
  //   NilaiPosisi = 8;

  //  } else if(kode.equals("001100000000")) {
  //   NilaiPosisi = 9;

  //  } else if(kode.equals("011100000000")) {
  //   NilaiPosisi = 10;

  //  } else if(kode.equals("001000000000")) {
  //   NilaiPosisi = 11;

  //  } else if(kode.equals("011000000000")) {
  //   NilaiPosisi = 12;

  //  } else if(kode.equals("111000000000")) {
  //   NilaiPosisi = 13;

  //  } else if(kode.equals("010000000000")) {
  //   NilaiPosisi = 14;

  //  } else if(kode.equals("110000000000")) {
  //   NilaiPosisi = 15;

  //  } else if(kode.equals("100000000000")) {
  //   NilaiPosisi = 16;
  //  }

  lineFollow();
  
  NilaiPosisi = 0;
}

void arah(String kode) {


  Serial.println(kode);


  // if (kode.equals("000001100000") || kode.equals("000011100000") || kode.equals("000001110000") || kode.equals("00000110000") || kode.equals("00001100000")) {
  //   maju(100, 138); //lurus

  // } else if (kode.equals("000111000000") || kode.equals("001100000000") || kode.equals("001110000000")) {
  //   maju(100, 100); //miring ke kiri
  // } else if (kode.equals("000000111000") || kode.equals("000000001100") || kode.equals("000000011100")) {
  //   maju(100, 150); //miring ke kanan
  // } else if (kode.equals("100001100001") || kode.equals("110001100001") || kode.equals("100001100011") || kode.equals("110001100011") || kode.equals("100011100011") || kode.equals("110001110001")) {
  //   maju(0, 100); //perempatan5
  // } else if (kode.equals("100000000001") || kode.equals("110000000001") || kode.equals("100000000011") || kode.equals("110000000011")) {
  //   maju(0, 100); //pertigaan
  // } else if (kode.equals("011000000000") || kode.equals("011100000000") || kode.equals("110000000000") || kode.equals("111000000000")) {
  //   maju(0, 100); //kiri
  // } else if (kode.equals("000000000110") || kode.equals("000000001110") || kode.equals("000000000011") || kode.equals("000000000111")) {
  //   maju(100, 0); //kanan

  // } else if (kode.equals("000001100000")) {
  //   maju(100, 100);
  // } else if (kode.equals("000001100000")) {
  //   maju(100, 100);
  // } else if (kode.equals("000001100000")) {
  //   maju(100, 100);
  // } else if (kode.equals("000001100000")) {
  //   maju(100, 100);
}
// else {
//   stopAllMotor();
// }
// }

float readMux(int channel) {

  int muxChannel[16][4] = {
    { 0, 0, 0, 0 },  //channel 0
    { 1, 0, 0, 0 },  //channel 1
    { 0, 1, 0, 0 },  //channel 2
    { 1, 1, 0, 0 },  //channel 3
    { 0, 0, 1, 0 },  //channel 4
    { 1, 0, 1, 0 },  //channel 5
    { 0, 1, 1, 0 },  //channel 6
    { 1, 1, 1, 0 },  //channel 7
    { 0, 0, 0, 1 },  //channel 8
    { 1, 0, 0, 1 },  //channel 9
    { 0, 1, 0, 1 },  //channel 10
    { 1, 1, 0, 1 },  //channel 11
    { 0, 0, 1, 1 },  //channel 12
    { 1, 0, 1, 1 },  //channel 13
    { 0, 1, 1, 1 },  //channel 14
    { 1, 1, 1, 1 }   //channel 15
  };

  int controlPin[] = { S0, S1, S2, S3 };
  // String kode = "";



  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  int val = analogRead(SIG_pin);
  voltage = (val * 5) / 1024;
  // Serial.println(voltage);

  return voltage;
  // delay(1);


  //  if (voltage >= 4) {
  //   kode = kode + 1;

  //   //set led
  //   nyalaled(channel, true);

  // } else {
  //   kode = kode + 0;
  //   nyalaled(channel, false);
  // }

  // return kode;
}

void nyalaled(int channel, bool on) {
  switch (channel) {
    case 13:
      display.setLed(0, 0, 5, on);
      break;
    case 12:
      display.setLed(0, 0, 4, on);
      break;
    case 11:
      display.setLed(0, 0, 3, on);
      break;
    case 10:
      display.setLed(0, 0, 2, on);
      break;
    case 9:
      display.setLed(0, 1, 1, on);
      break;
    case 8:
      display.setLed(0, 2, 0, on);
      break;
    case 7:
      display.setLed(0, 3, 0, on);
      break;
    case 6:
      display.setLed(0, 4, 0, on);
      break;
    case 5:
      display.setLed(0, 5, 0, on);
      break;
    case 4:
      display.setLed(0, 6, 1, on);
      break;
    case 3:
      display.setLed(0, 7, 2, on);
      break;
    case 2:
      display.setLed(0, 7, 3, on);
      break;
    case 1:
      display.setLed(0, 7, 4, on);
      break;
    case 0:
      display.setLed(0, 7, 5, on);
      break;
  }
}

void maju(double speed1, double speed2) {

  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);

  motor2.run(FORWARD);
  motor1.run(FORWARD);
  lcd.setCursor(0, 1);  //Set cursor to character 2 on line 0
  lcd.print(speed1);
  Serial.println(speed1);
  Serial.println(speed2);
}

void stopAllMotor() {

  motor2.run(RELEASE);
  motor1.run(RELEASE);
}