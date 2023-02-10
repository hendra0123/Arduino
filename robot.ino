#include <AFMotor.h>
#include <LedControl.h>
#include <Wire.h>

const int DIN_PIN = 51;
const int CS_PIN = 53;
const int CLK_PIN = 49;

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
double Kp = 0.05;  // Proporsional
double Ki = 0;     // Integral
double Kd = 0.05;  // Diferensial
int Kec_Max = 100;
int Kec_Min = 50;
double motorsp1;
double motorsp2;

#define SetPoint 5500

int hitamPutih = 0;

int proportional = 0;
int integral = 0;
int derivative = 0;
int last_proportional = 0;
float Position = 0;
int error_value = 0;
int sensors_sum = 0;
int sensors_average = 0;

int count = 1;
int pertigaan = 0;

String kode = "";

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
}

void loop() {
  cekSensor();
  if (count == 1) {
    maju(90, 90);
    delay(500);
    count++;
  } else if (count == 2) {
    if (sensors_sum >= 4) {
      bantingKanan(90, 90);
      delay(300);
      pertigaan++;
      if (pertigaan > 1) {

        count++;
      }
    } else {
      lineFollow();
    }
  } else if (count == 3) {
    if (sensors_sum >= 4) {
      bantingKiri(90, 90);
      delay(300);
      pertigaan++;
    } else {
      lineFollow();
    }
  } else {
    lineFollow();
  }


  // if (sensors_Sum > 3) {
  //   count++;
  // }

  // if (count == 2) {
  //   maju(0, 100);
  //   delay(700);
  //   count++;
  // } else {
  //   lineFollow();
  // }

  sensors_average = 0;
  sensors_sum = 0;
}

void cekSensor() {
  kode = "";

  for (int channel = 0; channel < 12; channel++) {
    int sense = readMux(channel + 1);

    if (sense >= 1000) {
      hitamPutih = 1;

      //set led
      nyalaled(channel + 1, true);

    } else {
      hitamPutih = 0;
      nyalaled(channel + 1, false);
    }

    kode = kode + hitamPutih;

    sensors_average += hitamPutih * channel * 1000;
    sensors_sum += hitamPutih;
  }

  if (sensors_average == 0) {
    Position = 0;
  } else {
    Position = sensors_average / sensors_sum;
  }
  Serial.print("Position=");
  Serial.println(Position);

  Serial.println(kode);
}

int readMux(int channel) {

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

  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  int val = analogRead(SIG_pin);

  return val;
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

void lineFollow() {

  proportional = Position - SetPoint;
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int((proportional * Kp) + (integral * Ki) + (derivative * Kd));
  Serial.print("Error Value=");
  Serial.println(error_value);

  if (error_value < -255) {
    error_value = -255;
  }
  if (error_value > 255) {
    error_value = 255;
  }

  if (error_value < 0) {
    motorsp1 = Kec_Max + error_value;
    motorsp2 = Kec_Max;
  } else {
    motorsp1 = Kec_Max;
    motorsp2 = Kec_Max - error_value;
  }


  if (motorsp2 > 100)
    motorsp2 = 100;
  if (motorsp2 < 0)
    motorsp2 = 0;
  if (motorsp1 > 100)
    motorsp1 = 100;
  if (motorsp1 < 0)
    motorsp1 = 0;

  Serial.println("Left Speed=");
  Serial.println(motorsp1);
  Serial.println("Right Speed=");
  Serial.println(motorsp2);

  maju(motorsp1, motorsp2);
}

void maju(double speed1, double speed2) {

  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);

  motor2.run(FORWARD);
  motor1.run(FORWARD);
}

void bantingKiri(double speed1, double speed2) {
  stopAllMotor();
  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);

  motor2.run(FORWARD);
  motor1.run(BACKWARD);
}

void bantingKanan(double speed1, double speed2) {
  stopAllMotor();
  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);

  motor2.run(BACKWARD);
  motor1.run(FORWARD);
}

void stopAllMotor() {
  motor2.run(RELEASE);
  motor1.run(RELEASE);
}
