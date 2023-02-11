#include <AFMotor.h>
#include <LedControl.h>
#include <Wire.h>

const int DIN_PIN = 51;
const int CS_PIN = 53;
const int CLK_PIN = 49;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN);
// unsigned long delaytime = 100;

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

int S0 = 34;
int S1 = 36;
int S2 = 38;
int S3 = 40;

int voltage;

int SIG_pin = A14;

//PID Control
// double Kp = 0.028;// Proporsional
double Kp = 0.03;  // Proporsional
double Ki = 0;     // Integral
// double Kd = 0.052;  // Diferensial
double Kd = 0.06;  // Diferensial
int Kec_Max = 90;
int Kec_Min = 0;
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

int count = 3;
int pertigaan = 0;
int GudangTujuan = 0;
int PabrikTujuan = 0;
int GudangAsal = 0;
int PabrikAsal = 0;

int kodeSensor[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int rutePabrik[3][3][3] = { { { 0, 3, 3 },      // pabrik 1 ke gudang 1
                              { 1, 2, 3 },      // pabrik 1 ke gudang 2
                              { 1, 0, 2 } },    // pabrik 1 ke gudang 3
                            { { 2, 1, 3 },      // pabrik 2 ke gudang 1
                              { 0, 3, 3 },      // pabrik 2 ke gudang 2
                              { 1, 2, 3 } },    // pabrik 2 ke gudang 3
                            { { 2, 0, 1 },      // pabrik 3 ke gudang 1
                              { 2, 1, 3 },      // pabrik 3 ke gudang 2
                              { 0, 3, 3 } } };  // pabrik 3 ke gudang 3

int ruteGudang[3][3][3] = { { { 3, 3, 3 },
                              { 2, 1, 3 },    // gudang 1 ke pabrik 2
                              { 2, 0, 1 } },  // gudang 1 ke pabrik 3
                            { { 3, 3, 3 },
                              { 0, 3, 3 },    // gudang 2 ke pabrik 2
                              { 2, 1, 3 } },  // gudang 2 ke pabrik 3
                            { { 3, 3, 3 },
                              { 1, 2, 3 },      // gudang 3 ke pabrik 2
                              { 0, 3, 3 } } };  // gudang 3 ke pabrik 3

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
    // maju(100, 100);
    // delay(300);
    // count++;
  } else if (count == 2) {
    // if (sensors_sum >= 4 && (Position >= 3500 && Position <= 7500)) {
    //   bantingKanan(90, 90);
    //   delay(200);
    // } else {
    //   lineFollow();
    // }
    lineFollow();
  } else if (count == 3) {
    if (sensors_sum >= 4 && (Position >= 3500 && Position <= 7500)) {

      if (pertigaan == 1) {
        // ambil barang
        PabrikAsal = 0;
        GudangTujuan = 1;  // tergantung
        putarBalik();
        pertigaan = 0;
        count++;
      } else if (pertigaan == 0) {
        bantingKiri(90, 90);
        delay(500);
        pertigaan++;
      }
    } else {
      lineFollow();
    }
  } else if (count == 4) {
    if (sensors_sum >= 4 && (Position >= 3500 && Position <= 7500)) {
      cariRute(rutePabrik[PabrikAsal][GudangTujuan][pertigaan]);
      pertigaan++;
    } else {
      lineFollow();
    }
  } else if (count == 5) {
    if (sensors_sum >= 4 && (Position >= 3500 && Position <= 7500)) {
      cariRute(ruteGudang[GudangAsal][PabrikTujuan][pertigaan]);
      pertigaan++;
    } else {
      lineFollow();
    }

    sensors_average = 0;
    sensors_sum = 0;
  } else {
    lineFollow();
  }
}


void cekSensor() {
  kode = "";

  sensors_sum = 0;
  sensors_average = 0;

  for (int channel = 0; channel < 12; channel++) {
    int sense = readMux(channel + 1);

    if (sense >= 1010) {
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

void putarBalik() {
  stopAllMotor();
  delay(200);
  bantingKiri(100, 100);
  delay(900);
  stopAllMotor();
  delay(200);
}

void bantingKiri(double speed1, double speed2) {
  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);

  motor2.run(FORWARD);
  motor1.run(BACKWARD);
}

void bantingKanan(double speed1, double speed2) {
  motor1.setSpeed(speed1);
  motor2.setSpeed(speed2);

  motor2.run(BACKWARD);
  motor1.run(FORWARD);
}

void stopAllMotor() {
  motor2.run(RELEASE);
  motor1.run(RELEASE);
}

void cariRute(int perintah) {
  switch (perintah) {
    case 0:  //lurus
      maju(90, 90);
      delay(300);
      break;

    case 1:  //kiri
      bantingKiri(90, 90);
      delay(300);
      break;

    case 2:  //kanan
      bantingKanan(90, 90);
      delay(300);
      break;

    case 3:
      // ambil barang
      putarBalik();
      count++;
      PabrikTujuan = PabrikAsal + 1;
      GudangAsal = GudangTujuan;
      break;

    case 4:
      break;

    default:
      return;
  }
}
