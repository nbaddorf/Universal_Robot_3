#include <Servo.h>

//set up which pins controll what
const int motorFLspeedpin = 22;
const int motorBLspeedpin = 23;
const int motorFRspeedpin = 28;
const int motorBRspeedpin = 29;

Servo FL;
Servo BL;
Servo FR;
Servo BR;

const int FLencoderA = 9;
const int FLencoderB = 8;
const int BLencoderA = 7;
const int BLencoderB = 6;
const int FRencoderA = 5;
const int FRencoderB = 4;
const int BRencoderA = 3;
const int BRencoderB = 2;

int BLpos = 0;
int FLpos = 0;
int BRpos = 0;
int FRpos = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(FLencoderA, INPUT_PULLUP);
  pinMode(FLencoderB, INPUT_PULLUP);
  pinMode(FRencoderA, INPUT_PULLUP);
  pinMode(FRencoderB, INPUT_PULLUP);
  pinMode(BLencoderA, INPUT_PULLUP);
  pinMode(BLencoderB, INPUT_PULLUP);
  pinMode(BRencoderA, INPUT_PULLUP);
  pinMode(BRencoderB, INPUT_PULLUP);

  attachInterrupt(FRencoderA, FR0, RISING);
  attachInterrupt(FRencoderB, FR1, RISING);
  attachInterrupt(FLencoderA, FL0, RISING);
  attachInterrupt(FLencoderB, FL1, RISING);
  attachInterrupt(BRencoderA, BR0, RISING);
  attachInterrupt(BRencoderB, BR1, RISING);
  attachInterrupt(BLencoderA, BL0, RISING);
  attachInterrupt(BLencoderB, BL1, RISING);

  FR.attach(motorFRspeedpin);
  BR.attach(motorBRspeedpin);
  FL.attach(motorFLspeedpin);
  BL.attach(motorBLspeedpin);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (false) {
    Serial.println("Front Left Motor On");
    setMotorSpeed(20,0,0,0);
    delay(5000);
    Serial.println("Back Left Motor On");
    setMotorSpeed(0,20,0,0);
    delay(5000);
    Serial.println("Front Right Motor On");
    setMotorSpeed(0,0,20,0);
    delay(5000);
    Serial.println("Back Right Motor On");
    setMotorSpeed(0,0,0,20);
    delay(5000);
    Serial.println("Off");
    setMotorSpeed(0,0,0,0);
    delay(6000);
  }

  if (true) {
    char buffer[40];
    sprintf(buffer, "FL: %d. BL: %d. FR: %d. BR: %d.", FLpos, BLpos, FRpos, BRpos);
    Serial.println(buffer);
    delay(10);
  }


}

void setMotorSpeed(float fl, float bl, float fr, float br) {
  fl = map(fl, -100, 100, 180, 0);
  fr = map(fr, -100, 100, 180, 0);
  bl = map(bl, -100, 100, 180, 0);
  br = map(br, -100, 100, 180, 0);
  FL.write(fl);
  FR.write(fr);
  BL.write(bl);
  BR.write(br);
}

void BR0() {
  if (digitalRead(BRencoderB) == LOW) {
    BRpos++;
  } else {
    BRpos--;
  }
}

void BR1() {
  if (digitalRead(BRencoderA) == LOW) {
    BRpos--;
  } else {
    BRpos++;
  }
}

void BL0() {
  if (digitalRead(BLencoderB) == LOW) {
    BLpos--;
  } else {
    BLpos++;
  }
}

void BL1() {
  if (digitalRead(BLencoderA) == LOW) {
    BLpos++;
  } else {
    BLpos--;
  }
}

void FR0() {
  if (digitalRead(FRencoderB) == LOW) {
    FRpos++;
  } else {
    FRpos--;
  }
}

void FR1() {
  if (digitalRead(FRencoderA) == LOW) {
    FRpos--;
  } else {
    FRpos++;
  }
}

void FL0() {
  if (digitalRead(FLencoderB) == LOW) {
    FLpos--;
  } else {
    FLpos++;
  }
}

void FL1() {
  if (digitalRead(FLencoderA) == LOW) {
    FLpos++;
  } else {
    FLpos--;
  }
}
