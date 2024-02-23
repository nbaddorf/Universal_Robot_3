#include <Servo.h>
#include "AS5600.h"
#include "Wire.h"

AS5600 odom_left;  //  use default Wire
AS5600 odom_front;
AS5600 odom_back;
AS5600 odom_right;

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

const int odom_a0 = 30;
const int odom_a1 = 31;
const int odom_a2 = 32;

int te = 2;

const int LED = 13;

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

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(odom_a0, OUTPUT);
  pinMode(odom_a1, OUTPUT);
  pinMode(odom_a2, OUTPUT);
  

  /* Sets the TCA9548A address to 0x70 */
  digitalWrite(odom_a0, LOW);
  digitalWrite(odom_a1, LOW);
  digitalWrite(odom_a2, LOW);

  delay(5000);

  Wire.begin();
  delay(1000);

  TCA9548A(2); //left
  odom_left.begin();                          //  set direction pin.
  odom_left.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  TCA9548A(0);//front
  odom_front.begin();                          //  set direction pin.
  odom_front.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  TCA9548A(1);//back
  odom_back.begin();                          //  set direction pin.
  odom_back.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  TCA9548A(3);//right
  odom_right.begin();                          //  set direction pin.
  odom_right.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
}

void loop() {
  // put your main code here, to run repeatedly:
  if (false) {
    Serial.println("Front Left Motor On");
    setMotorSpeed(20, 0, 0, 0);
    delay(5000);
    Serial.println("Back Left Motor On");
    setMotorSpeed(0, 20, 0, 0);
    delay(5000);
    Serial.println("Front Right Motor On");
    setMotorSpeed(0, 0, 20, 0);
    delay(5000);
    Serial.println("Back Right Motor On");
    setMotorSpeed(0, 0, 0, 20);
    delay(5000);
    Serial.println("Off");
    setMotorSpeed(0, 0, 0, 0);
    delay(6000);
  }

if (false) {
    Serial.println("Front Left Motor On");
    setMotorSpeed(-20, -20, -20, -20);
    //delay(1500);
    //setMotorSpeed(-20, -20, -20, -20);
    //Serial.println("Off");
    //delay(1500);
  }

  if (false) {
    char buffer[40];
    sprintf(buffer, "FL: %d. BL: %d. FR: %d. BR: %d.", FLpos, BLpos, FRpos, BRpos);
    Serial.println(buffer);
    delay(10);
  }

  if (true) {
    TCA9548A(2);
    Serial.print("left. ");
    Serial.print("\t");
    int test = odom_left.rawAngle();
    Serial.println(test);
    if (test >= 4500) {
      Serial.println("TTTTTTTTTTTT");
    } else if (test < 0) {
      Serial.println("BBBBBBBBBBBBBBBBB");
    }
    //TCA9548A(0);
    //Serial.print("front. ");
    //Serial.print("\t");
    //Serial.println(odom_front.rawAngle() * AS5600_RAW_TO_DEGREES);
    //TCA9548A(1);
    //Serial.print("back. ");
    //Serial.print("\t");
    //Serial.println(odom_back.rawAngle() * AS5600_RAW_TO_DEGREES);
    //TCA9548A(3);
    //Serial.print("right. ");
    //Serial.print("\t");
    //Serial.println(odom_right.rawAngle() * AS5600_RAW_TO_DEGREES);
    
    //delay(40);
  }
  //Serial.println(te);
  //Test(&te);
  //Serial.println(te);
  //delay(5000);
}


void TCA9548A(uint8_t bus) {
  if (bus == 0) {  // front
    bus = 7;
  } else if (bus == 1) {  // back
    bus = 6;
  } else if (bus == 2) {  // left
    bus = 5;
  } else {  // right
    bus = 4;
  }

  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  //Serial.print(bus);
}

void Test(int* tes) {
  *tes = 15;
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
