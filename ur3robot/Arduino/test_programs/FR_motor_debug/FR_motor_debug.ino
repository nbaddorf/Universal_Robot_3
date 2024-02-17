#include <Servo.h>
#include "AS5600.h"
#include "Wire.h"

const int motorFRspeedpin = 28;//22; //28;

Servo FR;

const int FRencoderA = 5;
const int FRencoderB = 4;

int FRpos = 0;

int te = 2;

const int LED = 13;

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 1000; // 10

int toggle = 0;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  
  pinMode(FRencoderA, INPUT_PULLUP);
  pinMode(FRencoderB, INPUT_PULLUP);


  attachInterrupt(FRencoderA, FR0, RISING);
  attachInterrupt(FRencoderB, FR1, RISING);

  FR.attach(motorFRspeedpin);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}

void loop() {

  currentMillis = millis();

  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    if (!toggle) {
      Serial.println("Front Right Motor On");
      setMotorSpeed(25);
      //FR.writeMicroseconds(1600);
      toggle = 1;
    } else if (toggle == 1) {
      Serial.println("Front Right Motor reverse");
      setMotorSpeed(-25);
      //FR.writeMicroseconds(1400);
      toggle = 0;
    }

    if (true) {
    char buffer[40];
    sprintf(buffer, "FR: %d.", FRpos);
    Serial.println(buffer);
    //delay(10);
  }
  }

  // put your main code here, to run repeatedly:
  if (false) {
    Serial.println("Front Right Motor On");
    setMotorSpeed(20);
    //delay(5000);
    
  }


}


void setMotorSpeed(float fr) {
  fr = map(fr, -100, 100, 180, 0);
  FR.write(fr);
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

