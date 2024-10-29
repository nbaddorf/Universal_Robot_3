#include <FlexCAN_T4.h>
#include <math.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;

struct motor_values {
  double radians = 0;
  bool enabled = false;
  int run_status = 0;
};

int numLoop = 1;

motor_values motor1;


void setup() {
  // put your setup code here, to run once:
can1.begin();
  can1.setBaudRate(500000);
  Serial.begin(115200);
//getEncoderVal();
delay(1000);
getEncoderVal(numLoop);


  //setCanMotorSpeed(10, 0);
}

void loop() {
  
  /* position control (foc mode)
msgSend.len=8;
msgSend.id=0x01;
msgSend.buf[0]=0xFD;
msgSend.buf[1]=0x01;
msgSend.buf[2]=0x40;
msgSend.buf[3]=0x05;
msgSend.buf[4]=0x09;
msgSend.buf[5]=0xC4;
msgSend.buf[6]=0x00;
msgSend.buf[7]=0x11;
*/
//msgSend.len=2;
//msgSend.id=0x01;
//msgSend.buf[0]=0x30;
//msgSend.buf[1]=0x31;
//msgSend.buf[2]=0x40;
//msgSend.buf[3]=0x05;
//msgSend.buf[4]=0x09;
//msgSend.buf[5]=0xC4;
//msgSend.buf[6]=0x00;
//msgSend.buf[7]=0x11;


  // put your main code here, to run repeatedly:
if ( can1.read(msg) ) {
  
  //if (numLoop > 3) {
  //  numLoop = 1;
  //  Serial.println();
  //}
    //Serial.print("CAN1 "); 
    //Serial.print("MB: "); Serial.print(msg.mb);
    //Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
    //Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    //Serial.print("  LEN: "); Serial.print(msg.len);
    //Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      //Serial.print(msg.buf[i], HEX); Serial.print(" ");
    }
    //Serial.print("  TS: "); Serial.println(msg.timestamp);

    switch (msg.buf[0]) {
      case 0x31:
        motor1.radians = returnEncoderRad(msg);
        //Serial.print(msg.id);
        Serial.print("motor: ");
        Serial.println(motor1.radians);
        //Serial.print(" ");
        break;
      case 0xF3:
        motor1.enabled = (msg.buf[1] == 0x01) ? true : false;
        break;
      case 0xF5:
        motor1.run_status = msg.buf[1]; //0 = fail, 1 = starting, 2 = complete, 3 = end limit stoped
        break;

    }
    

    delay(100);
    getEncoderVal(1);
    //numLoop++;

   
  

  //
  }

      
  
  

}

void goHome() {
  CAN_message_t msgSend;
  msgSend.len=2;
  msgSend.id= 0x01;
  msgSend.buf[0] = 0x91; //go home command
  msgSend.buf[1] = 0x92; //crc

  can1.write(msgSend);
}

void setCanMotorSpeed(short speed, uint8_t acc) { //DOESNT WORK
    //4095 is the max size for unsigned 12 bit number
    uint16_t bin_speed = abs(constrain(speed, -4095, 4095)); // make an uint16 where the lowest 12 bits get set regardless of signedness, and the high-order 4 bits are allways 0. Example for value 4095 (or -4095) value is 0000111111111111.
    
    bin_speed = (speed < 0) ? bin_speed | (1U << 15) : bin_speed; // if speed is negative then perform bitwise or. 1U == unsigned int (16 bit minimum) with a value of 1. (0000000000000001). Then left shift it 15 places to result in 1000000000000000.

    CAN_message_t msgSend;
    msgSend.len = 5;
    msgSend.id = 0x01;
    msgSend.buf[0] = 0xF6; //run motor in speed mode
    msgSend.buf[1] = bin_speed >> 8; 
    msgSend.buf[2] = bin_speed; // sets the 8 bit num to the lowest order 8 bits in our 16 bit number
    msgSend.buf[3] = acc; //acceleration

    uint8_t crc = msgSend.id;
    for (int i = 0; i<7; i++) {
      crc += msgSend.buf[i];
    }

    msgSend.buf[4] = crc;
    can1.write(msgSend);

}

void runToPosition(long encoder_counts, uint16_t speed, uint8_t acc) { 
  CAN_message_t msgSend;
  speed = constrain(speed, 0, 3000);
  msgSend.len=8;
  msgSend.id=0x01;
  msgSend.buf[0]=0xF5; //run motor to absolute position mode 4
  msgSend.buf[2]=speed; 
  msgSend.buf[1]=speed >> 8; 
  msgSend.buf[3]= acc; //acceleration
  msgSend.buf[6]=encoder_counts;
  msgSend.buf[5]=encoder_counts >> 8;
  msgSend.buf[4]=encoder_counts >> 16;

  uint8_t crc = msgSend.id;
  for (int i = 0; i<7; i++) {
    crc += msgSend.buf[i];
  }

  msgSend.buf[7] = crc;
  
  can1.write(msgSend);
}

void getEncoderVal(int id) {
  CAN_message_t msgSend;
  msgSend.len=2;
  msgSend.id=id;
  msgSend.buf[0]=0x31; //encoder mode
  msgSend.buf[1]=msgSend.buf[0] + id; //checksum

  can1.write(msgSend);
}

void setMotorEnable(bool status) {
  CAN_message_t msgSend;
  msgSend.len=3;
  msgSend.id=0x01;
  msgSend.buf[0]=0xF3; //enable motor
  msgSend.buf[1]= status ? 0x01 : 0x00;
  msgSend.buf[2]= status ? 0xF5 : 0xF4;

  can1.write(msgSend);
}

double returnEncoderRad(CAN_message_t recievedMsg) {
  u_long encoderVal = recievedMsg.buf[2]; //demo 0x01;
  encoderVal = (encoderVal << 8) | recievedMsg.buf[3];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[4];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[5];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[6];

  /*
  encoderVal <<= 8; //left shift 8 bits so 0x01 becomes 0x100
  encoderVal |= msg.buf[3]; //OR operation, merges 0x100 with for example 0x72 to result in 0x172
  */

  return ((encoderVal / double (0x2000)) * PI); // take the encoder then divide by 1/2 rotation then multiply by PI
}
/*
void initMotor(motor_values &values) {
  CAN_message_t initMsg;
  getEncoderVal();
  bool gotEncoder = false;
  bool gotEnabled = false;
  bool gotRunStatus = false;
  while (!can1.read(initMsg)) {
    if (initMsg.buf[0] == 0x31) {
      values.radians = returnEncoderRad(initMsg);
      gotEncoder = true;
    } else if (initMsg.buf[0] == 0xF3) {
      values.enabled = (initMsg.buf[1] == 0x01) ? true : false;
      gotEnabled = true;
    }
  }
  
}
*/