#include <math.h>
#include <FlexCAN_T4.h>
#include <PID_v1.h>


const int LED = 13;

// homing button pin
const int homing_button_pin = 17;

//E_Stop variables
const int e_stop_pin = 16;

bool e_stop = true;
bool old_e_stop = true;

bool has_homed = false; // SHOULD BE false
//bool started_homing = true;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;


//math for top speeds and accelerations:
const int axis1_steps_per_motor_rev = 800;
const int axis1_gear_ratio = 8; //20:160 teeth
const double axis1_steps_per_rad = (axis1_steps_per_motor_rev * axis1_gear_ratio) / (2.0 * PI); //1018.59 steps per rad

const int axis2_steps_per_motor_rev = 200; //Z axis
const int axis2_gear_ratio = 30; //30:1 worm drive, 20:1 belt drive = 40mm per rev of pulley (CHECK)
const double axis2_mm_per_rad = 0.212207; // mm
double axis2_current_pos = 0;

const int axis3_gear_ratio = 4; //20:80 teeth


unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 30; 

struct {
  bool axis1 = false;
  bool axis2 = false;
  bool axis3 = false;
  //const double axis1_homing_speed = 0.12;
  //const int axis2_homing_speed = 3;
  //const int axis3_homing_speed = 10;
  const double axis1_offset = 4.63239; //encoder offset in rad DONT CARE A OU
  const double axis2_offset = 0;
  const double axis3_offset = 0.06452; // encoder offset in rad 0.09052
} homed;

struct can_motor {
  uint8_t id = 0;
  bool sent_home_command = false;
  bool motor_enabled = true;
  //bool sent_get_encoder_command = false;
  uint8_t request_encoder_counter = 0;
  bool sent_position_command = false;
  double encoder = 0; // Value to store encoder position.
  double lower_limit = 0; // Lower limit of movement allowed (in m or rad) (Used in constrain for input value)
  double upper_limit = 0;// Upper limit of movement allowed (in m or rad) 
  double pid_speed = 0; // Speed value that will be used for running motor at speed.
  double setpoint = 0; // Setpoint for PID value
};

double joints_pos[] = {0,0,0};

can_motor axis1;
can_motor axis2;
can_motor axis3;

void setup() {
  Serial.begin(115200);

  can1.begin();
  can1.setBaudRate(500000); //250000

  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canRecieve);
  can1.mailboxStatus();

  pinMode(LED, OUTPUT);
  pinMode(homing_button_pin, INPUT_PULLUP);
  pinMode(e_stop_pin, INPUT_PULLUP);

  old_e_stop = digitalRead(e_stop_pin); //set e_stop values to whatever they currently are

  axis1.id = 0x01; 
  axis2.id = 0x02;
  axis3.id = 0x03;

}

void loop() {

  can1.events();

  currentMillis = millis();

  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    if (has_homed) {
    getCanEncoderVal(axis1);
    getCanEncoderVal(axis2);
    getCanEncoderVal(axis3);
    } else {
      home_axis();
    }

    for (int i = 0; i < 3; i++) {
      Serial.print("axis");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(joint_pos[i]);
    }



  }


}

void canRecieve(const CAN_message_t &msg) {
  switch (msg.buf[0]) {
      case 0x31: // encoder
        switch (msg.id) {
          case 1:
            joints_pos[0] = (returnEncoderRad(msg) / axis1_gear_ratio);  //  - homed.axis1_offset;
            //axis1.encoder = (axis1.encoder > axis1.upper_limit + 5) ? axis1.lower_limit : axis1.encoder;
            //axis1.sent_get_encoder_command = false;
            axis1.request_encoder_counter = 0;
            //Serial.print("axis1: ");
            //Serial.println(axis1.encoder);
            break;
          case 2:
            //joints_pos[1] = 1647099.33 - (returnEncoderRad(msg)) - homed.axis2_offset; //54903.31 is the vars overflow.
            joints_pos[1] = (returnEncoderRad(msg)); //54903.31 is the vars overflow.
            //axis2.encoder = (axis2.encoder > 2500) ? 0.0 : axis2.encoder;
            
            axis2_current_pos = (joints_pos[1] * axis2_mm_per_rad) / 1000;
            //axis2.sent_get_encoder_command = false;
            axis2.request_encoder_counter = 0;
            //Serial.print("axis2: ");
            //Serial.println(axis2.encoder);
            break;
          case 3:
            joints_pos[2] = (returnEncoderRad(msg) / axis3_gear_ratio);
            //axis3.encoder = (axis3.encoder > 6) ? axis3.lower_limit : axis3.encoder;
            //axis3.sent_get_encoder_command = false;
            axis3.request_encoder_counter = 0;
            //Serial.print("axis3: ");
            //Serial.println(axis3.encoder);
            
            break;
        }
        break;
      case 0x91: //homing responce (If CanRSP is on)
        switch (msg.id) {
          case 1:
            homed.axis1 = (msg.buf[1] == 2);
            axis1.sent_home_command = !(msg.buf[1] == 2);
            break;
          case 2:
            homed.axis2 = (msg.buf[1] == 2);
            axis2.sent_home_command = !(msg.buf[1] == 2);
            break;
          case 3:
            homed.axis3 = (msg.buf[1] == 2);
            axis3.sent_home_command = !(msg.buf[1] == 2);
            break;
        }
        break;

}

double returnEncoderRad(CAN_message_t recievedMsg) {
  u_long encoderVal = recievedMsg.buf[2]; //demo 0x01;
  encoderVal = (encoderVal << 8) | recievedMsg.buf[3];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[4];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[5];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[6];

  return ((encoderVal / double (0x2000)) * PI); // take the encoder then divide by 1/2 rotation then multiply by PI
}

void getCanEncoderVal(can_motor &motor) {
  //if (!motor.sent_get_encoder_command) {
  if (motor.request_encoder_counter == 0) {
    CAN_message_t msgSend;
    msgSend.len = 2;
    msgSend.id = motor.id;
    msgSend.buf[0] = 0x31; //encoder mode
    msgSend.buf[1] = msgSend.id + msgSend.buf[0]; //checksum
    //motor.sent_get_encoder_command = true;
    can1.write(msgSend);
    motor.request_encoder_counter++;
  } else if (motor.request_encoder_counter > 3) {
    motor.request_encoder_counter = 0;
  } else {
    motor.request_encoder_counter++;
  }

  
}

void home_axis() {
  if (!homed.axis3) {
    //query_motor(axis3);
    goHomeCan(axis3);
  } else if (!homed.axis2) {
    //query_motor(axis2);
    goHomeCan(axis2);
  } else if (!homed.axis1) {
    //query_motor(axis1);
    goHomeCan(axis1);
  }
  if (homed.axis1 && homed.axis2 && homed.axis3) {
    has_homed = true;
  }
  
}

