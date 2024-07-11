#include <AccelStepper.h>
#include <math.h>

const int stepper_axis_1_enable_pin = 7;  //tower axis
const int stepper_axis_1_dir_pin = 8;
const int stepper_axis_1_step_pin = 9;

const int stepper_axis_2_enable_pin = 4;  //Z axis
const int stepper_axis_2_dir_pin = 5;
const int stepper_axis_2_step_pin = 6;

//These are the xlr ports on top
const int limit_1_pin = 21; 
const int limit_2_pin = 20;

// homing button pin
const int homing_button_pin = 17;

// Define a stepper and the pins it will use
AccelStepper axis1(1, stepper_axis_1_step_pin, stepper_axis_1_dir_pin);  //tower
AccelStepper axis2(1, stepper_axis_2_step_pin, stepper_axis_2_dir_pin);  //Z axis

//top speeds and accelerations for axis:
double axis1_top_speed = 0.75;    //1 radian per second
double axis1_acceleration = 0.5;  //1 radian per second per second

double axis2_top_speed = 8;     //0.02 mm per second
double axis2_acceleration = 2;  //0.01 mm per second per second

//math for top speeds and accelerations:
const int axis1_steps_per_motor_rev = 800;
const int axis1_belt_ratio = 8;                                                                  //20:160 teeth
const double axis1_steps_per_rad = (axis1_steps_per_motor_rev * axis1_belt_ratio) / (2.0 * PI);  //509.29 steps per rad

const int axis2_steps_per_motor_rev = 200;                                                //Z axis
const int axis2_gear_ratio = 30;                                                          //30:1 worm drive, 20:1 belt drive = 40mm per rev of pulley (CHECK)
const double axis2_steps_per_mm = (axis2_steps_per_motor_rev * axis2_gear_ratio) / 40.0;  //150 steps per mm

bool axis1_enabled = false;
bool axis2_enabled = false;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  //Define axis 1 stepper (Tower axis)
  axis1.setEnablePin(stepper_axis_1_enable_pin);
  axis1.setMaxSpeed(axis1_top_speed * axis1_steps_per_rad);
  axis1.setAcceleration(axis1_acceleration * axis1_steps_per_rad);
  axis1.setPinsInverted(false, false, true);

  //Define axis 2 stepper (Z axis)
  axis2.setEnablePin(stepper_axis_2_enable_pin);
  axis2.setMaxSpeed(axis2_top_speed * axis2_steps_per_mm);
  axis2.setAcceleration(axis2_acceleration * axis2_steps_per_mm);
  axis2.setPinsInverted(true, false, true);

  pinMode(homing_button_pin, INPUT_PULLUP);
  pinMode(limit_1_pin, INPUT);
  pinMode(limit_2_pin, INPUT);

int axis2_position = -0.300 * (axis2_steps_per_mm * 1000.00);  //Z axis
  axis2.moveTo(axis2_position);
}

void loop() {
  // put your main code here, to run repeatedly:

  axis1.run();
  axis2.run();

  int axis1_position = 0 * axis1_steps_per_rad;           //Tower
  

  //axis1.moveTo(axis1_position);
  Serial.print("limit: ");
  Serial.println(digitalRead(limit_1_pin));
  //Serial.println(digitalRead(limit_2_pin));
  Serial.print("homing: ");
  Serial.println(digitalRead(homing_button_pin));
  

  if (!axis1.isRunning() && axis1_enabled) {
      axis1.disableOutputs();
      axis1_enabled = false;
    } else if (axis1.isRunning() && !axis1_enabled) {
      axis1.enableOutputs();
      axis1_enabled = true;
    }

    if (!axis2.isRunning() && axis2_enabled) {
      axis2.disableOutputs();
      axis2_enabled = false;
    } else if (axis2.isRunning() && !axis2_enabled) {
      axis2.enableOutputs();
      axis2_enabled = true;
    }
}
