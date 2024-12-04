#include <robot.hpp>

// sensor wire documentation 2022-2023:
// https://docs.google.com/spreadsheets/d/1eX79YtawJqBA8VePFFtKJT6RR21gvK4qH1DKJX_vJE0/edit#gid=0

// MCs
Sabertooth MC1(128, ST_SERIAL); // LINEAR ACTUATORS

// ADCs
namespace actuation {

Sabertooth_MotorCtrl act_right_mtr{&MC1, STMotor::M1};
Sabertooth_MotorCtrl act_left_mtr{&MC1, STMotor::M2};

constexpr uint8_t ACT_RIGHT_CURR_ADC = 0;
constexpr uint8_t ACT_RIGHT_CURR_MUX = 1; // U6 curr_sense_board

constexpr uint8_t ACT_LEFT_CURR_ADC = 1;
constexpr uint8_t ACT_LEFT_CURR_MUX = 1; // U4 curr_sense_board

void update(float &act_right_curr) {
  act_right_curr = ADS1119_Current_Bus::read(ACT_RIGHT_CURR_ADC, ACT_RIGHT_CURR_MUX);
}

void cb(int8_t lin_act_volt) {
  act_left_mtr.write(lin_act_volt);
  act_right_mtr.write(lin_act_volt);
}

} // namespace actuation

namespace drivetrain {
StepperMotor left_drive_mtr(LEFT_DRIVE_MOTOR_ID);
StepperMotor right_drive_mtr(RIGHT_DRIVE_MOTOR_ID);

void begin() {
  left_drive_mtr.begin();
  right_drive_mtr.begin();
}

void update(float &left_curr, float &right_curr, float &left_angle, float &right_angle) {
  left_curr = left_drive_mtr.read_current();
  right_curr = right_drive_mtr.read_current();
  left_angle = left_drive_mtr.read_motor_position_radians();
  right_angle = -right_drive_mtr.read_motor_position_radians();
}

float update_curr_left() {
  return left_drive_mtr.read_current();
}

float update_curr_right() {
  return right_drive_mtr.read_current();
}

void cb(int32_t left_drive_rpm, int32_t right_drive_rpm) { // TODO RJN - ensure this entire message chain is int32 RPM
  // Tank drive steering
  left_drive_mtr.move_at_speed(left_drive_rpm);
  right_drive_mtr.move_at_speed(right_drive_rpm);
}

} // namespace drivetrain

namespace uwb {
void update(float &d0, float &d1, float &d2) {
  d0 = M5Stack_UWB_Trncvr::read_uwb(0);
  d1 = M5Stack_UWB_Trncvr::read_uwb(1);
  d2 = M5Stack_UWB_Trncvr::read_uwb(2);
}
} // namespace uwb

namespace excavation {
StepperMotor exc_mtr(EXC_MOTOR_ID);

void begin() {
  exc_mtr.begin();
}

void update(float &exc_curr, float &exc_angle) {
  exc_curr = exc_mtr.read_current();
  exc_angle = -exc_mtr.read_motor_position_radians();
}

float update_curr() {
  return exc_mtr.read_current();
}

void cb(int32_t speed_rpm) {
  exc_mtr.move_at_speed(speed_rpm);
}
} // namespace excavation

namespace deposition {
StepperMotor dep_mtr(DEP_MOTOR_ID);

void begin() {
  dep_mtr.begin();
}

void update(float &dep_curr) {
  dep_curr = dep_mtr.read_current();
}

float update_curr() {
  return dep_mtr.read_current();
}

void cb(int32_t speed_rpm) {
  dep_mtr.move_at_speed(-speed_rpm);
}

} // namespace deposition
