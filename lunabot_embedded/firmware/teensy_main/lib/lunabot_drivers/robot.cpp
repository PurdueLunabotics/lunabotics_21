#include <robot.hpp>

// sensor wire documentation 2022-2023:
// https://docs.google.com/spreadsheets/d/1eX79YtawJqBA8VePFFtKJT6RR21gvK4qH1DKJX_vJE0/edit#gid=0

// MCs
Sabertooth MC1(128, ST_SERIAL); // DRIVE
Sabertooth MC2(130, ST_SERIAL); // EXCAVATON
Sabertooth MC3(131, ST_SERIAL); // DEPOSIT

// ADCs
namespace actuation {

Sabertooth_MotorCtrl act_right_mtr{&MC3, STMotor::M2};
Sabertooth_MotorCtrl act_left_mtr{&MC2, STMotor::M2};

constexpr uint8_t ACT_RIGHT_CURR_ADC = 0;
constexpr uint8_t ACT_RIGHT_CURR_MUX = 1; // U6 curr_sense_board

constexpr uint8_t ACT_LEFT_CURR_ADC = 1;
constexpr uint8_t ACT_LEFT_CURR_MUX = 1; // U4 curr_sense_board

void update(float &act_right_curr) {
#ifdef OLD_CURRENT_SENSOR
  act_right_curr = ACS711_Current_Bus::read(ACT_RIGHT_CURR_ADC, ACT_RIGHT_CURR_MUX);
#else
  act_right_curr = ADS1119_Current_Bus::read(ACT_RIGHT_CURR_ADC, ACT_RIGHT_CURR_MUX);
#endif
}

void cb(int8_t lin_act_volt) {
  act_left_mtr.write(lin_act_volt);
  act_right_mtr.write(lin_act_volt);
}

} // namespace actuation

namespace drivetrain {
Sabertooth_MotorCtrl left_drive_mtr{&MC1, STMotor::M2};
Sabertooth_MotorCtrl right_drive_mtr{&MC1, STMotor::M1};

constexpr uint8_t LEFT_CURR_ADC = 0;
constexpr uint8_t LEFT_CURR_MUX = 2; // U5 curr_sense_board

constexpr uint8_t RIGHT_CURR_ADC = 0;
constexpr uint8_t RIGHT_CURR_MUX = 3; // U2 curr_sense_board

constexpr uint8_t RIGHT_ENC_ID = 0;
constexpr uint8_t LEFT_ENC_ID = 1;

void update(float &left_curr, float &right_curr, float &left_angle, float &right_angle) {
#ifdef OLD_CURRENT_SENSOR
  left_curr = ACS711_Current_Bus::read(LEFT_CURR_ADC, LEFT_CURR_MUX);
  right_curr = ACS711_Current_Bus::read(RIGHT_CURR_ADC, RIGHT_CURR_MUX);
#else
  left_curr = ADS1119_Current_Bus::read(LEFT_CURR_ADC, LEFT_CURR_MUX);
  right_curr = ADS1119_Current_Bus::read(RIGHT_CURR_ADC, RIGHT_CURR_MUX);
#endif

  left_angle = AMT13_Angle_Bus::read_enc(LEFT_ENC_ID);
  right_angle = -AMT13_Angle_Bus::read_enc(RIGHT_ENC_ID);
}

float update_curr_left() {
#ifdef OLD_CURRENT_SENSOR
  return ACS711_Current_Bus::read(LEFT_CURR_ADC, LEFT_CURR_MUX);
#else
  return ADS1119_Current_Bus::read(LEFT_CURR_ADC, LEFT_CURR_MUX);
#endif
}

float update_curr_right() {
#ifdef OLD_CURRENT_SENSOR
  return ACS711_Current_Bus::read(RIGHT_CURR_ADC, RIGHT_CURR_MUX);
#else
  return ADS1119_Current_Bus::read(RIGHT_CURR_ADC, RIGHT_CURR_MUX);
#endif
}

void cb(int8_t left_drive_volt, int8_t right_drive_volt) {
  // Tank drive steering
  left_drive_mtr.write(left_drive_volt);
  right_drive_mtr.write(right_drive_volt);
}

} // namespace drivetrain

namespace uwb {
void update(float &d0, float &d1, float &d2) {
  d0 = M5Stack_UWB_Trncvr::read_uwb(0);
  d1 = M5Stack_UWB_Trncvr::read_uwb(1);
  d2 = M5Stack_UWB_Trncvr::read_uwb(2);
}
} // namespace uwb
namespace load_cell {
void update(float &d0) {
  float val1 = HX711_Bus::read_scale(0);
  float val2 = HX711_Bus::read_scale(1);
  if (val1 != -1 && val2 != -1) {
    d0 = (val1 - 8143500) / -33.1;
  } else if (val1 != -1) {
    d0 = (val1 - 8143500) / -33.1;
  } else if (val2 != -1) {
    d0 = val2;
  }
}
} // namespace load_cell

namespace excavation {
Sabertooth_MotorCtrl exc_mtr{&MC2, STMotor::M1};

constexpr uint8_t EXC_CURR_ADC = 0;
constexpr uint8_t EXC_CURR_MUX = 0; // U1 curr_sense_board

constexpr uint8_t EXC_ENC_ID = 2;

void update(float &exc_curr, float &exc_angle) {
#ifdef OLD_CURRENT_SENSOR
  exc_curr = ACS711_Current_Bus::read(EXC_CURR_ADC, EXC_CURR_MUX);
#else
  exc_curr = ADS1119_Current_Bus::read(EXC_CURR_ADC, EXC_CURR_MUX);
#endif

  exc_angle = -AMT13_Angle_Bus::read_enc(EXC_ENC_ID);
}

float update_curr() {
#ifdef OLD_CURRENT_SENSOR
  return ACS711_Current_Bus::read(EXC_CURR_ADC, EXC_CURR_MUX);
#else
  return ADS1119_Current_Bus::read(EXC_CURR_ADC, EXC_CURR_MUX);
#endif
}

void cb(int8_t speed) { exc_mtr.write(speed); }
} // namespace excavation

namespace deposition {
Sabertooth_MotorCtrl dep_mtr{&MC3, STMotor::M1};
// Sabertooth_MotorCtrl exc_mtr{&MC3, STMotor::M1};

constexpr uint8_t DEP_CURR_ADC = 1;
constexpr uint8_t DEP_CURR_MUX = 0; // U3 curr_sense_board

void update(float &dep_curr) {
#ifdef OLD_CURRENT_SENSOR
  dep_curr = ACS711_Current_Bus::read(DEP_CURR_ADC, DEP_CURR_MUX);
#else
  dep_curr = ADS1119_Current_Bus::read(DEP_CURR_ADC, DEP_CURR_MUX);
#endif
}

float update_curr() {
#ifdef OLD_CURRENT_SENSOR
  return ACS711_Current_Bus::read(DEP_CURR_ADC, DEP_CURR_MUX);
#else
  return ADS1119_Current_Bus::read(DEP_CURR_ADC, DEP_CURR_MUX);
#endif
}

void cb(int8_t volt) { dep_mtr.write(-volt); }
// exc_mtr.write(volt); }

} // namespace deposition
