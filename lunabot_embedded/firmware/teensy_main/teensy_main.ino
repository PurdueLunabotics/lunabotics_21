// include ROS and all messages
#include <RobotMsgs.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "robot.hpp"
#include "interfaces.hpp"

#define TX_PERIOD 10               // ms
#define CTRL_PERIOD 2              // ms
#define UWB_TRANSFER_PERIOD 10'000 // microsec
#define CURR_UPDATE_PERIOD 8       // ms
#define STALE_EFFORT_PERIOD 1000   // ms

RobotSensors state = RobotSensors_init_zero;
RobotEffort effort = RobotEffort_init_zero;
size_t effort_msg_size;

uint8_t buffer[64];
uint8_t flags = 0;

void ctrl() {
  actuation::cb(effort.lin_act);
  drivetrain::cb(effort.left_drive, effort.right_drive);
  deposition::cb(effort.deposit);
  excavation::cb(effort.excavate);
}

void send() {
  actuation::update(state.act_right_curr);
  drivetrain::update(state.drive_left_curr, state.drive_right_curr, state.drive_left_ang,
                     state.drive_right_ang);
  deposition::update(state.dep_curr);

  excavation::update(state.exc_curr, state.exc_ang);

  uwb::update(state.uwb_dist_0, state.uwb_dist_1, state.uwb_dist_2);

  /*
  Serial.print("Raw: ");
  Serial.print(state.act_right_curr);
  Serial.print(" To_curr: ");
  Serial.println(ADS1119_Current_Bus::adc_to_current_31A(state.act_right_curr));
  */
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  pb_encode(&stream, RobotSensors_fields, &state);
}

IntervalTimer uwb_timer;
float last_effort;

void setup() {
  // Serial.begin(115200);
  Sabertooth_MotorCtrl::init_serial(ST_SERIAL, ST_BAUD_RATE);

  M5Stack_UWB_Trncvr::init();
  KillSwitchRelay::init();

  ADS1119_Current_Bus::init_ads1119();

  uwb_timer.begin(M5Stack_UWB_Trncvr::transfer, UWB_TRANSFER_PERIOD);

  drivetrain::begin();
  excavation::begin();
  deposition::begin();

  // disable timeout
  MC1.setTimeout(0);
  // set to fast ramp (1-10 - fast, 11-20 slow, 20-80 intermed)
  MC1.setRamping(1);

  last_effort = millis();
}

elapsedMillis ms_until_send;
elapsedMillis ms_until_ctrl;
elapsedMillis ms_curr_update;

void loop() {

  int n;
  n = RawHID.recv(buffer, 0); // 0 timeout = do not wait
  if (n > 0) {
    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));

    /* Now we are ready to decode the message. */
    pb_decode(&stream, RobotEffort_fields, &effort);
    last_effort = millis();
  }

  if (millis() - last_effort > STALE_EFFORT_PERIOD) {
    effort = RobotEffort_init_zero;
  }

  if (ms_until_ctrl > CTRL_PERIOD) {
    ms_until_ctrl -= CTRL_PERIOD;
    // TODO, add timer if robot effort not changing for too long, exit?
    // KillSwitchRelay::logic(effort);
    ctrl();
  }

  if (ms_until_send > TX_PERIOD) {
    ms_until_send -= TX_PERIOD;
    send();
    n = RawHID.send(buffer, 0);
  }
}
