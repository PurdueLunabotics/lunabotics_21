#include <ros/ros.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <ctime>
#include <sys/ioctl.h>
#include <termios.h>

#include <lunabot_embedded/sensor_proc.h>
#include <lunabot_msgs/RobotEffort.h>
#include <lunabot_msgs/RobotSensors.h>

extern "C" {
#include "RobotMsgs.pb.h"
#include "hid.h"
#include "pb_decode.h"
#include "pb_encode.h"
}

using namespace std;

#define BUF_SIZE 64
#define MAX_ANGLE_DELTA_DEG 15

#define LEAKY_INTEGRATOR_ALPHA 0.6

uint8_t buf[BUF_SIZE];

RobotSensors prev_state = RobotSensors_init_zero;
RobotSensors state = RobotSensors_init_zero;
RobotEffort effort = RobotEffort_init_zero;
lunabot_msgs::RobotSensors prev_state_msg;

float prev_valid_drive_ang_left = 0;
float prev_valid_drive_ang_right = 0;
float prev_valid_exc_ang = 0;

RingBuffer<10> left_buffer;
RingBuffer<10> right_buffer;
RingBuffer<10> exc_buffer;

float prev_drive_left_vel = 0;
float prev_drive_right_vel = 0;
float prev_exc_vel = 0;

double prev_time = 0;

pb_ostream_t sizestream = {0};

void recv(ros::Publisher &pub) {
  int status;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(buf, sizeof(buf));
  /* Now we are ready to decode the message. */
  pb_decode(&stream, RobotSensors_fields, &state);
  lunabot_msgs::RobotSensors state_msg;

  state_msg.act_right_curr = state.act_right_curr;
  state_msg.drive_right_curr = state.drive_right_curr;
  state_msg.drive_left_curr = state.drive_left_curr;
  state_msg.dep_curr = state.dep_curr;
  state_msg.exc_curr = state.exc_curr;

  // TODO: add timestamp to each value using StampedValue, allowing for more accurate derivative
  // calculations:
  // https://github.com/PurdueLunabotics/purdue_lunabotics/pull/43#discussion_r1421183087
  angle_noise_rej_filter(&state.drive_left_ang, &prev_valid_drive_ang_left, MAX_ANGLE_DELTA_DEG);
  angle_noise_rej_filter(&state.drive_right_ang, &prev_valid_drive_ang_right, MAX_ANGLE_DELTA_DEG);

  double dt;
  dt = ros::Time::now().toSec() - prev_time;
  prev_time = ros::Time::now().toSec();

  float left_vel, right_vel, exc_vel;
  left_vel = deg_angle_delta(state.drive_left_ang, prev_state.drive_left_ang) / dt;
  right_vel = deg_angle_delta(state.drive_right_ang, prev_state.drive_right_ang) / dt;
  exc_vel = deg_angle_delta(state.exc_ang, prev_state.exc_ang) / dt;

  left_buffer.push(left_vel);
  right_buffer.push(right_vel);
  exc_buffer.push(exc_vel);

  left_vel = leaky_integrator(left_buffer.mean(), prev_drive_left_vel, LEAKY_INTEGRATOR_ALPHA);
  right_vel = leaky_integrator(right_buffer.mean(), prev_drive_right_vel, LEAKY_INTEGRATOR_ALPHA);
  exc_vel = leaky_integrator(exc_buffer.mean(), prev_exc_vel, LEAKY_INTEGRATOR_ALPHA);

  prev_state = state;
  prev_drive_left_vel = left_vel;
  prev_drive_right_vel = right_vel;
  prev_exc_vel = exc_vel;

  state_msg.drive_left_ang = DEG2RAD(state.drive_left_ang);
  state_msg.drive_right_ang = DEG2RAD(state.drive_right_ang);
  state_msg.drive_left_vel = DEG2RAD(left_vel);
  state_msg.drive_right_vel = DEG2RAD(right_vel);
  state_msg.exc_ang = DEG2RAD(state.exc_ang);
  state_msg.exc_vel = DEG2RAD(exc_vel);
  state_msg.uwb_dists.push_back(state.uwb_dist_0);
  state_msg.uwb_dists.push_back(state.uwb_dist_1);
  state_msg.uwb_dists.push_back(state.uwb_dist_2);
  state_msg.load_cell_weight = state.load_cell_weight;

  pub.publish(state_msg);
}

void effort_cb(const lunabot_msgs::RobotEffort &msg) {
  effort.lin_act = msg.lin_act;
  effort.left_drive = msg.left_drive;
  effort.right_drive = msg.right_drive;
  effort.excavate = msg.excavate;
  effort.deposit = msg.deposit;
}

void publish(const ros::TimerEvent &) {
  memset(buf, 0, sizeof(buf));
  pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
  pb_encode(&stream, RobotEffort_fields, &effort);
  rawhid_send(0, buf, 64, 0);
}

int main(int argc, char **argv) {
  int num_read_fails = 0;
  int i, r, num;

  r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
  if (r <= 0) {
    printf("no rawhid device found\n");
    return -1;
  }
  printf("found rawhid device\n");

  ros::init(argc, argv, "teensy_driver_node");
  ros::NodeHandle nh;

  ros::Subscriber effort_sub = nh.subscribe("/effort", 10, &effort_cb);
  ros::Publisher state_pub = nh.advertise<lunabot_msgs::RobotSensors>("/sensors", 10);

  ros::Rate rate(100);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish);

  while (ros::ok()) {
    // check if any Raw HID packet has arrived
    ros::spinOnce();
    num = rawhid_recv(0, buf, BUF_SIZE, 0);
    if (num < 0) {
      printf("error reading. Retrying connection\n");
      num_read_fails += 1;
      if (num_read_fails >= 5) {
        printf("Sorry, too many read errors. Giving up.\n");
        printf("sleeping\n!");
        // ros::Duration(1).sleep();
        r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
        printf("Reopened rawhid, r is %d\n", r);
        num_read_fails = 0;
      }
    }

    if (num > 0) {
      recv(state_pub);
    }

    rate.sleep();
  }
  rawhid_close(0);
}
