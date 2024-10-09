#ifndef __INTERFACES_H__
#define __INTERFACES_H__

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <HX711.h>
#include <RobotMsgs.pb.h>
#include "StepperLib.hpp"
#include "robot.hpp"

#define UWBSerial Serial4

enum MotorDir { CW = HIGH,
                CCW = LOW };
enum STMotor { M1 = 1,
               M2 = 2 };

class M5Stack_UWB_Trncvr {
public:
  M5Stack_UWB_Trncvr() {};
  static void init();
  static float read_uwb(uint8_t id);
  static void transfer();

private:
  static constexpr int NUM_UWB_TAGS = 3;
  volatile static float recv_buffer_[NUM_UWB_TAGS];
};

class KillSwitchRelay {
public:
  static bool dead;
  static long kill_time;

  static void init();

  // these two functions deal with the relay that kills all power to the motors
  static void reset();
  static void kill();

  // the main loop, to be run before the effort values are assigned to motors
  static void logic(RobotEffort &effort);

  // sets an individual motor to 0% power.
  static void disable_motor(int id, RobotEffort &effort);

private:
  // the pin to cut power to all motors. Active low to kill
  static constexpr int kill_pin = 9;
  static constexpr float drive_kill_curr = 7.0;
  static constexpr float exdep_kill_curr = 25.0;

  // the threshold at which the motor is set to 0% power
  static constexpr int cutoff_thresh = 1000;
  // the threshold at which a motor set a 0% power is allowed to turn back on
  static constexpr int reset_thresh = 500;

  // Every cycle that a motor is overcurrent, a counter increases by this amount (as well as decreasing by cutoff_decay)
  static constexpr int cutoff_increase = 3;
  // Every cycle, that counter decreases by this amount
  static constexpr int cutoff_decay = 1;

  // The relay that kills all motors must be dead for at least this long before resetting
  static constexpr int relay_dead_time = 2000;

  // If a motor has been set to 0% this many times, activate the kill relay.
  static constexpr int kill_thresh = 3;

  volatile static int cutoff_buffer[4];
  volatile static int disable_counter[4];
  volatile static bool is_disable[4];
};

#endif
