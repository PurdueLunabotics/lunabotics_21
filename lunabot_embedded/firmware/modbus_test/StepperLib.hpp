#ifndef STEPPERLIB_H
#define STEPPERLIB_H
#include <Arduino.h>
#include "ModbusLite.hpp"

#define USE_DEFAULT 0

class StepperMotor {
public:
  uint16_t def_speed;
  uint16_t def_acceleration;
  uint16_t def_deceleration;

  StepperMotor(uint8_t MotorID, uint16_t def_speed = 500, uint16_t def_acceleration = 250, uint16_t def_deceleration = 250);

  void begin();
  void write_estop();
  void move_at_speed(uint16_t speed, uint16_t acceleration = USE_DEFAULT, uint16_t deceleration = USE_DEFAULT);
  void move_to_pos(uint32_t position, bool absolute, uint16_t speed = USE_DEFAULT, uint16_t acceleration = USE_DEFAULT, uint16_t deceleration = USE_DEFAULT);

  int read_raw_velocity();
  int read_velocity();
  int read_torque();
  float read_current();
  int read_voltage();
  int read_temperature();
  int read_over_load_ratio();
  int read_regen_load_ratio();
  int read_motor_position_raw();
  float read_motor_position_radians();

  void print_motor_state();

private:
  uint8_t MotorID;
  void trigger_motion();
  void write_register(uint16_t address, uint16_t value);
  int read_register(uint16_t address, uint16_t num_to_read = 1);
};
#endif