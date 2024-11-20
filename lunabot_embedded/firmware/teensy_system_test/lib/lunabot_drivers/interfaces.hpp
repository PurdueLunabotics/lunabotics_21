#include <Arduino.h>
#include <Sabertooth.h>

#ifndef __INTERFACES_H__
#define __INTERFACES_H__

enum StepperDir { RETRACT = -1, EXTEND = 1 };
enum MotorDir { CW = HIGH, CCW = LOW };
enum STMotor { M1 = 1, M2 = 2 };

class StepperInterface {
public:
  StepperInterface(uint8_t PWM1_P, uint8_t PWM2_P, uint8_t DIR1_P, uint8_t DIR2_P, int steps,
                   int speed, int step_size);
  void on();
  void off();
  void step(StepperDir dir);

private:
  Stepper s_;
  uint8_t PWM1_P_;
  uint8_t PWM2_P_;
  uint8_t DIR1_P_;
  uint8_t DIR2_P_;
  int en_;
  int steps_;     // steps per revolution
  int speed_;     // steps per minute
  int step_size_; // step cnt per step
};

class MotorInterface {
public:
  MotorInterface(uint8_t PWM_P, uint8_t DIR_P);
  void write(uint8_t pwm, MotorDir dir);

private:
  uint8_t PWM_P_;
  uint8_t DIR_P_;
};

class STMotorInterface {
public:
  STMotorInterface(Sabertooth *st, STMotor motor);
  void write(int8_t pwm);
  static void init_serial(HardwareSerial &s, int baud_rate);

private:
  static int initialized_serial_;
  Sabertooth *st_;
  STMotor motor_;
};

#endif
