#include "interfaces.hpp"

// Stepper Motor Interfacing

StepperInterface::StepperInterface(uint8_t PWM1_P, uint8_t PWM2_P, uint8_t DIR1_P, uint8_t DIR2_P,
                                   int steps, int speed, int step_size)
    : PWM1_P_{PWM1_P}, PWM2_P_{PWM2_P}, DIR1_P_{DIR1_P}, DIR2_P_{DIR2_P}, en_{0}, steps_{steps},
      speed_{speed}, step_size_{step_size}, s_(steps, DIR1_P, DIR2_P) {
  pinMode(PWM1_P_, OUTPUT);
  pinMode(PWM2_P_, OUTPUT);
  s_.setSpeed(speed_);
}

void StepperInterface::step(StepperDir dir) {
  if (en_) {
    s_.step(step_size_ * dir);
  }
}

void StepperInterface::on() {
  digitalWrite(PWM1_P_, HIGH);
  digitalWrite(PWM2_P_, HIGH);
  en_ = 1;
}

void StepperInterface::off() {
  digitalWrite(PWM1_P_, LOW);
  digitalWrite(PWM2_P_, LOW);
  en_ = 0;
}

// PWM Motor Control Interfacing

MotorInterface::MotorInterface(uint8_t PWM_P, uint8_t DIR_P) : PWM_P_{PWM_P}, DIR_P_{DIR_P} {
  pinMode(DIR_P_, OUTPUT);
  pinMode(PWM_P_, OUTPUT);
}

void MotorInterface::write(uint8_t pwm, MotorDir dir) {
  digitalWrite(DIR_P_, dir);
  analogWrite(PWM_P_, pwm);
}

// Sabertooth MC Interfacing

int STMotorInterface::initialized_serial_ = 0;

STMotorInterface::STMotorInterface(Sabertooth *s, STMotor m) : st_{s}, motor_{m} {}

void STMotorInterface::init_serial(HardwareSerial &s, int baud_rate) {
  s.begin(baud_rate);
  initialized_serial_ = 1;
}
void STMotorInterface::write(int8_t power) {
  if (initialized_serial_) {
    power = min(power, 127);
    st_->motor(static_cast<byte>(motor_), power);
  }
}