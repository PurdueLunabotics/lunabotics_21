#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "interfaces.hpp"
#include <Arduino.h>
#include <Stepper.h>

#define ST_SERIAL Serial1
#define ST_BAUD_RATE 9600

// MCs
extern Sabertooth MC1; // top
extern Sabertooth MC2; // middle
extern Sabertooth MC3; // bottom

#define LEFT_DRIVE_MOTOR_ID 0x01 // TODO RJN - set these ids on robot
#define RIGHT_DRIVE_MOTOR_ID 0x02
#define EXC_MOTOR_ID 0x03
#define DEP_MOTOR_ID 0x04

namespace actuation {
void cb(int8_t lin_act);
void update(float &);

} // namespace actuation

namespace drivetrain {
void begin();
void cb(int32_t, int32_t);
void update(float &, float &, float &, float &);
float update_curr_left();
float update_curr_right();
} // namespace drivetrain

namespace uwb {
void update(float &d0, float &d1, float &d2);
} // namespace uwb

namespace deposition {
void begin();
void cb(int32_t);
void update(float &);
float update_curr();
} // namespace deposition

namespace excavation {
void begin();
void cb(int32_t);
void update(float &, float &);
float update_curr();
} // namespace excavation
#endif
