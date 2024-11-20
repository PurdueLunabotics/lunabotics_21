#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "interfaces.hpp"
#include <Arduino.h>
#include <Sabertooth.h>

#define ST_SERIAL Serial1
#define ST_BAUD_RATE 9600

#ifdef OLD_CURRENT_SENSOR
extern ADS1115_lite adc0;
extern ADS1115_lite adc1;
#endif

// MCs
extern Sabertooth MC1; // top
extern Sabertooth MC2; // middle
extern Sabertooth MC3; // bottom

namespace actuation {
void cb(int8_t lin_act);
void update(float &);

} // namespace actuation

namespace drivetrain {
void cb(int8_t left, int8_t right);
void update(float &, float &, float &, float &);
float update_curr_left();
float update_curr_right();
} // namespace drivetrain

namespace uwb {
void update(float &d0, float &d1, float &d2);
} // namespace uwb
namespace load_cell {
void update(float &);
} // namespace load_cell

namespace deposition {
void cb(int8_t dep);
void update(float &);
float update_curr();
} // namespace deposition

namespace excavation {
void cb(int8_t exc);
void update(float &, float &);
float update_curr();
} // namespace excavation
#endif
