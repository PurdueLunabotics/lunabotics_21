#include "StepperLib.hpp"

// HOW TO SETUP MOTOR
// set ID using rotating dial
// set DIP switch 1 to on to set baud rate
// set DIP switch 3 to on on the final motor to enable resistor

// USER MANUAL HERE:
// https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=3354/User%20Manual%20Of%20iSV2-RS.pdf

// TODO RJN - deal with alarms and error codes

struct Addrs {             // addrs of various motor params
  uint16_t Reset = 0x0033; // reset alarms or the entire motor
  // there is a bunch of PID stuff here - probably easier to tune with motionstudio
  uint16_t InternalEnable = 0x0405; // use to enable servo (must set before sending movement commands)
  uint16_t RS485_Mode = 0x053B;     // *SET TO 4* to enable Serial_8N1 (see manual page 42)
  uint16_t RS485_BAUD = 0x053D;     // USE SWITCHES INSTEAD - BAUD SHOULD BE 19200
  uint16_t RS485_ID = 0x053F;       // USE DIAL INSTEAD - different for each motor
  // there is some stuff with regen breaking here - could be interesting for power saving

  // TIME FOR THE STUFF WE WANT TO READ:
  // software version stuff
  uint16_t Read_ErrorCode = 0x0B03;      // error code
  uint16_t Read_MotorState = 0x0B05;     // see page 53
  uint16_t Read_RawVelocity = 0x0B06;    // rpm - unfilitered
  uint16_t Read_Torque = 0x0B07;         // % of rated
  uint16_t Read_Current = 0x0B08;        // 0.01A
  uint16_t Read_Velocity = 0x0B09;       // rpm
  uint16_t Read_Voltage = 0x0B0A;        // V
  uint16_t Read_Temperature = 0x0B0B;    // deg C
  uint16_t Read_OverLoadRatio = 0x0B0F;  // %
  uint16_t Read_RegenLoadRatio = 0x0B10; // %
  // digital input stuff
  // motor position stuff
  uint16_t Read_MotorPosition = 0x602c; // unit: pulses - 32 bit! (low is +1)

  uint16_t ControlReg = 0x6002;

  uint16_t PathMode = 0x6200;
  uint16_t Pos = 0x6201; // high - 32 bit! (low is +1)
  uint16_t Speed = 0x6203;
  uint16_t Acceleration = 0x6204;
  uint16_t Deceleration = 0x6205;

} Addrs;

struct ControlCmds {
  uint8_t TRIG_PATH = 0x10;
  uint8_t ESTOP = 0x40;
  uint8_t ENABLE = 0x83;
  uint16_t AbsolutePosMode = 0x0001;
  uint16_t RelativePosMode = 0x0041;
  uint16_t VelocityMode = 0x0002;
} ControlCmds;

// Create a StepperMotor object
// MotorID and baudrate are required
// Optional default values for speed, acceleration, deceleration
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
StepperMotor::StepperMotor(uint8_t MotorID, uint16_t def_speed, uint16_t def_acceleration, uint16_t def_deceleration) {
  this->MotorID = MotorID;
  this->def_speed = def_speed;
  this->def_acceleration = def_acceleration;
  this->def_deceleration = def_deceleration;
}

void StepperMotor::begin() {
  modbus_begin();
  write_register(Addrs.InternalEnable, ControlCmds.ENABLE);
}

// emergency stops motor
void StepperMotor::write_estop() {
  write_register(Addrs.ControlReg, ControlCmds.ESTOP);
}

// move the motor at constant velocity, using default values for acceleration and deceleration unless specified
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void StepperMotor::move_at_speed(uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  if (acceleration == USE_DEFAULT) {
    acceleration = def_acceleration;
  }
  if (deceleration == USE_DEFAULT) {
    deceleration = def_deceleration;
  }

  write_register(Addrs.PathMode, ControlCmds.VelocityMode);
  write_register(Addrs.Speed, speed);
  write_register(Addrs.Acceleration, acceleration);
  write_register(Addrs.Deceleration, deceleration);
  trigger_motion();
}

// move the motor to a position, using default values for speed, acceleration, and deceleration unless specified
// position in encoder pulses
// absolute (TRUE = absolute, FALSE = relative)
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void StepperMotor::move_to_pos(uint32_t position, bool absolute, uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  if (speed == USE_DEFAULT) {
    speed = def_speed;
  }
  if (acceleration == USE_DEFAULT) {
    acceleration = def_acceleration;
  }
  if (deceleration == USE_DEFAULT) {
    deceleration = def_deceleration;
  }

  if (absolute) {
    write_register(Addrs.PathMode, ControlCmds.AbsolutePosMode);
  } else {
    write_register(Addrs.PathMode, ControlCmds.RelativePosMode);
  }

  uint16_t pos_high = (uint16_t)(position >> 16);
  uint16_t pos_low = (uint16_t)(position & 0xFFFF);

  write_register(Addrs.Pos, pos_high);
  write_register(Addrs.Pos + 1, pos_low);
  write_register(Addrs.Speed, speed);
  write_register(Addrs.Acceleration, acceleration);
  write_register(Addrs.Deceleration, deceleration);
  trigger_motion();
}

// internal function to start motion path (see motor docs for more info)
void StepperMotor::trigger_motion() {
  write_register(Addrs.ControlReg, ControlCmds.TRIG_PATH);
}

void StepperMotor::write_register(uint16_t addr, uint16_t data) {
  modbus_write_register(MotorID, addr, data);
}

int StepperMotor::read_register(uint16_t addr, uint16_t num_to_read) {
  return modbus_read_register(MotorID, addr, num_to_read);
}

// rpm - unfilitered
int StepperMotor::read_raw_velocity() {
  return (int16_t)read_register(Addrs.Read_RawVelocity);
}

// rpm
int StepperMotor::read_velocity() {
  return (int16_t)read_register(Addrs.Read_Velocity);
}

// % of rated
int StepperMotor::read_torque() {
  return read_register(Addrs.Read_Torque);
}

// amps
float StepperMotor::read_current() {
  return read_register(Addrs.Read_Current) * 0.01;
}

// volts
int StepperMotor::read_voltage() {
  return read_register(Addrs.Read_Voltage);
}

// deg C
int StepperMotor::read_temperature() {
  return read_register(Addrs.Read_Temperature);
}

// %
int StepperMotor::read_over_load_ratio() {
  return read_register(Addrs.Read_OverLoadRatio);
}

// %
int StepperMotor::read_regen_load_ratio() {
  return read_register(Addrs.Read_RegenLoadRatio);
}

// pulses
int StepperMotor::read_motor_position_raw() {
  return read_register(Addrs.Read_MotorPosition, 2); // this is a 2 byte value
}

// radians
float StepperMotor::read_motor_position_radians() {
  return read_motor_position_raw() / 10000.0 * 2 * PI; // 2500 pulses per rotation, * 2 * PI
}

// print the motor state from the state register
void StepperMotor::print_motor_state() {
  switch (read_register(Addrs.Read_MotorState)) {
  case 0:
    Serial.println("Motor State: Ready");
    break;
  case 1:
    Serial.println("Motor State: Run");
    break;
  case 2:
    Serial.println("Motor State: Error");
    break;
  case 3:
    Serial.println("Motor State: Home OK");
    break;
  case 4:
    Serial.println("Motor State: at pos");
    break;
  case 5:
    Serial.println("Motor State: at speed");
    break;
  default:
    Serial.println("Motor State: ???");
    break;
  }
}