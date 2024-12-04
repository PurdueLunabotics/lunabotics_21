#ifndef MODBUSLITE_H
#define MODBUSLITE_H
#include <Arduino.h>

#define RS485_TX_CONTROL 23 // RS485 Direction control
#define RS485Serial Serial2 // TODO RJN - set these on the actual system
#define RS485Transmit HIGH
#define RS485Receive LOW

void modbus_begin();
void modbus_write_register(uint8_t ID, uint16_t addr, uint16_t data);
int modbus_read_register(uint8_t ID, uint16_t addr, uint16_t num_to_read);
#endif