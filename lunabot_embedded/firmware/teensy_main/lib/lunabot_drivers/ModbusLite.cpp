#include <ModbusLite.hpp>

bool serial_has_started = false; // only begin serial once

struct FuncCodes { // these are modbus codes - we only use the ones below
  uint8_t ReadSingleReg = 0x03;
  uint8_t WriteSingleReg = 0x06;
} FuncCodes;

void modbus_CRC(uint8_t *buf, int data_range) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < data_range; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  buf[data_range] = (uint8_t)(crc & 0xFF);
  buf[data_range + 1] = (uint8_t)(crc >> 8);
}

void modbus_begin() {
  if (!serial_has_started) {
    serial_has_started = true;
    pinMode(RS485_TX_CONTROL, OUTPUT);
    digitalWrite(RS485_TX_CONTROL, RS485Receive); // Init Transceiver
    RS485Serial.begin(19200, SERIAL_8N1);
  }
}

// debug function to print out hex buffer
void print_hex_buffer(uint8_t *buf, int len) {
  for (int j = 0; j < len; j++) {
    Serial.print(buf[j], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// sends buf_to_send out on RS485, wait briefly for data to come back and read it into buf_to_read
// if MODBUS_DEBUG is defined, print information about the outgoing data
// returns size of data read
int modbus_send_rcv(uint8_t *buf_to_send, int len_to_send, uint8_t *buf_to_recv) {
  digitalWrite(RS485_TX_CONTROL, RS485Transmit); // Enable RS485 Transmit
  delay(10);                                     // Wait for transmit aquired
  RS485Serial.write(buf_to_send, len_to_send);   // Send data
  RS485Serial.flush();                           // Wait for data to send
  digitalWrite(RS485_TX_CONTROL, RS485Receive);  // Disable RS485 Transmit
  delay(10);                                     // Wait for data to come back in

  int recv_size = 0;
  while (RS485Serial.available()) {
    buf_to_recv[recv_size] = RS485Serial.read();
    recv_size += 1;
    if (recv_size == 32) {
      break; // panic if read in too much data to stop writing out of bounds
    }
  }

#ifdef MODBUS_DEBUG
  Serial.println("TX: ");
  print_hex_buffer(buf_to_send, len_to_send);
  Serial.println();
  Serial.println("RX: ");
  print_hex_buffer(buf_to_recv, recv_size);
  Serial.println('\n');
#endif

  return recv_size;
}

void modbus_write_register(uint8_t ID, uint16_t addr, uint16_t data) {
  uint8_t buf_to_send[8] = {ID, FuncCodes.WriteSingleReg, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF),
                            (uint8_t)(data >> 8), (uint8_t)(data & 0xFF), 0, 0};
  uint8_t buf_to_recv[32];
  modbus_CRC(buf_to_send, 6);
  modbus_send_rcv(buf_to_send, 8, buf_to_recv);
}

int modbus_read_register(uint8_t ID, uint16_t addr, uint16_t num_to_read) {
  uint8_t buf_to_send[8] = {ID, FuncCodes.ReadSingleReg, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF),
                            (uint8_t)(num_to_read >> 8), (uint8_t)(num_to_read & 0xFF), 0, 0}; // read one register

  uint8_t buf_to_recv[32];
  modbus_CRC(buf_to_send, 6);
  int recv_size = modbus_send_rcv(buf_to_send, 8, buf_to_recv);

  if (num_to_read == 1) {
    if (recv_size == 9) { // start + stop bits (2), device addr, func code, number of data bits, DATA [2], crc (2)
      return buf_to_recv[4] * 0x100 + buf_to_recv[5];
    } else {
      Serial.print("Error reading from single register - size mismatch. ");
      Serial.println(recv_size);
      return -1;
    }
  }

  else if (num_to_read == 2) {
    if (recv_size == 11) { // start + stop bits (2), device addr, func code, number of data bits, DATA [4], crc (2)
      return buf_to_recv[4] * 0x1000000 + buf_to_recv[5] * 0x10000 + buf_to_recv[6] * 0x100 + buf_to_recv[7];
    } else {
      Serial.println("Error reading from single register - size mismatch.");
      Serial.println(recv_size);
      return -1;
    }
  }

  else {
    Serial.println("Unexpected num_to_read");
    return -1;
  }
}