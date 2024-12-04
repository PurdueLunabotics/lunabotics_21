#include "interfaces.hpp"

// Sabertooth MC Interfacing

int Sabertooth_MotorCtrl::initialized_serial_ = 0;

Sabertooth_MotorCtrl::Sabertooth_MotorCtrl(Sabertooth *s, STMotor m) : st_{s}, motor_{m} {}

void Sabertooth_MotorCtrl::init_serial(HardwareSerial &s, int baud_rate) {
  s.begin(baud_rate);
  initialized_serial_ = 1;
}
void Sabertooth_MotorCtrl::write(int8_t power) {
  if (initialized_serial_) {
    power = min(power, 127);
    st_->motor(static_cast<byte>(motor_), power);
  }
}

// ---- Sensors ----

// Current Sensor

ADS1119Configuration ADS1119_Current_Bus::configurations[BUSES][MUXES] = {};
ADS1119 ADS1119_Current_Bus::ads1 = ADS1119(ads1_addr);
ADS1119 ADS1119_Current_Bus::ads2 = ADS1119(ads2_addr);

void ADS1119_Current_Bus::init_ads1119() {
  // init all channels to same general config, but with different mux config.
  // different configs for both chips for futureproofing, not because they're any different
  for (int i = 0; i < BUSES; ++i) {
    configurations[i][0].mux = ADS1119MuxConfiguration::positiveAIN0negativeAGND;
    configurations[i][1].mux = ADS1119MuxConfiguration::positiveAIN1negativeAGND;
    configurations[i][2].mux = ADS1119MuxConfiguration::positiveAIN2negativeAGND;
    configurations[i][3].mux = ADS1119MuxConfiguration::positiveAIN3negativeAGND;

    for (int j = 0; j < MUXES; ++j) {
      configurations[i][j].gain = ADS1119Configuration::Gain::one;
      configurations[i][j].dataRate = ADS1119Configuration::DataRate::sps330;
      configurations[i][j].conversionMode = ADS1119Configuration::ConversionMode::continuous;
      configurations[i][j].voltageReference = ADS1119Configuration::VoltageReferenceSource::external;
      configurations[i][j].externalReferenceVoltage = 3.3;
    }
  }

  ads1.begin();
  ads1.reset();

  ads2.begin();
  ads2.reset();
}

float ADS1119_Current_Bus::read(uint8_t bus, uint8_t mux) {
  switch (bus) {
  case 0:
    return ADS1119_Current_Bus::adc_to_current_31A(ads1.readVoltage(configurations[0][mux]));
  case 1:
    return ADS1119_Current_Bus::adc_to_current_31A(ads2.readVoltage(configurations[1][mux]));
  default:
    // we only have two muxes
    return 69.420;
  }
}

float ADS1119_Current_Bus::adc_to_current_31A(float adc_value, float adc_fsr, float vcc) {
  float vout = adc_value / pow(2, 2) * adc_fsr;
  return 73.3 * (vout / vcc) - 37.52;
}

volatile float M5Stack_UWB_Trncvr::recv_buffer_[NUM_UWB_TAGS] = {0};

float M5Stack_UWB_Trncvr::read_uwb(uint8_t id) {
  noInterrupts();
  float data = recv_buffer_[id];
  interrupts();
  return data;
}

void M5Stack_UWB_Trncvr::init() {
  UWBSerial.begin(115200);

  // serial 2 setup as tag on bot (does dist calc)
  for (int b = 0; b < 2; b++) { // Repeat twice to stabilize the connection
    delay(50);
    UWBSerial.write("AT+anchor_tag=0\r\n"); // Set up the Tag
    delay(50);
    UWBSerial.write("AT+interval=5\r\n");  // Set the calculation precision,
                                           // the larger the response is, the
                                           // slower it will be
    delay(50);                             // 设置计算精度，越大响应越慢
    UWBSerial.write("AT+switchdis=1\r\n"); // Began to distance 开始测距
    delay(50);
    if (b == 0) {
      UWBSerial.write("AT+RST\r\n"); // RESET 复位
    }
  }
  if (UWBSerial.available()) {
    delay(3);
  }
}

void M5Stack_UWB_Trncvr::transfer() {
  if (UWBSerial.available()) {
    // Read the original input
    String originalInput = UWBSerial.readStringUntil('\n');
    originalInput.trim();

    // Check if the message starts with "anX:" where X is a digit
    if (originalInput.startsWith("an") && isdigit(originalInput.charAt(2)) &&
        originalInput.charAt(3) == ':') {
      // Extract sensor number
      int sensorNumber = originalInput.charAt(2) - '0';

      // Find the index of the colon
      int colonIndex = originalInput.indexOf(':');

      // Extract the value part and trim spaces
      String valueString = originalInput.substring(colonIndex + 1);
      valueString.trim();

      // Convert the trimmed string to a float
      float num = valueString.toFloat();

      // Assign the values to d0, d1, or d2 based on sensor number
      M5Stack_UWB_Trncvr::recv_buffer_[sensorNumber] = num;
    }
  }
}

long KillSwitchRelay::kill_time;
bool KillSwitchRelay::dead;

void KillSwitchRelay::init() {
  pinMode(kill_pin, OUTPUT);
  KillSwitchRelay::dead = false;
  reset();
  KillSwitchRelay::kill_time = millis();
}

void KillSwitchRelay::reset() {
  digitalWrite(kill_pin, HIGH);
  KillSwitchRelay::dead = false;
}

void KillSwitchRelay::kill() {
  digitalWrite(kill_pin, LOW);
  KillSwitchRelay::kill_time = millis();
  KillSwitchRelay::dead = true;
}

void KillSwitchRelay::disable_motor(int id, RobotEffort &effort) {
  switch (id) {
  case 0:
    effort.excavate = 0;
    break;
  case 1:
    effort.deposit = 0;
    break;
  case 2:
    effort.left_drive = 0;
    break;
  case 3:
    effort.right_drive = 0;
    break;
  default:
    return;
  }
}

// exc, dep, drive_L, drive_R
volatile int KillSwitchRelay::cutoff_buffer[4] = {0};
volatile int KillSwitchRelay::disable_counter[4] = {0};
volatile bool KillSwitchRelay::is_disable[4] = {false};

void KillSwitchRelay::logic(RobotEffort &effort) {
  /*
  if (KillSwitchRelay::dead && millis() - KillSwitchRelay::kill_time >= relay_dead_time) {
    reset();
  } */

  float exc_curr = ADS1119_Current_Bus::adc_to_current_31A(excavation::update_curr());
  float dep_curr = ADS1119_Current_Bus::adc_to_current_31A(deposition::update_curr());
  float drive_left_curr = ADS1119_Current_Bus::adc_to_current_31A(drivetrain::update_curr_left());
  float drive_right_curr = ADS1119_Current_Bus::adc_to_current_31A(drivetrain::update_curr_right());

  if (exc_curr >= exdep_kill_curr) {
    cutoff_buffer[0] += cutoff_increase;
  }
  if (dep_curr >= exdep_kill_curr) {
    cutoff_buffer[1] += cutoff_increase;
  }
  if (drive_left_curr >= drive_kill_curr) {
    cutoff_buffer[2] += cutoff_increase;
  }
  if (drive_right_curr >= drive_kill_curr) {
    cutoff_buffer[3] += cutoff_increase;
  }

  for (int i = 0; i < 4; ++i) {
    cutoff_buffer[i] -= cutoff_decay;
    if (cutoff_buffer[i] < 0) {
      cutoff_buffer[i] = 0;
    }
    if (is_disable[i]) {
      if (cutoff_buffer[i] >= reset_thresh) {
        disable_motor(i, effort);
      } else {
        is_disable[i] = false;
      }
    } else {
      if (cutoff_buffer[i] >= cutoff_thresh) {
        disable_motor(i, effort);
        is_disable[i] = true;
        disable_counter[i] += 1;
      }
    }

    if (disable_counter[i] >= kill_thresh) {
      disable_counter[i] = 0;
      // TODO, send "all fucked" signal back to teensy
      kill();
    }
  }
}
