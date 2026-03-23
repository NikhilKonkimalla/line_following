
#ifndef UTILS_H
#define UTILS_H

#include "motorgo_plink.h"

void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

#define DATA_MESSAGE_TYPE 0x02
#define BUFFER_OUT_SIZE 69
union data_out_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = DATA_MESSAGE_TYPE;

    // Encoder data from all 4 channels
    float channel_1_pos;
    float channel_1_vel;

    float channel_2_pos;
    float channel_2_vel;

    float channel_3_pos;
    float channel_3_vel;

    float channel_4_pos;
    float channel_4_vel;

    // IMU data
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;

    float mag_x;
    float mag_y;
    float mag_z;
  };

  uint8_t raw[BUFFER_OUT_SIZE];
};

enum class BrakeMode : uint8_t
{
  BRAKE,
  COAST
};

#define BUFFER_IN_SIZE 25
union data_in_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = DATA_MESSAGE_TYPE;

    // Motor data for all 4 channels
    float channel_1_command;
    float channel_2_command;
    float channel_3_command;
    float channel_4_command;

    // Control mode
    MotorGo::ControlMode channel_1_control_mode;
    MotorGo::ControlMode channel_2_control_mode;
    MotorGo::ControlMode channel_3_control_mode;
    MotorGo::ControlMode channel_4_control_mode;

    // Brake mode
    BrakeMode channel_1_brake_mode;
    BrakeMode channel_2_brake_mode;
    BrakeMode channel_3_brake_mode;
    BrakeMode channel_4_brake_mode;
  };

  uint8_t raw[BUFFER_IN_SIZE];
};

#define INIT_MESSAGE_TYPE 0x01
#define INIT_IN_SIZE 25
union init_input_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = INIT_MESSAGE_TYPE;

    // Target update frequency
    float frequency;
    float power_supply_voltage;
    float channel_1_voltage_limit;
    float channel_2_voltage_limit;
    float channel_3_voltage_limit;
    float channel_4_voltage_limit;
  };

  uint8_t raw[INIT_IN_SIZE];
};

#define INIT_OUT_SIZE 5
union init_output_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = INIT_MESSAGE_TYPE;

    uint16_t board_id;
    uint16_t firmware_version;
  };

  uint8_t raw[INIT_OUT_SIZE];
};

#define PID_CONTROLLER_MESSAGE_TYPE 0x03
// 5 floats, 2 uint8_t: 4 * 5 + 2 = 22
#define PID_CONTROLLER_FROM_CONTROLLER_SIZE 22
union pid_controller_from_controller_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = 0x03;
    uint8_t channel;
    float p;
    float i;
    float d;
    float output_ramp;
    float lpf;
  };

  uint8_t raw[PID_CONTROLLER_FROM_CONTROLLER_SIZE];
};

// Buffer out size is 69 bytes, pad with 4 bytes for DMA driver, and then
// round up to the nearest multiple of 4
#define BUFFER_SIZE (76)

void print_data_in(const data_in_t &data)
{
  String output = "";

  output += "Message type: " + String(data.message_type) + "\n";

  output += "Channel 1 Command: " + String(data.channel_1_command) + "\n";
  output += "Channel 1 Control Mode: " +
            String(static_cast<int>(data.channel_1_control_mode)) + "\n";
  output += "Channel 1 Brake Mode: " +
            String(static_cast<int>(data.channel_1_brake_mode)) + "\n";

  output += "Channel 2 Command: " + String(data.channel_2_command) + "\n";
  output += "Channel 2 Control Mode: " +
            String(static_cast<int>(data.channel_2_control_mode)) + "\n";
  output += "Channel 2 Brake Mode: " +
            String(static_cast<int>(data.channel_2_brake_mode)) + "\n";

  output += "Channel 3 Command: " + String(data.channel_3_command) + "\n";
  output += "Channel 3 Control Mode: " +
            String(static_cast<int>(data.channel_3_control_mode)) + "\n";
  output += "Channel 3 Brake Mode: " +
            String(static_cast<int>(data.channel_3_brake_mode)) + "\n";

  output += "Channel 4 Command: " + String(data.channel_4_command) + "\n";
  output += "Channel 4 Control Mode: " +
            String(static_cast<int>(data.channel_4_control_mode)) + "\n";
  output += "Channel 4 Brake Mode: " +
            String(static_cast<int>(data.channel_4_brake_mode)) + "\n";

  freq_println(output, 1);  // Adjust frequency as needed
}

#endif  // UTILS_H