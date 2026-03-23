#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <ESP32DMASPISlave.h>
#include <Preferences.h>
#include <Wire.h>

#include "SPI.h"
#include "motorgo_plink.h"
#include "utils.h"

// IMU objects
Adafruit_LSM6DS33 lsm6ds;
Adafruit_LIS3MDL lis3mdl;

MotorGo::MotorGoPlink plink;
MotorGo::MotorChannel &ch1 = plink.ch1;
MotorGo::MotorChannel &ch2 = plink.ch2;
MotorGo::MotorChannel &ch3 = plink.ch3;
MotorGo::MotorChannel &ch4 = plink.ch4;

MotorGo::ChannelConfiguration ch1_config;
MotorGo::ChannelConfiguration ch2_config;
MotorGo::ChannelConfiguration ch3_config;
MotorGo::ChannelConfiguration ch4_config;

// Pin the MotorGo uses to indicate that it is ready to transfer new data
#define DATA_READY 36

// SPI communication setup
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *tx_buf;
uint8_t *rx_buf;

data_out_t data_out;
data_in_t data_in;

ESP32DMASPI::Slave slave;

// Time to delay between data transmissions
unsigned long delay_time = 0;
unsigned long last_data_time = 0;
unsigned long loop_start_time = 0;

bool communication_active = true;

void init_spi_comms()
{
  bool ready = false;

  //   Prepare the initialize data
  init_output_t init_out;

  //   Load board ID from preferences, 16-bit unsigned integer
  Preferences preferences;
  preferences.begin("__mg", true);
  init_out.board_id = preferences.getUShort("__mg_id", 0);
  preferences.end();

  init_out.firmware_version = VERSION_HASH;

  while (!ready)
  {
    // Copy data to tx buffer
    memcpy(tx_buf, init_out.raw, INIT_OUT_SIZE);

    //  Indicate that data is ready
    digitalWrite(DATA_READY, HIGH);

    //  Send the data
    const size_t received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    //  Indicate that data is not ready
    digitalWrite(DATA_READY, LOW);
    delay(100);

    init_input_t init_in;
    if (received_bytes != 0)
    {
      // Decode data into data_in_t
      memcpy(init_in.raw, rx_buf, INIT_IN_SIZE);

      if (init_in.message_type == INIT_MESSAGE_TYPE)
      {
        ready = true;

        if (init_in.frequency == 0)
        {
          init_in.frequency = 1;
        }

        // Delay time in microseconds
        delay_time = 1000000 / init_in.frequency;

        // Set up motor channels
        ch1_config.power_supply_voltage = init_in.power_supply_voltage;
        ch2_config.power_supply_voltage = init_in.power_supply_voltage;
        ch3_config.power_supply_voltage = init_in.power_supply_voltage;
        ch4_config.power_supply_voltage = init_in.power_supply_voltage;

        ch1_config.voltage_limit = init_in.channel_1_voltage_limit;
        ch2_config.voltage_limit = init_in.channel_2_voltage_limit;
        ch3_config.voltage_limit = init_in.channel_3_voltage_limit;
        ch4_config.voltage_limit = init_in.channel_4_voltage_limit;

        ch1.init(ch1_config);
        ch2.init(ch2_config);
        ch3.init(ch3_config);
        ch4.init(ch4_config);
      }
    }
  }

  loop_start_time = micros();

  Serial.println("SPI communication initialized");
}

MotorGo::MotorChannel *channels[] = {&ch1, &ch2, &ch3, &ch4};
void update_pid_controller(pid_controller_from_controller_t pid_data)
{
  // Confim that channel is between 1 and 4
  if (pid_data.channel < 1 || pid_data.channel > 4)
  {
    return;
  }
  //   0-indexed channel
  pid_data.channel--;

  MotorGo::PIDParameters pid_params = {
      .p = pid_data.p,
      .i = pid_data.i,
      .d = pid_data.d,
      .output_ramp = pid_data.output_ramp,
      .lpf_time_constant = pid_data.lpf,
  };

  channels[pid_data.channel]->set_velocity_controller(pid_params);

  // Re-enable the channel, in case it was disabled due to mis-configured
  // controller
  channels[pid_data.channel]->enable();
}

void setup()
{
  Serial.begin(115200);

  Wire1.begin(HIDDEN_SDA, HIDDEN_SCL, 400000);

  pinMode(DATA_READY, OUTPUT);
  digitalWrite(DATA_READY, LOW);

  bool lsm6ds_success = lsm6ds.begin_I2C(0x6a, &Wire1);
  bool lis3mdl_success = lis3mdl.begin_I2C(0x1C, &Wire1);

  //   Allocate buffers
  tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  // Restart if IMU initialization fails
  if (!lsm6ds_success || !lis3mdl_success)
  {
    Serial.println("IMU initialization failed, restarting!");
    ESP.restart();
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true,  // enable z axis
                          true,                // polarity
                          false,               // don't latch
                          true);               // enabled!

  slave.setDataMode(SPI_MODE3);    // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE);  // default: 1
  slave.setMaxTransferSize(BUFFER_SIZE);

  slave.begin(FSPI, SCK, MISO, MOSI, SS);

  Serial.println(BUFFER_IN_SIZE);
  Serial.println(BUFFER_OUT_SIZE);

  Serial.println("start spi slave");

  init_spi_comms();

  ch1.set_control_mode(MotorGo::ControlMode::Voltage);
  ch2.set_control_mode(MotorGo::ControlMode::Voltage);
  ch3.set_control_mode(MotorGo::ControlMode::Voltage);
  ch4.set_control_mode(MotorGo::ControlMode::Voltage);

  ch1.enable();
  ch2.enable();
  ch3.enable();
  ch4.enable();
}

void loop()
{
  sensors_event_t accel, gyro, mag, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  data_out.channel_1_pos = ch1.get_position();
  data_out.channel_1_vel = ch1.get_velocity();

  data_out.channel_2_pos = ch2.get_position();
  data_out.channel_2_vel = ch2.get_velocity();

  data_out.channel_3_pos = ch3.get_position();
  data_out.channel_3_vel = ch3.get_velocity();

  data_out.channel_4_pos = ch4.get_position();
  data_out.channel_4_vel = ch4.get_velocity();

  data_out.gyro_x = gyro.gyro.x;
  data_out.gyro_y = gyro.gyro.y;
  data_out.gyro_z = gyro.gyro.z;

  data_out.accel_x = accel.acceleration.x;
  data_out.accel_y = accel.acceleration.y;
  data_out.accel_z = accel.acceleration.z;

  data_out.mag_x = mag.magnetic.x;
  data_out.mag_y = mag.magnetic.y;
  data_out.mag_z = mag.magnetic.z;

  // Copy data to tx buffer
  memcpy(tx_buf, data_out.raw, BUFFER_OUT_SIZE);

  digitalWrite(DATA_READY, HIGH);

  const size_t received_bytes =
      slave.transfer(tx_buf, rx_buf, BUFFER_SIZE, 100);

  digitalWrite(DATA_READY, LOW);

  if (received_bytes != 0)
  {
    // Check first byte for message type and handle accordingly
    uint8_t message_type = rx_buf[0];
    switch (message_type)
    {
      case DATA_MESSAGE_TYPE:
      {
        // Decode data into data_in_t
        memcpy(data_in.raw, rx_buf, BUFFER_IN_SIZE);

        // Skip brake/coast mode for now

        if (ch1.get_control_mode() != data_in.channel_1_control_mode)
        {
          ch1.set_control_mode(data_in.channel_1_control_mode);
          ch1.enable();
        }

        if (ch2.get_control_mode() != data_in.channel_2_control_mode)
        {
          ch2.set_control_mode(data_in.channel_2_control_mode);
          ch2.enable();
        }

        if (ch3.get_control_mode() != data_in.channel_3_control_mode)
        {
          ch3.set_control_mode(data_in.channel_3_control_mode);
          ch3.enable();
        }

        if (ch4.get_control_mode() != data_in.channel_4_control_mode)
        {
          ch4.set_control_mode(data_in.channel_4_control_mode);
          ch4.enable();
        }

        if (data_in.channel_1_control_mode == MotorGo::ControlMode::Velocity)
        {
          ch1.set_target_velocity(data_in.channel_1_command);
        }
        else if (data_in.channel_1_control_mode == MotorGo::ControlMode::None)
        {
          ch1.disable();
        }
        else
        {
          ch1.set_power(data_in.channel_1_command);
        }

        if (data_in.channel_2_control_mode == MotorGo::ControlMode::Velocity)
        {
          ch2.set_target_velocity(data_in.channel_2_command);
        }
        else if (data_in.channel_2_control_mode == MotorGo::ControlMode::None)
        {
          ch2.disable();
        }
        else
        {
          ch2.set_power(data_in.channel_2_command);
        }

        if (data_in.channel_3_control_mode == MotorGo::ControlMode::Velocity)
        {
          ch3.set_target_velocity(data_in.channel_3_command);
        }
        else if (data_in.channel_3_control_mode == MotorGo::ControlMode::None)
        {
          ch3.disable();
        }
        else
        {
          ch3.set_power(data_in.channel_3_command);
        }

        if (data_in.channel_4_control_mode == MotorGo::ControlMode::Velocity)
        {
          ch4.set_target_velocity(data_in.channel_4_command);
        }
        else if (data_in.channel_4_control_mode == MotorGo::ControlMode::None)
        {
          ch4.disable();
        }
        else
        {
          ch4.set_power(data_in.channel_4_command);
        }

        last_data_time = millis();
        break;
      }
      case PID_CONTROLLER_MESSAGE_TYPE:
      {
        // Decode data into pid_controller_from_controller_t
        pid_controller_from_controller_t pid_data;
        memcpy(pid_data.raw, rx_buf, PID_CONTROLLER_FROM_CONTROLLER_SIZE);

        update_pid_controller(pid_data);

        break;
      }
    };
  }

  if (millis() - last_data_time > 1000)
  {
    ch1.disable();
    ch2.disable();
    ch3.disable();
    ch4.disable();

    communication_active = false;
  }
  //   Re-enable if data is received
  else if (!communication_active)
  {
    ch1.enable();
    ch2.enable();
    ch3.enable();
    ch4.enable();

    communication_active = true;
  }

  // Delay to maintain desired frequency
  unsigned long loop_duration = micros() - loop_start_time;
  unsigned long corrected_delay_time = 0;
  if (loop_duration > delay_time)
  {
    // Serial.println("Loop duration exceeded delay time: ");
  }
  else
  {
    corrected_delay_time = delay_time - loop_duration;
  }

  delayMicroseconds(corrected_delay_time);

  loop_start_time = micros();
}
