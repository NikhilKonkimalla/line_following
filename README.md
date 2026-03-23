<h1 align="center">📘 | CMU 16-311: MotorGo Plink API Reference | 📘 </h1>

<p align="center">
    Complete guide to using all of the features of the MotorGo Plink Python API
</p>

---


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Getting Started](#getting-started)
- [Accessing Motor Channels](#accessing-motor-channels)
- [Reading Encoder Data](#reading-encoder-data)
- [Controlling Motors](#controlling-motors)
  - [Power Control Mode](#power-control-mode)
  - [Velocity Control Mode](#velocity-control-mode)
- [Reading IMU Data](#reading-imu-data)
- [Reading Light and Range Sensor Data](#reading-light-and-range-sensor-data)
- [Examples](#examples)


## Getting Started


To start using the MotorGo Python API, first import your MotorGo board from the `motorgo` module:

```python filename="python"
from motorgo import Plink
```

Next, create an instance of the board you imported:

```python filename="python"
plink = Plink()
```

Optionally, you can configure the communication parameter for the board:

```python filename="python"
plink = Plink(frequency = 200, timeout = 1.0)
```

Frequency defines the frequency of data transmission to the MotorGo board. Reducing the frequency results in less CPU usage on the MotorGo and Pi. The frequency may not be stable above 200 Hz. Timeout defines the time in seconds to wait for a response from the MotorGo board before attempting to reset communications.


To begin communication, run the following command:
```python filename="python"
plink.connect()
```

## Accessing Motor Channels
The `MotorChannel` objects are the primary interface for controlling the motors and reading their corresponding encoder data. The `MotorChannel` objects are accessible as:

```python filename="python"
plink.channel1
plink.channel2
plink.channel3
plink.channel4
```

You can save a reference to the `MotorChannel` objects as local variables for convenience. For example:

```python filename="python"
left_drive_wheel = plink.channel1
right_drive_wheel = plink.channel2
```

## Reading Encoder Data

The encoder data can be accessed from the `MotorChannel` objects in the `Plink` object. The `MotorChannel` objects are accessible as:

To read the encoder data, simply access the `position` and `velocity` attributes of the `MotorChannel` object:

```python filename="python"
left_position = left_drive_wheel.position # Position, in radians
left_velocity = left_drive_wheel.velocity # Velocity, in radians/sec

channel_3_position = plink.channel3.position
channel_3_velocity = plink.channel3.velocity

print(f"Left wheel: {left_position}\t {left_velocity}")
print(f"Channel 3: {channel_3_position}\t {channel_3_velocity}")
```

Note that the forward direction of the position is dependent on the direction that the motor is connected to the MotorGo and the direction the encoder is connected to the motor.

## Controlling Motors

The Plink supports two motor control modes: `ControlMode.POWER` and `ControlMode.VELOCITY`.

### Power Control Mode
In power control mode, the motor is controlled by directly setting the power level. The power level is a value between -1.0 and 1.0, where -1.0 is full reverse, 0.0 is stopped, and 1.0 is full forward. To configure a motor channel to power control mode, set the `control_mode` attribute of the `MotorChannel` object to `ControlMode.POWER`.

```python filename="python"
from motorgo import ControlMode

left_drive_wheel.control_mode = ControlMode.POWER
```

To set the power level of the motor, set the `power` attribute of the `MotorChannel` object to a value between -1.0 and 1.0.

```python filename="python"
left_drive_wheel.power = 0.5
```

### Velocity Control Mode
In velocity control mode, the motor is controlled by setting the desired velocity in rad/s. To configure a motor channel to velocity control mode, set the `control_mode` attribute of the `MotorChannel` object to `ControlMode.VELOCITY`.

```python filename="python"
from motorgo import ControlMode

left_drive_wheel.control_mode = ControlMode.VELOCITY
```

Next, you must configure the velocity PID controller parameters for the motor:

```python filename="python"
left_drive_wheel.set_velocity_pid_gains(p = 1.0, i = 0.0, d = 0.0, output_ramp = None, lpf = None)
```

The `output_ramp` can be set to limit the rate of change of the motor power output. The `lpf` parameter can be set to apply a low-pass filter to the velocity setpoint. You can set any of the parameters to `None` to use the default values.

Finally, set the desired velocity of the motor by setting the `velocity` attribute of the `MotorChannel` object to the desired velocity in rad/s.

```python filename="python"
left_drive_wheel.velocity = 1.0
```

If the motor seems to be spinning at the top speed regardless of the velocity setpoint, you may need to flip the sign of the PID gains, as the motor may be spinning in the opposite direction of the encoder.

## Reading IMU Data

The data from the onboard IMU can be accessed through the `IMU` object in the `Plink` object.

```python
imu = plink.imu
```

The `IMU` object provides the raw IMU data from the gyroscope, accelerometer, and magnetometer.
```
gyro = imu.gyro
accel = imu.accel
mag = imu.mag
```

The data is returned as `numpy.ndarray` objects, each containing 3 elements corresponding to the x, y, and z axes. The coordinate frame is on the MotorGo board. The gyroscope data is in rad/s, the accelerometer data is in m/s^2, and the magnetometer data is in uT.

In addition to the raw data, the MotorGo Python API filters the IMU data to compute the orientation of the MotorGo relative to the Earth's gravity vector. You can access the orientation of the Plink board as a quaternion:


```python
orientation = imu.orientation # Returns a numpy.ndarray containing [w, x, y, z]
```

You can also read the gravity vector, relative to the MotorGo board, as a numpy.ndarray containing [x, y, z]:

```python
gravity_vector = imu.gravity_vector
```

## Reading Light and Range Sensor Data

Since the QWIIC ports on the MotorGo are connected to the Raspberry Pi I2C bus, you can access the sensors directly from the Raspberry Pi.

Once you have connected the sensor, you can confirm that the Raspberry Pi can detect the sensor by running the following command:

```bash
i2cdetect -y 1
```

You should see the address of the sensor in the output. For example, the BH1750 light sensor has an address of `0x23`.

You can read the light sensor data in Python with the following code:

```python
import board
import adafruit_bh1750

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bh1750.BH1750(i2c)

while True:
    print("%.2f Lux" % sensor.lux)
    time.sleep(0.2)
```

You can read the range sensor data in Python with the following code:

```python
import board
import adafruit_vl53l4cx

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

vl53 = adafruit_vl53l4cx.VL53L4CX(i2c)

print("VL53L4CX Simple Test.")

vl53.start_ranging()

while True:
    while not vl53.data_ready:
        pass
    vl53.clear_interrupt()
    print("Distance: {} cm".format(vl53.distance))
```

## Examples

You can find full examples of using the MotorGo Plink Python API in the [`examples`](examples/) directory of this repository.
