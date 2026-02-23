from motorgo import Plink, ControlMode
import adafruit_vl53l4cx
import board
import time

i2c = board.I2C()

sensor = adafruit_vl53l4cx.VL53L4CX(i2c)

while True:
    sensor.start_ranging()
    print(sensor.distance)
    time.sleep(0.05)
