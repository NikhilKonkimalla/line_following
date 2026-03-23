# Simple demo of the VL53L4CD distance sensor.
# Will print the sensed range/distance every second.
import board
import adafruit_vl53l4cx

i2c = board.I2C()  # uses board.SCL and board.SDA

vl53 = adafruit_vl53l4cx.VL53L4CX(i2c)

print("VL53L4CX Simple Test.")

vl53.start_ranging()

while True:
    while not vl53.data_ready:
        pass
    vl53.clear_interrupt()
    print("Distance: {} cm".format(vl53.distance))
