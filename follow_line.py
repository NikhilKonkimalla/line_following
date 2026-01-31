import numpy as np
import time
import board
import adafruit_bh1750

# Initialize I2C connection
i2c = board.I2C()  # uses board.SCL and board.SDA

# Create sensor object
sensor = adafruit_bh1750.BH1750(i2c)

#Calibration on white
print("BEGIN CALIBRATION WHITE")
time.sleep(3)
whiteCal = [sensor.lux for _ in range (20)]
meanWhite = sum(whiteCal) / len(whiteCal)

#Calibration on black
print("BEGIN CALIBRATION BLACK")
time.sleep(3)
blackCal = [sensor.lux for _ in range(20)]
meanBlack = sum(blackCal) / len(blackCal)

#Caluculate Threshold
edgeThreshold = (meanBlack + meanWhite) / 2
print("Average White reading: " + meanWhite)
print("Average Black reading: " + meanBlack)
print("Resulting edge threshold: " + edgeThreshold)
print("Beginning course traversal in 5 seconds:")
time.sleep(5)

#TUNING INSTRUCTIONS PROBABLY
#----------------------------#
#Change gain to edit how drastic correction is
#Chagne calibration sample size if not calibrated enough
#We could also change the correction stuff to some freaky stuff


gain = 0.1

while True:
    luxSample = sensor.lux
    error = luxSample - edgeThreshold

    correction = gain * error

    #If positive correction then it is too bright --> Turn left
    #If negative correction then it is too dark --> Turn Right
    print(f"Lux: {luxSample:6.1f} | Error: {error:6.1f} | Correction: {correction:6.2f}")

    # Motor speeds:
    # left_motor_speed = base_speed - correction
    # right_motor_speed = base_speed + correction
    
    time.sleep(0.05)