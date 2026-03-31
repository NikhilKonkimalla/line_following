#!/usr/bin/env python3
"""Simple motor test — spins all 4 Plink channels for 2 seconds then stops."""
import time
from motorgo import Plink, ControlMode

plink = Plink()
plink.power_supply_voltage = 9.6

motors = [plink.channel1, plink.channel2, plink.channel3, plink.channel4]

for m in motors:
    m.motor_voltage_limit = 6.0

plink.connect()

for m in motors:
    m.control_mode = ControlMode.POWER

print("Running all 4 motors at power=1.0 for 2 seconds...")
for m in motors:
    m.power_command = 1.0

time.sleep(2.0)

for m in motors:
    m.power_command = 0.0
print("Done.")
