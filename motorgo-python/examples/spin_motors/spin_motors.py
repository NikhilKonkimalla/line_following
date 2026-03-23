# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

import time

from motorgo import BrakeMode, ControlMode, Plink


def main():
    # Create a Plink object, the main interface to the MotorGo board
    plink = Plink()

    # The Plink object has 4 MotorChannel objects, corresponding to the 4 motor channels
    # on the board
    # You can access them directly: plink.channel1
    # Or you can save references as local variables for convenience:
    left_motor = plink.channel1
    right_motor = plink.channel2

    # Finally, connect to the MotorGo board and push the configuration
    plink.connect()

    # You can configure how you want to control the motor channels.
    # Power mode: Set the power of the motor in the range [-1, 1]
    #            This directly corresponds to setting a voltage to the motor
    #
    # Velocity mode: Set the velocity of the motor in rad/s
    #              This mode requires setting the velocity PID gains
    #              It also requires an encoder to be connected to the motor
    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.VELOCITY
    plink.channel3.control_mode = ControlMode.POWER
    plink.channel4.control_mode = ControlMode.POWER

    # If you are using ControlMode.VELOCITY, you must set the velocity PID gains
    right_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

    while True:

        # Set motor powers
        # Set the power command in the range [-1, 1]
        left_motor.power_command = 0.5

        # Set the velocity command in rad/s
        right_motor.velocity_command = 1.2

        # Set the power command in the range [-1, 1]
        plink.channel3.power_command = 0.25
        plink.channel4.power_command = -0.25

        # You can read the position and velocity of the motor channels from the encoders
        print("----")
        print(
            f"Channel 1 position: {plink.channel1.position}, velocity: {plink.channel1.velocity}"
        )
        print(
            f"Channel 2 position: {plink.channel2.position}, velocity: {plink.channel2.velocity}"
        )
        print(
            f"Channel 3 position: {plink.channel3.position}, velocity: {plink.channel3.velocity}"
        )
        print(
            f"Channel 4 position: {plink.channel4.position}, velocity: {plink.channel4.velocity}"
        )
        print("----")

        time.sleep(0.01)


if __name__ == "__main__":
    main()
