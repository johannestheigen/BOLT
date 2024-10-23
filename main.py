#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor, GyroSensor, Motor, TouchSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

class Robot:
    def init(self):
        self.ev3 = EV3Brick()

Initialize the motors and sensors
        self.left_motor = Motor(Port.B)
        self.right_motor = Motor(Port.C)
        self.left_color_sensor = ColorSensor(Port.S1)
        self.right_color_sensor = ColorSensor(Port.S4)
        self.touch_sensor = TouchSensor(Port.S2)

        # Create a DriveBase object with the motors
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter = 56, axle_track = 90)

Constants for edge-following
        self.THRESHOLD = 30  # Light intensity threshold to detect edge
        self.TURN_SPEED = 100  # Turning speed when adjusting course
        self.STRAIGHT_SPEED = -200  # Speed when moving straight

        self.is_driving = False

    def start(self):
        """Starts the edge-following program when the touch sensor is pressed."""
        self.ev3.screen.print("Waiting for touch sensor press to start...")
        while True:
            if self.touch_sensor.pressed():
                while self.touch_sensor.pressed():
                    wait(10)  # Poll the touch sensor

                self.is_driving = not self.is_driving

                if self.is_driving:
                    self.ev3.screen.print("Driving begun...")
                else:
                    self.ev3.screen.print("Driving stopped...")
                    self.stop()

            if self.is_driving:
                self.drive()

            wait(50)

    def drive(self):
        """Main drive method for edge-following."""
        while self.is_driving:
            left_reflect, right_reflect = self.check_sensors()

            # Edge following logic based on sensor readings
            if left_reflect < self.THRESHOLD:  # Left sensor detects edge, turn right
                self.drive_base.drive(self.STRAIGHT_SPEED, self.TURN_SPEED)
            elif right_reflect < self.THRESHOLD:  # Right sensor detects edge, turn left
                self.drive_base.drive(self.STRAIGHT_SPEED, -self.TURN_SPEED)
            else:  # No edge detected, move straight
                self.drive_base.drive(self.STRAIGHT_SPEED, 0)

            if self.touch_sensor.pressed():
                self.ev3.screen.print("Touch sensor pressed; stopping")
                break

            wait(10)  # Small delay for stability

    def check_sensors(self):
        """Checks and returns the light intensity of both color sensors."""
        left_intensity = self.left_color_sensor.reflection()
        right_intensity = self.right_color_sensor.reflection()
        self.ev3.screen.print("Left Intensity:", left_intensity, "Right Intensity:", right_intensity)
        return left_intensity, right_intensity

    def stop(self):
        """Stops the robot and motors."""
        self.drive_base.stop()

Create a robot object and start the edge-following program
robot = Robot()
robot.start() 