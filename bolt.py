#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import random

class Robot:
    def __init__(self):
        self.ev3 = EV3Brick()

        # Initialize motors and sensors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.D)
        self.touch_sensor = TouchSensor(Port.S2)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S3)
        self.color_sensor_left = ColorSensor(Port.S1)
        self.color_sensor_right = ColorSensor(Port.S4)

        # Drivebase object
        self.robot_drivebase = DriveBase(self.left_motor, self.right_motor, wheel_diameter = 56, axle_track = 100)

        # constants for edge-following
        self.THRESHOLD = (self.color_sensor_left.reflection() + self.color_sensor_right.reflection()) / 2
        self.TURN_SPEED = 67.5
        self.STRAIGHT_SPEED = -150

        self.IS_DRIVING = False

    def start(self):
        """Starts the edge-following and obstacle detection program when touch sensor is pressed"""
        while True:
            if self.touch_sensor.pressed():
                while self.touch_sensor.pressed():
                    wait(10)

                self.IS_DRIVING = not self.IS_DRIVING 

                if self.IS_DRIVING:
                    self.ev3.screen.clear()
                    self.ev3.speaker.play_file('driving_begun.wav')
                    self.ev3.screen.print("Driving begun...")

                    wait(300)
                else:
                    self.ev3.screen.clear()
                    self.ev3.speaker.play_file('driving_stopped.wav')
                    self.ev3.screen.print("Driving stopped...")
                    self.stop()

            if self.IS_DRIVING:
                self.drive()

            wait(50)
            
    def drive(self):
        """Main drive method for edge-following."""
        while self.IS_DRIVING:
            LEFT_REFLECT, RIGHT_REFLECT = self.check_sensors()

            # Edge following logic based on sensor readings
            if LEFT_REFLECT < self.THRESHOLD:  # Left sensor detects edge, turn right
                self.robot_drivebase.drive(self.STRAIGHT_SPEED, self.TURN_SPEED)
            elif RIGHT_REFLECT < self.THRESHOLD:  # Right sensor detects edge, turn left
                self.robot_drivebase.drive(self.STRAIGHT_SPEED, -self.TURN_SPEED)
            else:  # No edge detected, move straight
                self.robot_drivebase.drive(self.STRAIGHT_SPEED, 0)

            
            # Obstacle following logic based on ultrasonic sensor reading
            while self.ultrasonic_sensor.distance() < 300:

                # random reversing and turning angle for when obstacle detected
                self.RANDOM_BACKWARDS_TIME = random.randint(300, 900)
                self.RANDOM_TURN_ANGLE = random.randint(-90, 90)

                
                self.robot_drivebase.stop()
                self.ev3.speaker.play_file("obstacle_detected.wav")
                self.robot_drivebase.drive(100, 0)
                wait(self.RANDOM_BACKWARDS_TIME)
                self.robot_drivebase.stop()
                
                print("turning angle:", self.RANDOM_TURN_ANGLE)
                self.robot_drivebase.turn(self.RANDOM_TURN_ANGLE)
                
                break
            
            
            if self.touch_sensor.pressed():
                self.ev3.speaker.play_file('driving_stopped.wav')
                robot.stop()
                break

            wait(10)  # Small delay for stability

    def check_sensors(self):
        """Sensor checking for the drive method"""
        LEFT_INTENSITY = self.color_sensor_left.reflection()
        RIGHT_INTENSITY = self.color_sensor_right.reflection()
        return LEFT_INTENSITY, RIGHT_INTENSITY

    def stop(self):
        """Stops the robot"""
        self.robot_drivebase.stop()

robot = Robot()
wait(500)
robot.ev3.speaker.play_file('touch_to_start.wav')
robot.start()
