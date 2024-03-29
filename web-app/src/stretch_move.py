#!/usr/bin/env python3

import rclpy
import time
import stretch_body.robot

robot=stretch_body.robot.Robot()
robot.startup()
robot.stow()

def move_lift_up():
    robot.lift.move_to(0.4)
    robot.push_command()

def move_lift_down():
    robot.lift.move_to(0.1)
    robot.push_command()

def stop():
    robot.stow()
    robot.stop()
