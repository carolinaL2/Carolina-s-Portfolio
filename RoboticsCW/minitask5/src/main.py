#!/usr/bin/env python3

"""
Main entry point for the robot node.
"""

import rospy
from robot import Robot

if __name__ == '__main__':
    try:
        robot = Robot() 
        robot.run()
    except rospy.ROSInterruptException:
        pass 