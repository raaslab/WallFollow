#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
import numpy as np
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int64
from sensor_msgs.msg import *
from roslaunch.parent import ROSLaunchParent
from wall_follow.msg import Lines
from pynput import keyboard
import tty, termios
import sys
import thread as _thread
import time


def main():
	rospy.init_node('manualSwitcher')
	flyMode = rospy.Publisher("/manualSwitcher/flyMode", Int64, queue_size=10) # publishes flag to tell either girderRight = 0, girderLeft = 1, columnUp = 2, columnDown =3

	while not rospy.is_shutdown():
		print("Which mode would you like?\n(girderRight = 0, girderLeft = 1, columnUp = 2, columnDown = 3)")
		myMode = int(input())
		flyMode.publish(myMode)




if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

	

