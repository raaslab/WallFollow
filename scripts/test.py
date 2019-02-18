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
try:
    from msvcrt import getch  # try to import Windows version
except ImportError:
    def getch():   # define non-Windows version
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
 

def horLaserCB(data):
	global horTopic
	horTopic = data.ranges

def vertLaserCB(data):
	global vertTopic
	vertTopic = data.ranges

def keypress():
    global char
    char = getch()
 
def main():
	rospy.init_node('bridgeFlight')
	rospy.Subscriber("/laser/scan",LaserScan,horLaserCB)
	rospy.Subscriber("/laser/scan_vert",LaserScan,vertLaserCB)

	time.sleep(5)

	counterOfBuffer = 5
	listOfListHor = [[] for x in xrange(counterOfBuffer)]
	listOfListVert = [[] for x in xrange(counterOfBuffer)]
	counter = 0 # index of preCLH and preCLV
	switches = 0

	while True:
		cleanedListHor = [x for x in horTopic if x != np.inf]
		cleanedListVert = [x for x in vertTopic if x != np.inf]
		listOfListHor[counter] = cleanedListHor
		listOfListVert[counter] = cleanedListVert
		if counter == counterOfBuffer-1:
			preCLH = listOfListHor[0]
			preCLV = listOfListVert[0]
		else:
			preCLH = listOfListHor[counter+1]
			preCLV = listOfListVert[counter+1]

		NCLH = len(cleanedListHor)
		NCLV = len(cleanedListVert)
		NPCLH = len(preCLH)
		NPCLV = len(preCLV)
		print("NCLH:" + str(NCLH))

		if counter == counterOfBuffer - 1:
			counter = 0
		counter = counter + 1
		time.sleep(0.2)

 
if __name__ == "__main__":
    main()
 