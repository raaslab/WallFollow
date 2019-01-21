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
 


def horLineCB(data):
	# print("\n----------horLineCB----------")
	# rospy.loginfo(data)
	data

def vertLineCB(data):
	data

def rightBesideWallPubCB(data):
	global rightBesideTopic
	rightBesideTopic = data

def leftBesideWallPubCB(data):
	global leftBesideTopic
	leftBesideTopic = data	

def upColumnLoopPubCB(data):
	global upColumnTopic
	upColumnTopic = data

def downColumnLoopPubCB(data):
	global downColumnTopic
	downColumnTopic = data

def horLaserCB(data):
	global horTopic
	horTopic = data.ranges

def vertLaserCB(data):
	global vertTopic
	vertTopic = data.ranges

def keypress():
	global char
	print("KP")

	char = getch()



def main():
	rospy.init_node('bridgeFlight')
	rospy.Subscriber("/hor/ho/li",Lines,horLineCB)
	rospy.Subscriber("/vert/ho/li",Lines,vertLineCB)
	rospy.Subscriber("/right/besideWallPub",PositionTarget,rightBesideWallPubCB) # besidewall output going right
	rospy.Subscriber("/left/besideWallPub",PositionTarget,leftBesideWallPubCB) # besidewall output going left
	rospy.Subscriber("/up/columnLoopPub",PositionTarget,upColumnLoopPubCB) # columnloop output going up
	rospy.Subscriber("/down/columnLoopPub",PositionTarget,downColumnLoopPubCB) # columnloop output going down
	rospy.Subscriber("/laser/scan",LaserScan,horLaserCB)
	rospy.Subscriber("/laser/scan_vert",LaserScan,vertLaserCB)
	outputData = rospy.Publisher("/mavros/setpoint_raw/local",PositionTarget,queue_size=10) # mavros topic
	GCmode = rospy.Publisher("/bridgeFlight/GCmode", Int64, queue_size=10) # publishes flag to tell either girderRight = 0, girderLeft = 1, columnUp = 2, columnDown =3
	rate = rospy.Rate(10) # 10hz

	okayMode = 0
	listBufferTime = 0
	timeOfBuffer = 5
	global char



	while not rospy.is_shutdown():
		print("Switch between modes.")
		print("Which mode would you like to start with?\n(girderRight = 0, girderLeft = 1, columnUp = 2, columnDown = 3, manualMode = 4)")
		gcmode = int(input()) # get the start input mode from user
		GCmode.publish(gcmode) # publishing starting mode
		if gcmode == 0:	# starting girderRight flight
			okayMode = 1
			print("girderRight flight choosen.")
		elif gcmode == 1: # starting girderLeft flight
			okayMode = 1
			print("girderLeft flight choosen.")
		elif gcmode == 2: # starting columnUp flight
			okayMode = 1
			print("columnUp flight choosen.")
		elif gcmode == 3: # starting columnDown flight
			okayMode = 1
			print("columnDown flight choosen.")
		elif gcmode == 4: # starting manual mode
			okayMode = 2
		else:
			print("Not a valid choice. Re-choose.")

		cleanedListHor = [x for x in horTopic if x != np.inf]
		cleanedListVert = [x for x in vertTopic if x != np.inf]
		preCLH = cleanedListHor	# previousCleanedListHor
		preCLV = cleanedListVert # previousCleanedListVert

		if cleanedListHor > cleanedListVert: # printing information
			print("Girder flight better.")
		else:
			print("Column flight better.")

		rospy.sleep(timeOfBuffer) # long pause

		while okayMode == 1: # autonomous mode
			cleanedListHor = [x for x in horTopic if x != np.inf]
			cleanedListVert = [x for x in vertTopic if x != np.inf]

			# need to add buffer for below variables
#				if cleanedListHor > preCLH && cleanedListVert < preCLV: # if hor is getting bigger and vert is getting smaller
#				gcmode = 0
#				print("Switching to Right.")
#			elif cleanedListHor < preCLH && cleanedListVert > preCLV: # if hor is getting smaller and vert is getting bigger
#				gcmode = 3
#				print("Switching to Down.")
#			else: # no change in type



			
			if gcmode == 0:	# starting girderRight flight
				outputData.publish(rightBesideTopic)
				print("Right.")
			elif gcmode == 1: # starting girderLeft flight
				outputData.publish(leftBesideTopic)
				print("Left.")
			elif gcmode == 2: # starting columnUp flight
				outputData.publish(upColumnTopic)
				print("Up.")
			else: # starting columnDown flight. gcmode == 3
				outputData.publish(downColumnTopic)
				print("Down.")

				# we want to check then number of laser data we are getting from each laser
				# compare the two and see which one has more
					# more along the lines of check which one has drastically increased from previous time steps
				# if it has more then switch modes
				# figure out how to manually switch modes
			
			preCLH = cleanedListHor
			preCLV = cleanedListVert
			GCmode.publish(gcmode)
			rospy.sleep(1)

		while okayMode == 2: # manual mode
			print("start")
			char = None
			_thread.start_new_thread(keypress, ())

			while True:
				if char is not None: # gets keypress
					try:
						print(char)
						print("Key pressed is " + char.decode('utf-8'))
						# gcmode = char
					except UnicodeDecodeError:
						print("character can not be decoded, sorry!")
						char = None
					_thread.start_new_thread(keypress, ())
					if char == '\x1b':  # x1b is ESC
						exit()
					char = None

				if gcmode == 0:	# starting girderRight flight
					topic = rightBesideTopic
					outputData.publish(rightBesideTopic)
					print("girderRight flight choosen.")
					GCmode.publish(gcmode)

				elif gcmode == 1: # starting girderLeft flight
					topic = leftBesideTopic
					outputData.publish(leftBesideTopic)
					print("girderLeft flight choosen.")
					GCmode.publish(gcmode)

				elif gcmode == 2: # starting columnUp flight
					topic = upColumnTopic
					outputData.publish(upColumnTopic)
					print("columnUp flight choosen.")
					GCmode.publish(gcmode)

				elif gcmode == 3: # starting columnDown flight
					topic = downColumnTopic
					outputData.publish(downColumnTopic)
					print("columnDown flight choosen.")
					GCmode.publish(gcmode)

				else:
					print("Which mode would you like?\n(girderRight = 0, girderLeft = 1, columnUp = 2, columnDown = 3)")
					gcmode = int(input()) # get the start input mode from user
					GCmode.publish(gcmode)
					print("start")

				rospy.sleep(1)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	

