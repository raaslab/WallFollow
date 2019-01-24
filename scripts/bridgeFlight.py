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
 


#def horLineCB(data):
#	data

#def vertLineCB(data):
#	data

def rightBesideWallPubCB(data):
	# print("\n----------rightBesideWallPubCB----------")
	# rospy.loginfo(data)
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
	char = getch()


def main():
	rospy.init_node('bridgeFlight')
	#rospy.Subscriber("/hor/ho/li",Lines,horLineCB)
	#rospy.Subscriber("/vert/ho/li",Lines,vertLineCB)
	rospy.Subscriber("/right/besideWallPub",PositionTarget,rightBesideWallPubCB) # besidewall output going right
	rospy.Subscriber("/left/besideWallPub",PositionTarget,leftBesideWallPubCB) # besidewall output going left
	rospy.Subscriber("/up/columnLoopPub",PositionTarget,upColumnLoopPubCB) # columnloop output going up
	rospy.Subscriber("/down/columnLoopPub",PositionTarget,downColumnLoopPubCB) # columnloop output going down
	rospy.Subscriber("/laser/scan",LaserScan,horLaserCB)
	rospy.Subscriber("/laser/scan_vert",LaserScan,vertLaserCB)
	outputData = rospy.Publisher("/mavros/setpoint_raw/local",PositionTarget,queue_size=10) # mavros topic
	GCmode = rospy.Publisher("/bridgeFlight/GCmode", Int64, queue_size=10) # publishes flag to tell either girderRight = 0, girderLeft = 1, columnUp = 2, columnDown =3
	rate = rospy.Rate(10) # 10hz

	global rightBesideTopic
	global leftBesideTopic
	global upColumnTopic
	global downColumnTopic
	global horTopic
	global vertTopic
	global char

	okayMode = 0
	listBufferTime = 0
	counterOfBuffer = 5 # time buffer between checking if the LIDAR is getting more data compared to previous LIDAR scan

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

		while okayMode == 1: # autonomous mode
			char = None
			_thread.start_new_thread(keypress, ())
			cleanedListHor = [x for x in horTopic if x != np.inf]
			cleanedListVert = [x for x in vertTopic if x != np.inf]
			preCLH = cleanedListHor	# previousCleanedListHor
			preCLV = cleanedListVert # previousCleanedListVert
			listOfListHor = [[] for x in xrange(counterOfBuffer)]
			listOfListVert = [[] for x in xrange(counterOfBuffer)]

			rospy.sleep(counterOfBuffer) # long pause
			counter = 0 # index of preCLH and preCLV
			switches = 0
			while True:
				if char == '\x1b':  # x1b is ESC
					exit()
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

				# need to add buffer for below variables
				if NCLH > NPCLH: # if hor is getting bigger and vert is getting smaller
					if NCLV < NPCLV:
						gcmode = 0 # go right
						print("Switching to Right.")
					elif NCLV > NPCLV:
						gcmode = ???
						print("")
					else:
						pring("")
				elif NCLH < NPCLH and NCLV > NPCLV: # if hor is getting smaller and vert is getting bigger
					if NCLV > NPCLV:
						gcmode = 3 # go down
						print("Switching to Down.")
					elif NCLV < NPCLV:
						gcmode = ???
						print("")
					else:
						print("")
				else: # no change in type
					print("NCLH == NPCLH")

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

				GCmode.publish(gcmode)
				if counter == counterOfBuffer - 1:
					counter = 0
				counter = counter + 1
				rospy.sleep(0.1)

		while okayMode == 2: # manual mode
			char = None
			print("Which mode would you like?\n(girderRight = 0, girderLeft = 1, columnUp = 2, columnDown = 3)")
			gcmode = int(input()) # get the start input mode from user
			GCmode.publish(gcmode)
			_thread.start_new_thread(keypress, ())

			while True:
				if char is not None: # gets keypress
					try:
						print("Key pressed is " + char.decode('utf-8'))
						gcmode = int(char)
					except UnicodeDecodeError:
						print("character can not be decoded, sorry!\n")
						char = None
					_thread.start_new_thread(keypress, ())
					if char == '\x1b':  # x1b is ESC
						exit()
					char = None

				if gcmode == 0:	# starting girderRight flight
					topic = rightBesideTopic
					outputData.publish(rightBesideTopic)
					GCmode.publish(gcmode)
					print("girderRight flight choosen.\n")

				elif gcmode == 1: # starting girderLeft flight
					topic = leftBesideTopic
					outputData.publish(leftBesideTopic)
					GCmode.publish(gcmode)
					print("girderLeft flight choosen.\n")

				elif gcmode == 2: # starting columnUp flight
					topic = upColumnTopic
					outputData.publish(upColumnTopic)
					GCmode.publish(gcmode)
					print("columnUp flight choosen.\n")

				elif gcmode == 3: # starting columnDown flight
					topic = downColumnTopic
					outputData.publish(downColumnTopic)
					GCmode.publish(gcmode)
					print("columnDown flight choosen.\n")

				else:
					print("ERROR!!!\nNo mode selected.\nPrevious mode kept.\n")
					outputData.publish(topic)
					GCmode.publish(gcmode)
				rospy.sleep(0.1)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	

