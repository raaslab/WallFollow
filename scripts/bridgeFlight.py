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

	okayMode = 0
	listBuffer = 0
	timeOfBuffer = 5


	while not rospy.is_shutdown():
		print("Switch between modes.")
		print("Which mode would you like to start with?\n(girderRight = 0, girderLeft = 1, columnUp = 2, columnDown =3)")
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
		else:
			print("Not a valid choice. Re-choose.")

		cleanedListHor = [x for x in horTopic if x != np.inf]
		cleanedListVert = [x for x in vertTopic if x != np.inf]
		preCLH = cleanedListHor
		preCLV = cleanedListVert

		rospy.sleep(1)

		while okayMode == 1:
			cleanedListHor = [x for x in horTopic if x != np.inf]
			cleanedListVert = [x for x in vertTopic if x != np.inf]

			if cleanedListHor > cleanedListVert: # printing information
				print("Girder flight better.")
			else:
				print("Column flight better.")

			# need to add buffer for below variables
			# need to add the assigning of preCLH and preCLV
			if cleanedListHor > preCLH && cleanedListVert < preCLV: # if hor is getting bigger and vert is getting smaller

			elif cleanedListHor < preCLH && cleanedListVert > preCLV: # if hor is getting smaller and vert is getting bigger

			else: # no change in type



			
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

	


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	

