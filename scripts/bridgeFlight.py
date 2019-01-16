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
	# print("\n----------vertLineCB----------")
	data

def besideWallPubCB(data):
	global besideTopic
	besideTopic = data

def columnLoopPubCB(data):
	global columnTopic
	columnTopic = data

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
	rospy.Subscriber("/besideWallPub",PositionTarget,besideWallPubCB) # besidewall output
	rospy.Subscriber("/columnLoopPub",PositionTarget,columnLoopPubCB) # columnloop output
	rospy.Subscriber("/laser/scan",LaserScan,horLaserCB)
	rospy.Subscriber("/laser/scan_vert",LaserScan,vertLaserCB)
	outputData = rospy.Publisher("/mavros/setpoint_raw/local",PositionTarget,queue_size=10) # mavros topic
	GCmode = rospy.Publisher("/bridgeFlight/GCmode", Int64, queue_size=10) # publishes flag to tell either girder = 0 or column =1 flight

	okayMode = 0

	while not rospy.is_shutdown():
		print("Switch between modes.")
		print("Which mode would you like to start with? (girder == 0, column == 1)")
		gcmode = int(input()) # get the start input mode from user
		GCmode.publish(gcmode) # publishing starting mode
		if gcmode == 0:	# starting girder flight
			okayMode = 1
			print("girder flight choosen.")
		elif gcmode == 1: # starting column flight
			okayMode = 1
			print("column flight choosen.")
		else:
			print("Not a valid choice. Re-choose.")


		while okayMode == 1:
			if gcmode == 0:	# starting girder flight
				cleanedList = [x for x in horTopic if x != np.inf]
				outputData.publish(besideTopic)
				print(len(cleanedList))
				print(len(horTopic))
				print("Works!")
			else: # starting column flight. gcmode == 1
				cleanedList = [x for x in vertTopic if x != np.inf]
				outputData.publish(columnTopic)
				print(len(cleanedList))
				print(len(vertTopic))
				print("Works!")

			# start which ever node is chosen by "gcmode"
			# get hor and vert laser data
				# we want to check then number of laser data we are getting from each laser
				# compare the two and see which one has more
					# more along the lines of check which one has drastically increased from previous time steps
				# if it has more then switch modes
			rospy.sleep(1)

	


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	

