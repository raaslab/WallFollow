#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int64
from sensor_msgs.msg import NavSatFix
from roslaunch.parent import ROSLaunchParent









def main():
	parent = ROSLaunchParent("bridgeFlightCode", [], is_core=True) # creating roscore
	parent.start() # starting roscore
	rospy.init_node('bridgeFlight')
	# rospy.Subscriber()
	GCmode = rospy.Publisher("/bridgeFlight/GCmode", Int64, queue_size=10) # publishes flag to tell either girder = 0 or column =1 flight

	try:
	    while True:
			print("Switch between modes.")
			print("Which mode would you like to start with?")
			gcmode = int(input()) # get the start input mode from user
			GCmode.publish(gcmode) # publishing starting mode

			# start which ever node is chosen by "gcmode"
			# get hor and vert laser data
				# we want to check then number of laser data we are getting from each laser
				# compare the two and see which one has more
					# more along the lines of check which one has drastically increased from previous time steps
				# if it has more then switch modes

			



	except KeyboardInterrupt:
	    print('interrupted!')

	parent.shutdown()

if __name__ == '__main__':
	main()

