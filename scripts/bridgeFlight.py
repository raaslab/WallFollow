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
from wall_follow.msg import Lines


def horLineCB(data):
	print("\n----------horLineCB----------")
	rospy.loginfo(data)

	# ROS_INFO("first line: dist:%2.4f, angle:%d, confidence:%d",lines->dist[0], lines->angle[0], lines->confidence[0]);
	#hor_lines = hor_lines*lines;
	#new_hor_data = 1;









def main():
	
	rospy.init_node('bridgeFlight')

	rospy.Subscriber("/hor/ho/li",Lines,horLineCB)
	#ros::Subscriber hor_lines_sub = nh.subscribe<wall_follow::Lines>("/hor/ho/li",10,hor_lines_cb);
	#ros::Subscriber vert_lines_sub = nh.subscribe<wall_follow::Lines>("/vert/ho/li",10,vert_lines_cb);
	
	GCmode = rospy.Publisher("/bridgeFlight/GCmode", Int64, queue_size=10) # publishes flag to tell either girder = 0 or column =1 flight

	while not rospy.is_shutdown():
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

	


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	

