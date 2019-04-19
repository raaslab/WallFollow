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

def modeCB(data):
	global myMode
	myMode = data.data

def keypress():
	global char
	char = getch()


def main():
	rospy.init_node('bridgeFlight')
	#rospy.Subscriber("/hor/ho/li",Lines,horLineCB)
	#rospy.Subscriber("/vert/ho/li",Lines,vertLineCB)
	rospy.Subscriber("/right/besideWall/vel",PositionTarget,rightBesideWallPubCB) # besidewall output going right
	rospy.Subscriber("/left/besideWall/vel",PositionTarget,leftBesideWallPubCB) # besidewall output going left
	rospy.Subscriber("/up/columnLoop/vel",PositionTarget,upColumnLoopPubCB) # columnloop output going up
	rospy.Subscriber("/down/columnLoop/vel",PositionTarget,downColumnLoopPubCB) # columnloop output going down
	rospy.Subscriber("/laser/scan",LaserScan,horLaserCB)
	rospy.Subscriber("/laser/scan_vert",LaserScan,vertLaserCB)
	rospy.Subscriber("/manualSwitcher/flyMode",Int64,modeCB)
	outputData = rospy.Publisher("/mavros/setpoint_raw/local",PositionTarget,queue_size=10) # mavros topic
	GCmode = rospy.Publisher("/bridgeFlight/GCmode", Int64, queue_size=10) # publishes flag to tell either girderRight = 0, girderLeft = 1, columnUp = 2, columnDown =3
	# rate = rospy.Rate(10) # 10hz

	# VARIABLES
	global rightBesideTopic
	global leftBesideTopic
	global upColumnTopic
	global downColumnTopic
	global horTopic
	global vertTopic
	global char
	global myMode

	okayMode = 0
	listBufferTime = 0
	counterOfBuffer = 20 # buffer between checking if the LIDAR is getting more data compared to previous LIDAR scan
	listOfModes = [3,0,1,2,3,1,2,3,1,2,3,1,2,3,1] # [3,0,...] is added at the beginning to get the UAV to the starting location
	sleepTime = 0.1 # amount of time we wait at the end of while loops (used in rospy.sleep(sleepTime))

	while not rospy.is_shutdown():
		print("Switch between modes.")
		print("Which mode would you like to start with?\n(manualMode = 4, assistedMode = 5, assistedTimed = 7)") # obsolete girderRight = 0, girderLeft = 1, columnUp = 2, columnDown = 3
		gcmode = int(input()) # get the start input mode from user
		GCmode.publish(gcmode) # publishing starting mode
		if gcmode == 4: # starting manual mode
			okayMode = 2
		elif gcmode == 5: # starting assisted mode
			okayMode = 3
		elif gcmode == 6:
			okayMode = 4
		elif gcmode == 7:
			okayMode = 5
		else:
			print("Not a valid choice. Re-choose.")

		while okayMode == 1: # autonomous mode
			char = None
			# _thread.start_new_thread(keypress, ())
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
				#if NCLH > NPCLH: # if hor is getting bigger and vert is getting smaller
				#	if NCLV < NPCLV:
				#		gcmode = 0 # go right
				#		print("Switching to Right.")
				#	elif NCLV > NPCLV:
				#		gcmode = ???
				#		print("")
				#	else: # No change in CLV but change in CLH
				#		print("")
				#elif NCLH < NPCLH and NCLV > NPCLV: # if hor is getting smaller and vert is getting bigger
				#	if NCLV > NPCLV:
				#		gcmode = 3 # go down
				#		print("Switching to Down.")
				#	elif NCLV < NPCLV:
				#		gcmode = ???
				#		print("")
				#	else: # No change in CLV but change in CLH
				#		print("")
				#else: # no change CLH but potential change in CLV
				#	print("NCLH == NPCLH")

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
			myMode = gcmode
			GCmode.publish(gcmode)
			#_thread.start_new_thread(keypress, ())

			while True:
				if char is not None: # gets keypress
					try:
						print("Key pressed is " + char.decode('utf-8'))
						gcmode = int(char)
					except UnicodeDecodeError:
						print("character can not be decoded, sorry!\n")
						char = None
					#_thread.start_new_thread(keypress, ())
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

				print(str(myMode))
				gcmode = int(myMode)
				rospy.sleep(0.05)

		while okayMode == 3: # assisted mode
			# this mode should be exactly the same as autonomous mode, but should follow a predefined set of modes instead of picking on the fly
			char = None
			#_thread.start_new_thread(keypress, ())
			cleanedListHor = [x for x in horTopic if x != np.inf] # list of all datapoints that are not inf coming from the lidar
			cleanedListVert = [x for x in vertTopic if x != np.inf]
			preCLH = cleanedListHor	# previousCleanedListHor
			preCLV = cleanedListVert # previousCleanedListVert
			listOfListHor = [[] for x in xrange(counterOfBuffer)] # list of cleaned hor (list of lidar data after cleaned)
			listOfListVert = [[] for x in xrange(counterOfBuffer)] # list of cleaned vert
			NLH = [0 for x in xrange(counterOfBuffer)] # list of length of hor (filled with the length of the cleaned list)
			NLV = [0 for x in xrange(counterOfBuffer)] # list of length of vert
			checkerH = [[] for x in xrange(counterOfBuffer)] # list of length of hor (filled with -1,0,1 for if current is less,same,more than previous)
			checkerV = [[] for x in xrange(counterOfBuffer)] # list of length of vert


			rospy.sleep(5) # long pause
			counter = 0 # index of preCLH and preCLV
			switches = 0 # how many mode switches we have been through
			counterOfModes = 0 # counter for what mode comes next
			lidarBuffer = 40 # +- range we give to number of lidar lasers per scan difference
			confidenceNumber = 7 # number of previous lidar scans that are less,same,more than current
			timeSwitchLock = 10 # time in seconds that we should wait to relook for a mode switch
			lock = 0 # lock for mode switching
			while True:
				if char == '\x1b':  # x1b is ESC
					exit()
				cleanedListHor = [x for x in horTopic if x != np.inf]
				cleanedListVert = [x for x in vertTopic if x != np.inf]
				listOfListHor[counter] = cleanedListHor
				listOfListVert[counter] = cleanedListVert
				NLH[counter] = len(cleanedListHor)
				NLV[counter] = len(cleanedListVert)
				if counter == counterOfBuffer-1:
					NPCLH = NLH[0]
					NPCLV = NLV[0]
				else:
					NPCLH = NLH[counter+1]
					NPCLV = NLV[counter+1]

				NCLH = NLH[counter] # current number of horizontal lidar lines
				NCLV = NLV[counter] # current number of vertical lidar lines
				#NPCLH = len(preCLH)
				#NPCLV = len(preCLV)

				# need to add buffer for below variables
				# print("NLH:" + str(NLH))
				# print("NCLV:" + str(NCLV))
				if NCLH != 0 and NCLV != 0 and NPCLH != 0 and NPCLV != 0:
					checkerCounter = 0
					for i in range(0,counterOfBuffer-1):
						if NCLH > NLH[checkerCounter]+lidarBuffer:		# if current is larger than previous checkerH is 1 in list
							checkerH[checkerCounter] = 1
						elif NCLH < NLH[checkerCounter]-lidarBuffer:	# if current is smaller than previous checkerH is -1 in list
							checkerH[checkerCounter] = -1
						else:											# if current is within +- lidarBuffer range of previous
							checkerH[checkerCounter] = 0
						if NCLV > NLV[checkerCounter]+lidarBuffer:		# if current is larger than previous checkerV is 1 in list
							checkerV[checkerCounter] = 1
						elif NCLV < NLV[checkerCounter]-lidarBuffer: 	# if current is smaller than previous checkerV is -1 in list
							checkerV[checkerCounter] = -1
						else:											# if current is within +- lidarBuffer range of previous
							checkerV[checkerCounter] = 0

						checkerCounter = checkerCounter + 1

					CH1 = checkerH.count(1) # if current is larger than previous number of laser scans
					CH0 = checkerH.count(0) # if current is smaller than previous number of laser scans
					CHn1 = checkerH.count(-1) # if current is similar to previous number of laser scans
					CV1 = checkerV.count(1) # if current is larger than previous number of laser scans
					CV0 = checkerV.count(0) # if current is smaller than previous number of laser scans
					CVn1 = checkerV.count(-1) # if current is similar to previous number of laser scans

					if lock <= 0:
						if CH1 > confidenceNumber:
							# going from column to girder
							gcmode = listOfModes[counterOfModes]
							counterOfModes = counterOfModes + 1
							print("change1")
							lock = timeSwitchLock
						elif CV1 > confidenceNumber:
							# going from girder to column
							gcmode = listOfModes[counterOfModes]
							counterOfModes = counterOfModes + 1
							print("change2")
							lock = timeSwitchLock
						else:
							gcmode = listOfModes[counterOfModes-1]

					if lock > 0:
						lock = lock-1


					print("CH1:" + str(CH1))
					print("CH0:" + str(CH0))
					print("CHn1:" + str(CHn1))
					print("counterOfModes:" + str(counterOfModes))


				if gcmode == 0:	# starting girderRight flight
					outputData.publish(rightBesideTopic)
					#print("Right.")
				elif gcmode == 1: # starting girderLeft flight
					outputData.publish(leftBesideTopic)
					#print("Left.")
				elif gcmode == 2: # starting columnUp flight
					outputData.publish(upColumnTopic)
					#print("Up.")
				else: # starting columnDown flight. gcmode == 3
					outputData.publish(downColumnTopic)
					#print("Down.")

				GCmode.publish(gcmode)
				if counter == counterOfBuffer - 1:
					counter = 0
				counter = counter + 1
				if counterOfModes == len(listOfModes)-1:
					print("DONE!!!")
					_thread.exit()
					exit()
				rospy.sleep(0.1)

		while okayMode == 4: # test mode for just pass through of topic data, currently testing girderRight
			char = None
			#print("Which mode would you like?\n(girderRight = 0, girderLeft = 1, columnUp = 2, columnDown = 3)")
			#gcmode = int(input()) # get the start input mode from user
			#myMode = gcmode
			#GCmode.publish(gcmode)
			#_thread.start_new_thread(keypress, ())

			while True: # TODO: check to make sure that the topics come through cleanly if there are two topics being published for left and right
				#topic = rightBesideTopic
				outputData.publish(downColumnTopic)
				#GCmode.publish(gcmode)
				#print("Special Mode\r\n")

				rospy.sleep(0.05)

		while okayMode == 5: # assisted mode with timer for down and up
			# this mode should be exactly the same as assisted mode, but should follow a predefined set of modes with timing for down then up
			cleanedListHor = [x for x in horTopic if x != np.inf] 		# list of all datapoints that are not inf coming from the lidar
			cleanedListVert = [x for x in vertTopic if x != np.inf]
			preCLH = cleanedListHor										# previousCleanedListHor
			preCLV = cleanedListVert 									# previousCleanedListVert
			listOfListHor = [[] for x in xrange(counterOfBuffer)] 		# list of cleaned hor (list of lidar data after cleaned)
			listOfListVert = [[] for x in xrange(counterOfBuffer)] 		# list of cleaned vert
			NLH = [0 for x in xrange(counterOfBuffer)] 					# list of length of hor (filled with the length of the cleaned list)
			NLV = [0 for x in xrange(counterOfBuffer)] 					# list of length of vert
			checkerH = [[] for x in xrange(counterOfBuffer)] 			# list of length of hor (filled with -1,0,1 for if current is less,same,more than previous)
			checkerV = [[] for x in xrange(counterOfBuffer)] 			# list of length of vert
			gradientH = [[] for x in xrange(counterOfBuffer)]
			gradientV = [[] for x in xrange(counterOfBuffer)]


			rospy.sleep(5) 			# long pause
			counter = 0 			# index of preCLH and preCLV
			switches = 0 			# how many mode switches we have been through
			counterOfModes = 0 		# counter for what mode comes next
			lidarBuffer = 13 		# +- range we give to number of lidar lasers per scan difference
			confidenceNumber = 0.8	# confidence of changing in %
			timeSwitchLock = 1500 	# buffer that we should wait to relook for a mode switch (real world time = timeSwitchLock * sleepTime)
			lock = 0 				# lock for mode switching
			while True:
				cleanedListHor = [x for x in horTopic if x != np.inf]
				cleanedListVert = [x for x in vertTopic if x != np.inf]
				listOfListHor[counter] = cleanedListHor
				listOfListVert[counter] = cleanedListVert
				NLH[counter] = len(cleanedListHor)
				NLV[counter] = len(cleanedListVert)
				if counter == counterOfBuffer-1: # getting pervious number of lidar lines
					NPCLH = NLH[0]
					NPCLV = NLV[0]
				else:
					NPCLH = NLH[counter+1]
					NPCLV = NLV[counter+1]

				NCLH = NLH[counter] # current number of horizontal lidar lines
				NCLV = NLV[counter] # current number of vertical lidar lines
				
				if NCLH != 0 and NCLV != 0 and NPCLH != 0 and NPCLV != 0:
					# comparing laser scans
					checkerCounter = 0
					for i in range(0,counterOfBuffer-1):
						if NPCLH > NLH[checkerCounter]+lidarBuffer:		# if current is larger than previous checkerH is 1 in list
							checkerH[checkerCounter] = 1
						elif NPCLH < NLH[checkerCounter]-lidarBuffer:	# if current is smaller than previous checkerH is -1 in list
							checkerH[checkerCounter] = -1
						else:											# if current is within +- lidarBuffer range of previous
							checkerH[checkerCounter] = 0

						if NPCLV > NLV[checkerCounter]+lidarBuffer:		# if current is larger than previous checkerV is 1 in list
							checkerV[checkerCounter] = 1
						elif NPCLV < NLV[checkerCounter]-lidarBuffer: 	# if current is smaller than previous checkerV is -1 in list
							checkerV[checkerCounter] = -1
						else:											# if current is within +- lidarBuffer range of previous
							checkerV[checkerCounter] = 0

						checkerCounter = checkerCounter + 1

					# tallying laser scan comparisons
					CH1 = checkerH.count(1) 	# if current is larger than previous number of laser scans
					gradientH[counter] = CH1 	# list of CH1 (list of number of current greater than past lidar scans)
					CH0 = checkerH.count(0) 	# if current is smaller than previous number of laser scans
					CHn1 = checkerH.count(-1) 	# if current is similar to previous number of laser scans
					CV1 = checkerV.count(1) 	# if current is larger than previous number of laser scans
					gradientV[counter] = CV1 	# list of CV1 (list of number of current greater than past lidar scans)
					CV0 = checkerV.count(0) 	# if current is smaller than previous number of laser scans
					CVn1 = checkerV.count(-1) 	# if current is similar to previous number of laser scans
					percentLargerH1 = float(CHn1) / float((CH1+CH0+CHn1))
					percentLargerV1 = float(CVn1) / float((CV1+CV0+CVn1))
					
					# mode switching
					if lock <= 0 and counterOfModes-1 != len(listOfModes):
						if listOfModes[counterOfModes] == 2 and listOfModes[counterOfModes+1] == 3: # TODO: check to make sure that this section works for timed switching for down up
							counterOfModes = counterOfModes + 1
							gcmode = listOfModes[counterOfModes]
							lock = timeSwitchLock * sleepTime - timeSwitchLock*sleepTime*0.5
							print("change0")
						else:	# regular checks for switching between modes
							if percentLargerH1 > confidenceNumber:
								# going from column to girder
								counterOfModes = counterOfModes + 1
								gcmode = listOfModes[counterOfModes]
								lock = timeSwitchLock * sleepTime
								print("change1")
							elif percentLargerV1 > confidenceNumber:
								# going from girder to column
								counterOfModes = counterOfModes + 1
								gcmode = listOfModes[counterOfModes]
								lock = timeSwitchLock * sleepTime
								print("change2")
							else:
								gcmode = listOfModes[counterOfModes]

					# buffer for not allowing switching of modes
					if lock > 0:
						lock = lock-1

					print("_________________")
					print("CH1:" + str(CH1))
					print("CH0:" + str(CH0))
					print("CHn1:" + str(CHn1))
					print("percentLargerH1:" + str(percentLargerH1))
					print("MODE:" + str(listOfModes[counterOfModes]))
					print("counterOfModes:" + str(counterOfModes))

				# publishing correct flight topic
				if gcmode == 0:	# starting girderRight flight
					outputData.publish(rightBesideTopic)
				elif gcmode == 1: # starting girderLeft flight
					outputData.publish(leftBesideTopic)
				elif gcmode == 2: # starting columnUp flight
					outputData.publish(upColumnTopic)
				else: # starting columnDown flight. gcmode == 3
					outputData.publish(downColumnTopic)
				GCmode.publish(gcmode)
				
				# reseting counter for NPCLH and NCLH (vert as well)
				if counter == counterOfBuffer - 1:
					counter = 0
				else:
					counter = counter + 1

				# checking if we've gone through all modes in listOfModes
				if counterOfModes == len(listOfModes)-1:
					print("DONE!!!")
					exit()
				rospy.sleep(sleepTime)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
