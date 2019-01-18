#!/usr/bin/env python

import rospy


def main():
	okay = 1
	while okay == 1:
		try:
			while True:
				print("1")
				rospy.sleep(0.1)
		except KeyboardInterrupt:
			print("2")


	






	rospy.sleep(1)




if __name__ == '__main__':
	main()
	