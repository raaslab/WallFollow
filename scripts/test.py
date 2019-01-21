#!/usr/bin/env python

import rospy
from pynput import keyboard
import threading





# The key combination to check
COMBINATIONS = [
	{keyboard.KeyCode(char='1')}
]

# The currently active modifiers
current = set()

def execute():
	print ("Do Something")

def on_press(key):
	if any([key in COMBO for COMBO in COMBINATIONS]):
		current.add(key)
		if any(all(k in current for k in COMBO) for COMBO in COMBINATIONS):
			execute()
	pass

def on_release(key):
	if any([key in COMBO for COMBO in COMBINATIONS]):
		current.remove(key)
	pass


def main():
	try:
		while True:
			with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
				listener.join()

			timerThread = threading.Thread(target= <your-function> )
			timerThread.daemon = False # Is this a background process?
			timerThread.start()


	except KeyboardInterrupt:
		print('interrupted!')
		




if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	