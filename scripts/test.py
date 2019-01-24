#!/usr/bin/env python

from __future__ import absolute_import, division, unicode_literals, print_function
 
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
 
def keypress():
    global char
    char = getch()
 
def main():
    global char
    char = None
    _thread.start_new_thread(keypress, ())
 
    while True:
        if char is not None:
            if char == 'q' or char == '\x1b':  # x1b is ESC
                exit()
            char = None
            try:
                print("Key pressed is " + char.decode('utf-8'))
            except UnicodeDecodeError:
                print("character can not be decoded, sorry!")
                char = None
            _thread.start_new_thread(keypress, ())
            
        print("Program is running\n")
        time.sleep(1)
 
if __name__ == "__main__":
    main()
 