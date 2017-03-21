#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os, sys, inspect
import signal
from json import dumps, load
import time
from geometry_msgs.msg import Twist
#from rts_project.msg import Num
import roslaunch

class Master(object):

    def __init__(self):
        # Sets up publishers and subscribers for the reference generator
        self.status_master = rospy.Subscriber('/system/status_master', String, self.handle_system_status)
        self.status_controller = rospy.Publisher('/system/status_controller', String, queue_size = 10)

    def handle_system_status(self, msg):
        # Enables the system to continue operation
        if msg.data == 'True':
            self.status = True

    def help(self):
        print ('USAGE: ....')

def signal_handler(signal, frame):
    print 'Shutting down master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('master')
    master = Master()
    signal.signal(signal.SIGINT, signal_handler)
    
    while True:
        command = raw_input('(@ master) Command: ')
        data = command.split(' ')
        master.status_controller.publish(data[0])

if __name__ == '__main__':
    main()
