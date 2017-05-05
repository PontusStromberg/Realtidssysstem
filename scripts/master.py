#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os, sys, inspect
import signal
from json import dumps, load
import time
from geometry_msgs.msg import Twist
from rts_project.msg import GenericLogData # Import the generic log data
import roslaunch

class Master(object):

    def __init__(self):
        # Sets up publishers and subscribers for the reference generator
        self.status_master = rospy.Subscriber('/system/status_master', String, self.handle_system_status)
        self.status_controller = rospy.Publisher('/system/status_controller', String, queue_size = 10)
        
        # A subscriber which listens and logs data from the crazyflie
        self.position_sub = rospy.Subscriber('/crazyflie/log_pos', GenericLogData, self.handle_pos)
        self.cmd_vel_pub = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size = 10)

        # Initial position and attributes where the positin is made accessible
        self.x = 0.0;
        self.y = 0.0;
        self.z = 0.0;
        self.twist = Twist()

    def send_csig(self, thrust, pitch, roll, yawrate):
        self.twist.linear.z = thrust
        self.twist.linear.y = pitch
        self.twist.linear.x = roll
        self.twist.angular.z = yawrate
        self.cmd_vel_pub.publish(self.twist)

    def handle_system_status(self, msg):
        # Example of a callback to the status_master topic
        if msg.data == 'True':
            self.status = True

    def help(self):
        # Example of how methods of the object may be accessed in the main loop
        print ('(@ master) USAGE: .... asd qwe rty')

    def handle_pos(self, data):
        # Callback for receiving positional data from the UAV
        # Currently, this simply sets the attributes x,y,z with
        # the corresponding position (meters). Try to uncomment
        # the print statement to get the positions in the terminal
        # in real-time
        
        # Save position to attributes of the Master object
        self.x,self.y,self.z = data.values
        
        # Optionally, print the positions in the terminal
        print (self.x,self.y,self.z)

def signal_handler(signal, frame):
    print 'Shutting down master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('master')
    master = Master()
    signal.signal(signal.SIGINT, signal_handler)
    
    start_time = time.time()

    while True:
        time.sleep(0.01)
        #command = raw_input('(@ master) Command: ')
        #data = command.split(' ')
        #data = 
        #master.help()
        #master.status_controller.publish(data[0])
        if time.time() - start_time > 10:
            print (time.time() - start_time)
            thrust = 20000
        else:
            thrust = 0.0
        master.send_csig(thrust,0.0,0.0,0.0)

if __name__ == '__main__':
    main()
