#!/usr/bin/env python
import rospy
import sys
import tty, termios
import argparse
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
THRESHOLD_VAL = 0.15
BASE_LENGTH = 0.2
ANGULAR_SPEED = 15
LINEAR_SPEED = -3

def threshold(val):
    if abs(val) < THRESHOLD_VAL:
        return 0
    else:
        if val > 0:
            return val - THRESHOLD_VAL
        else :
            return val + THRESHOLD_VAL

def joy_callback(data):
    global Vr, Vl
    #print 'data',data.axes[0], data.axes[1]
    w = ANGULAR_SPEED*threshold(data.axes[0])
    v = LINEAR_SPEED*threshold(data.axes[1])
    Vr = Twist()
    Vl = Twist()
    Vr.linear.x = (2*v + w*BASE_LENGTH )/2
    Vl.linear.x = (2*v - w*BASE_LENGTH )/2
    publishers[0].publish(Vr)
    publishers[1].publish(Vl)


def start():
    global Vr, Vl, publishers
    Vr = 0
    Vl = 0
    parser = argparse.ArgumentParser()
    parser.add_argument('args', nargs='+')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    print args
    rospy.init_node('joy_stick_control', anonymous=True)
    subJoystick = rospy.Subscriber('/joy', Joy, joy_callback)
    rate = rospy.Rate(10) # 10hz
    publishers = []
    for arg in args.args:
        topic_name = "cmd_vel/"+arg
        pub = rospy.Publisher(topic_name, Twist , queue_size=10)
        publishers.append(pub)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if Vr!=0 or Vl!=0:
            publishers[0].publish(Vr)
            publishers[1].publish(Vl)
        rate.sleep()

if __name__ == '__main__':
    start()
