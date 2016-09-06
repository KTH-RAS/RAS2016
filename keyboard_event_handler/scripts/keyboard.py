#!/usr/bin/env python
import rospy
import sys
import tty, termios
import argparse
import os
from geometry_msgs.msg import Twist

def move(akey):
    if len(publishers) == 2:

        speedr = Twist()
        speedl = Twist()

        # forward
        if akey == "w":
            speedr.linear.x = 1.0
            speedl.linear.x = 1.0

        # back
        elif akey == "s":
            speedr.linear.x = -1.0
            speedl.linear.x = -1.0

        # left
        elif akey == "a":
            speedr.linear.x = 1.0
            speedl.linear.x = -1.0
        # right
        elif akey == "d":
            speedr.linear.x = -1.0
            speedl.linear.x =  1.0


        publishers[0].publish(speedr)
        publishers[1].publish(speedl)






def getch():
    """getch() -> key character

        Read a single keypress from stdin and return the resulting character.
        Nothing is echoed to the console. This call will block if a keypress
        is not already available, but will not wait for Enter to be pressed.

        If the pressed key was a modifier key, nothing will be detected; if
        it were a special function key, it may return the first character of
        of an escape sequence, leaving additional characters in the buffer.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if ord(ch)==3: #Capturre Ctrl+C
	rospy.signal_shutdown("quit")
	#sys.exit(0)
    return ch

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument('args', nargs='+')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    print args


    rospy.init_node('keyboard_event_handler', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    publishers = []

    for arg in args.args:
        topic_name = "cmd_vel/"+arg
        pub = rospy.Publisher(topic_name, Twist , queue_size=10)
        publishers.append(pub)


    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        key = getch()
        move(key)
        rate.sleep()
