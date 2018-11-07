#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

def controller():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('control', anonymous=True)

    rospy.Subscriber('/joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(data):
    joy = data
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', str(joy.buttons[0]))

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
