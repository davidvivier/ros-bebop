#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

BTN_EMERGENCY = 2
BTN_TAKEOFF = 5
BTN_LAND = 4

resetPub = rospy.Publisher('bebop/reset', Empty, queue_size=10)

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
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', str(joy.buttons[BTN_EMERGENCY]))

    if joy.buttons[BTN_EMERGENCY] == 1:
        resetPub.publish()



if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
