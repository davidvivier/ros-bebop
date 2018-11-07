#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


BTN_EMERGENCY = 2
BTN_TAKEOFF = 5
BTN_LAND = 4
BTN_MODE = 0
BTN_FRONTFLIP = 3

AXIS_FORWARD_BACKWARD = 1
AXIS_RIGHT_LEFT = 0

AXIS_UP_DOWN = 2
AXIS_ORIENTATION = 3

resetPub = rospy.Publisher('bebop/reset', Empty, queue_size=10)
takeoffPub = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)
landPub = rospy.Publisher('bebop/land', Empty, queue_size=10)
flipPub = rospy.Publisher('bebop/flip', UInt8, queue_size=10)

pilotPub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)


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
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', str(joy))

    #order is important : priority if several buttons pressed at once
    if joy.buttons[BTN_EMERGENCY]:
        resetPub.publish()
        rospy.loginfo(rospy.get_caller_id() + 'emergency')
    elif joy.buttons[BTN_LAND]:
        landPub.publish()
        rospy.loginfo(rospy.get_caller_id() + 'land')
    elif joy.buttons[BTN_TAKEOFF]:
        takeoffPub.publish()
        rospy.loginfo(rospy.get_caller_id() + 'takeoff')
    elif joy.axes[AXIS_FORWARD_BACKWARD] != 0:
        linear = Vector3(joy.axes[AXIS_FORWARD_BACKWARD], 0, 0)
        angular = Vector3(0, 0, 0)
        twist = Twist(linear, angular)
        pilotPub.publish(twist)
        rospy.loginfo(rospy.get_caller_id() + 'pilot : %s', str(twist))
    elif joy.axes[AXIS_RIGHT_LEFT] != 0:
        linear = Vector3(0, joy.axes[AXIS_RIGHT_LEFT], 0)
        angular = Vector3(0, 0, 0)
        twist = Twist(linear, angular)
        rospy.loginfo(rospy.get_caller_id() + 'pilot : %s', str(twist))
        pilotPub.publish(twist)
    elif joy.axes[AXIS_UP_DOWN] != 0:
        linear = Vector3(0, 0, joy.axes[AXIS_UP_DOWN])
        angular = Vector3(0, 0, 0)
        twist = Twist(linear, angular)
        rospy.loginfo(rospy.get_caller_id() + 'pilot : %s', str(twist))
        pilotPub.publish(twist)
    elif joy.axes[AXIS_ORIENTATION] != 0:
        linear = Vector3(0, 0, 0)
        angular = Vector3(0, 0, joy.axes[AXIS_ORIENTATION])
        twist = Twist(linear, angular)
        rospy.loginfo(rospy.get_caller_id() + 'pilot : %s', str(twist))
        pilotPub.publish(twist)
    elif joy.buttons[BTN_FRONTFLIP] != 0:
        flipPub.publish(0)
        rospy.loginfo(rospy.get_caller_id() + 'backflip')



    
        



if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
