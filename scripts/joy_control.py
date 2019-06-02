#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray


buttons = {
    'A': 0,
    'B': 1,
    'X': 2,
    'Y': 3,
    'LB': 4,
    'RB': 5,
    'BACK': 6,
    'START': 7,
    'LOGITECH': 8,
    'LSTICK': 9,
    'RSTICK': 10
}
axes = {
    'LX': 0,
    'LY': 1,
    'LT': 2,
    'RX': 3,
    'RY': 4,
    'RT': 5,
    'ARROWX': 6,
    'ARROWY': 7
}


def joy_cb(msg):
    throttle = -1
    if msg.axes[axes['LY']] > 0:
        throttle += msg.axes[axes['LY']] * 2
    rudder = msg.axes[axes['LX']]
    elevator = msg.axes[axes['RY']]
    aileron = msg.axes[axes['RX']]

    pub_msg = Int8MultiArray()
    factor = 127
    pub_msg.data.append(throttle * factor)
    pub_msg.data.append(rudder * factor)
    pub_msg.data.append(elevator * factor)
    pub_msg.data.append(aileron * factor)
    pub_msg.data.append(-aileron * factor)

    rc_plane_cmd_Publisher.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node("joy_control", anonymous=True)
    rc_plane_cmd_Publisher = rospy.Publisher(
        "rc_plane_cmd", Int8MultiArray, queue_size=1)
    joy_Subscriber = rospy.Subscriber("joy", Joy, joy_cb)
    rospy.spin()