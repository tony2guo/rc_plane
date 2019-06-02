#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8MultiArray
import math

cmd_msg = Int8MultiArray()


def rc_plane_cmd_cb(msg):
    global cmd_msg
    cmd_msg = msg


if __name__ == '__main__':
    rospy.init_node("visualizer", anonymous=True)
    joint_Publisher = rospy.Publisher(
        "rc_plane_joint_states", JointState, queue_size=1)
    tm_robot_coordinate_Subscriber = rospy.Subscriber(
        "rc_plane_cmd_throttle", Int8MultiArray, rc_plane_cmd_cb)
    r = rospy.Rate(10)
    pub_msg = JointState()
    tf_prefix = rospy.get_param("~tf_prefix", "")
    joint_list = [
        "front_engine_propeller_joint", "vertical_stabilizer_aileron_joint",
        "horizontal_stabilizer_aileron_joint", "right_wing_aileron_joint",
        "left_wing_aileron_joint"
    ]
    pub_msg.name = [tf_prefix + joint for joint in joint_list]
    rospy.wait_for_message("rc_plane_cmd_throttle", Int8MultiArray)
    propeller_position = 0
    factor = 127.0
    while not rospy.is_shutdown():
        pub_msg.header.stamp = rospy.Time.now()
        throttle = cmd_msg.data[0] / factor + 1
        rudder = cmd_msg.data[1] / factor
        elevator = cmd_msg.data[2] / factor
        aileron_right = cmd_msg.data[3] / factor
        aileron_left = cmd_msg.data[4] / factor
        propeller_position += throttle
        pub_msg.position = [
            propeller_position, rudder, elevator, aileron_right, aileron_left
        ]
        joint_Publisher.publish(pub_msg)
        r.sleep()