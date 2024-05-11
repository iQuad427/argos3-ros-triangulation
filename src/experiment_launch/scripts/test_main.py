#!/usr/bin/python3

import rospy
from experiment_utils.msg import Positions


if __name__ == '__main__':
    # Read parameter test with default value
    rospy.init_node("eee")

    param = rospy.get_param("test_bool", False)

    print(rospy.Time.now())
    print(type(rospy.Time.now()))

    msg = Positions()
    msg.header.stamp = rospy.Time.now()

    print(msg)

    print(param)
