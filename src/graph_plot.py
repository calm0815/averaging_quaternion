#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Vector3
import numpy as np
import matplotlib.pyplot as plt

def quat2euler(quaternion):
    euler = tf.transformations.quat2euler((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=euler[0], y=euler[1], z=euler[2])  # RPY

if __name__ == '__main__':
    rospy.init_node('graph_plot')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    flip=1
    x = np.linspace(0, 14, 100)
    for i in range(1, 7):
        plt.plot(x, np.sin(x + i * .5) * (7 - i) * flip)

    plt.show()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans_A = tfBuffer.lookup_transform('world', 'TF_A', rospy.Time())
            trans_B = tfBuffer.lookup_transform('world', 'TF_B', rospy.Time())
            trans_C = tfBuffer.lookup_transform('world', 'TF_C', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rospy.loginfo(trans_A)

        rate.sleep()