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
    euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return Vector3(x=euler[0], y=euler[1], z=euler[2])  # RPY

if __name__ == '__main__':
    rospy.init_node('graph_plot')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    ax=plt.subplot(projection="polar")
    r = [1, 1, 1, 1]
    theta = [0, 0, 0, 0]

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans_A = tfBuffer.lookup_transform('world', 'TF_A', rospy.Time())
            trans_B = tfBuffer.lookup_transform('world', 'TF_B', rospy.Time())
            trans_C = tfBuffer.lookup_transform('world', 'TF_C', rospy.Time())
            trans_ave = tfBuffer.lookup_transform('world', 'TF_average', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        ax.cla()
        theta = [quat2euler(trans_A.transform.rotation).y, 
                 quat2euler(trans_B.transform.rotation).y, 
                 quat2euler(trans_C.transform.rotation).y, 
                 quat2euler(trans_ave.transform.rotation).y]
        
        ax.scatter(theta[0], r[0], color='b')
        ax.scatter(theta[1], r[1], color='g')
        ax.scatter(theta[2], r[2], color='c')
        ax.scatter(theta[3], r[3], color='r')
        ax.set_rlim((0, 1.5))
        plt.pause(.01)

        # rospy.loginfo(quat2euler(trans_A.transform.rotation))

        rate.sleep()