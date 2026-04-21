#!/usr/bin/env python

import rospy
import tf
from math import atan2, sqrt, pow

def get_yaw_from_quaternion(quaternion):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    return yaw

def tf_listener():
    rospy.init_node('tf_listener', anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(5.0)

    while not rospy.is_shutdown():
        try:
            listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(1.0))
            
            (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))

            x = trans[0]
            y = trans[1]

            yaw = get_yaw_from_quaternion(rot)

            rospy.loginfo("x: %.5f, y: %.5f, yaw: %.5f" % (x, y, yaw))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: %s" %(e))

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_listener()
    except rospy.ROSInterruptException:
        pass
