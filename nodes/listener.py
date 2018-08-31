#!/usr/bin/env python
"""LISTENER"""
# -*- coding: ascii -*-
import math
import rospy
import roslib
import tf
import geometry_msgs.msg
import turtlesim.srv
roslib.load_manifest('kpit_learning_tf')


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    LISTENER = tf.TransformListener()

    rospy.wait_for_service('spawn')
    SPAWNER = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    SPAWNER(4, 2, 0, 'turtle2')

    TURTLE_VEL = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    RATE = rospy.Rate(10.0)
    rospy.sleep(1)
    (TRANS, ROT) = LISTENER.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
    (T) = LISTENER.lookupTwist('/turtle2', '/turtle1', rospy.Time(0), rospy.Duration(.01))

    DISTANCE = math.sqrt(TRANS[1]*TRANS[1]+TRANS[0]*TRANS[0])
    print(DISTANCE)
    while not rospy.is_shutdown():
        try:
            (TRANS, ROT) = LISTENER.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            (T) = LISTENER.lookupTwist('/turtle2', '/turtle1', rospy.Time(0), rospy.Duration(.01))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        EULER = tf.transformations.euler_from_quaternion(ROT)
        CMD = geometry_msgs.msg.Twist()

        # print(DISTANCE)
        ANGLE = math.atan2(abs(TRANS[1]), abs(TRANS[0]))
        DISTANCE1 = math.sqrt(TRANS[1]*TRANS[1]+TRANS[0]*TRANS[0])
        print(ANGLE)
        # if t[1][2]!=0:
        #     rospy.sleep(1)
        FLAG = 0
        LINEAR = 4

        if DISTANCE > DISTANCE1 and TRANS[0] > 0:
            LINEAR = -LINEAR

        if DISTANCE < DISTANCE1 and TRANS[0] < 0:
            LINEAR = -LINEAR


        if abs(DISTANCE1 - DISTANCE) < .35:
            print("mndkfn")
            LINEAR = 0

        elif DISTANCE < DISTANCE1 and T[1][2] == 0:
            LINEAR = LINEAR

            FLAG = 1
            print("Asfdgfh")
        elif DISTANCE > DISTANCE1 and T[1][2] == 0:
            LINEAR = LINEAR

            print("jdhfgsf")

        elif abs(EULER[2]) > 0.05:
            LINEAR = 0
            print("iuytfd")

            # print(CMD.LINEAR.x)
        CMD.angular.z = 5*EULER[2] + .1
        CMD.linear.x = LINEAR

        TURTLE_VEL.publish(CMD)

        # rospy.sleep()

        RATE.sleep()
