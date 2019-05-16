#!/usr/bin/env python
import numpy as np
from math import tan,atan,degrees,radians,sqrt

import rospy
from camera_lidar_fusion.msg import FusedPerson,FusedPersonArray,LidarPointCloud

from LidarPointsListener import *
from HumanStateListener import *

def LidarPointsLabelled_main():
    # initialize
    RosNodeName = 'lidar_points_labelled'
    print "initializing {}...".format(RosNodeName)
    rospy.init_node(RosNodeName,anonymous=True)

    hz = 5.0
    rate = rospy.Rate(hz)

    pubLidarPointsLabelled = rospy.Publisher('/'+RosNodeName+'/lidar_points', LidarPointCloud, queue_size=10)

    lidarPoints_listener = LidarPointsListener()
    humanState_listener = HumanStateListener()

    LidarPoints = LidarPointCloud()

    rate_init = rospy.Rate(10.0)
    rate_init.sleep()

    while not rospy.is_shutdown():
        CurrentTime = rospy.Time.now()
        LidarPoints.header.stamp = CurrentTime
        LidarPoints.range_data = []

        human_state = humanState_listener.State
        rangeData = lidarPoints_listener.RangeData

        human_count = len(human_state)
        count = len(rangeData)

        for i in xrange(count):
            distance=rangeData[i][1]
            degree=rangeData[i][0]

            point = FusedPerson()
            point.distance = distance
            point.angle = degree
            point.label = 1

            X = distance*cos(radians(degree))
            Y = distance*sin(radians(degree))

            for j in xrange(human_count):
                #d = dist([X,Y],[human_state[j][0],human_state[j][1]])
                d = sqrt((X-human_state[j][0])**2 + (Y-human_state[j][1])**2)
                if d <= 0.3:
                    point.label = -1

            LidarPoints.range_data.append(point)

        pubLidarPointsLabelled.publish(LidarPoints)

        rate.sleep()

if __name__== '__main__':
    try:
        LidarPointsLabelled_main()
    except rospy.ROSInterruptException:
        pass


#LidarPointsLabelled_main()

