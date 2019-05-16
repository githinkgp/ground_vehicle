# LidarPointsLabelledListener Class
import rospy
import numpy as np
from math import *
#from sensor_msgs.msg import LaserScan
from camera_lidar_fusion.msg import LidarPointCloud

# begin class def
class LidarPointsLabelledListener:

    def __init__(self):
       
        # init data
        self.tStamp = 0.0
        self.PersonData = []
        self.StaticData = []

        # subscriber to LidarPointsLabelled that contains all the scanned data points
        # and the label inidicating if the point corresponds to a person(-1) or a static obstacle (1)
        print 'initializing LidarPointsLabelled subscriber...'
        self.subLidarPoints = rospy.Subscriber('/lidar_points_labelled/lidar_points',LidarPointCloud,self.callback)
        
    # end of init

    # callback to LidarPointsLabelled data
    def callback(self,msg):
        
        self.PersonData = []
        self.StaticData = []
        self.tStamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**(-9)
        count = len(msg.range_data)
        for i in xrange(count):
            distance = msg.range_data[i].distance
            degree = msg.range_data[i].angle
            label = msg.range_data[i].label

            if distance <= 5 and fabs(degree)<=90: #data points within 5m
                if label == -1:
                    self.PersonData.append((degree,distance)) #points corresponding to a person
                elif label == 1:
                    self.StaticData.append((degree,distance)) #points corresponding to a static obstacle

    # end callback

# end class def


