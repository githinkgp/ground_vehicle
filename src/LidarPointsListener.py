# LidarPointsListener Class
import rospy
import numpy as np
from math import *
#from sensor_msgs.msg import LaserScan
from camera_lidar_fusion.msg import FusedPerson,FusedPersonArray,LidarPointCloud

# begin class def
class LidarPointsListener:

    def __init__(self):
       
        # init data
        self.tStamp = 0.0
        self.RangeData = []

        # subscriber to LidarPoints that contains all the scanned data points
        print 'initializing LidarPoints subscriber...'
        self.subLidarPoints = rospy.Subscriber('/camera_lidar_fusion/lidar_points',LidarPointCloud,self.callback)
        
    # end of init

    # callback to LidarPoints data
    def callback(self,msg):
        
        self.RangeData = []
        self.tStamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**(-9)

        count = len(msg.range_data)
        for i in xrange(count):
            distance = msg.range_data[i].distance
            degree = msg.range_data[i].angle
            label = msg.range_data[i].label
            self.RangeData.append((degree,distance,label))
    
    # end callback

# end class def

