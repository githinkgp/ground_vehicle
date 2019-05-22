# RPLidar Listener Class
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

# begin class def
class LidarListener:
    def __init__(self,topic):
       
        # init data
        self.topic = topic
        self.tStamp = 0.0
        self.RangeData = []
        self.InfData = []

        # subscriber to the RPLIDAR
        print 'initializing RPLIDAR subscribers...'
        self.subRplidar = rospy.Subscriber(topic,LaserScan,self.callback)
        
    # end of init

    # callback to LaserScan
    def callback(self,msg):
        
        self.RangeData = []
        self.tStamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**(-9)
        #count = int(msg.scan_time/msg.time_increment)

        count = len(msg.ranges)
        for i in xrange(count):
            degree = np.rad2deg(msg.angle_min + msg.angle_increment * i)
            # lidar angle tf
            if degree >= 0:
                degree -= 180.0
            else:
                degree += 180.0
            distance = msg.ranges[i]
            #print distance
            intensity = msg.intensities[i]
            # if there is valid beam reflection
            #if 1.0 < intensity < np.inf:
            if distance <= 3:
                self.RangeData.append((-degree,distance))   # mirror reflection of the angle degree
    
    # end callback

# end class def

