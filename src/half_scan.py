#!/usr/bin/env python
# RPLidar Listener Class
import rospy
import numpy as np
import time
from math import *
from sensor_msgs.msg import LaserScan

# begin class def
class ScanHalf:

    def __init__(self):

        # subscriber to the RPLIDAR
        print 'initializing RPLIDAR subscribers...'
        rospy.init_node('half_scan', anonymous=True)

        self.subRplidar = rospy.Subscriber('/scan',LaserScan,self.callback)
        self.scan_pub = rospy.Publisher('/half_scan', LaserScan, queue_size=10)

        
    # end of init

    # callback to LaserScan
    def callback(self,msg):
        rate = rospy.Rate(10)

        scan_msg = LaserScan()
        scan_msg.header = msg.header
        scan_msg.angle_min = msg.angle_min
        scan_msg.angle_max = msg.angle_max
        scan_msg.angle_increment = msg.angle_increment
        scan_msg.time_increment = msg.time_increment
        scan_msg.scan_time = msg.scan_time
        scan_msg.range_min = msg.range_min
        scan_msg.range_max = msg.range_max
        #print np.shape(msg.ranges)
        count = int(msg.scan_time/msg.time_increment)
        scan_msg.ranges = np.zeros((count,),dtype=np.float)
        scan_msg.intensities = np.zeros((count,),dtype=np.float)

        for i in xrange(count):
            #if i < 359 or i > 1079:
            if i < 489 or i > 949:
                scan_msg.ranges[i] = msg.ranges[i]
                scan_msg.intensities[i] = msg.intensities[i]
            else:
                scan_msg.ranges[i] = float('inf')
                scan_msg.intensities[i] = 0.0
        #print "here"
        #rate.sleep()
        self.scan_pub.publish(scan_msg)
    
    # end callback

# end class def


def scanhalf_main():

    sh = ScanHalf()
    while not rospy.is_shutdown():
        start = time.time()
if __name__ == '__main__':
    try:
        scanhalf_main()

    except rospy.ROSInterruptException:
        pass
