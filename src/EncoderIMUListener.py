# EncoderIMU Listener Class
import rospy
import tf
import numpy as np
from math import sqrt
from sensor_msgs.msg import LaserScan
from vehicle_localization.msg import wheel_coordinate


# begin class def
class EncoderIMUListener:
    def __init__(self):
       
        # init data
        self.tStamp = 0.0
        self.State = [0.0,0.0,0.0]
        self.Twist = [0.0,0.0]
        self.Cov = []
        # subscriber to the EncoderIMU state published by vehicle_localization
        print 'initializing EncoderIMU state subscriber...'
        subEncoderIMU = rospy.Subscriber('/Localization/wheel_coordinates',wheel_coordinate,self.callback)
        
    # end of init

    # callback to EncoderIMU state estimator
    def callback(self,msg):
        self.State = [msg.xcoordinate_center_filtered, msg.ycoordinate_center_filtered,msg.heading]
        v= sqrt(msg.Vx_center_filtered**2 + msg.Vy_center_filtered**2)
        self.Twist = [v,0.0]
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.xcoordinate_center_filtered, msg.ycoordinate_center_filtered, 0),
                        tf.transformations.quaternion_from_euler(0, 0, msg.heading),
                        rospy.Time.now(),
                        "chassis",
                        "map")
    
    # end callback

# end class def



