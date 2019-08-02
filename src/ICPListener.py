# ICP Listener Class
import rospy
import tf
import numpy as np
from math import sqrt
from geometry_msgs.msg import Pose2D

# begin class def
class ICPListener:
    def __init__(self):
       
        # init data

        self.State = [0.0,0.0,0.0]
        self.Cov = []
        # subscriber to the ICP state published by laser_scan_matcher
        print 'initializing ICP state subscriber...'
        subICP = rospy.Subscriber('/pose2D',Pose2D,self.callback)
        
    # end of init

    # callback to EncoderIMU state estimator
    def callback(self,msg):
        #changing the sign of the state to conform with NED
        self.State = [msg.x, msg.y,-msg.theta]
    # end callback

# end class def




