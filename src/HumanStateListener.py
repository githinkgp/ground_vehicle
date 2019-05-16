# Humans state Listener Class
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from target_tracker.msg import TargetEstimateArray, TargetEstimate


# begin class def
class HumanStateListener:
    def __init__(self):
       
        # init data
        self.tStamp = 0.0
        self.State = []
        self.Cov = []
        # subscriber to the Human state estimator publishes by target_tracker
        print 'initializing Human state subscriber...'
        self.subHumanStateEstimate = rospy.Subscriber('/target_tracker/human_state_estimate',TargetEstimateArray,self.callback)
        
    # end of init

    # callback to Human state estimator
    def callback(self,msg):
        self.State = []
        self.Cov = []
        #targets=msg.targets

        for i in xrange(len(msg.targets)):
            self.State.append(msg.targets[i].state)
            self.Cov.append(msg.targets[i].cov)
    
    # end callback

# end class def


