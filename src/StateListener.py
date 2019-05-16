# Gazebo bot state Listener Class
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates

# begin class def
class StateListener:
    def __init__(self,topic):
       
        # init data
        self.topic = topic
        self.tStamp = 0.0
        self.Pose = [0,0,0]
        self.Twist = [0,0]

        self.ObsPose = [0,0,0]
        self.ObsTwist = [0,0]
        self.detectDObs = 0

        # subscriber to the RPLIDAR
        #print 'initializing RPLIDAR subscribers...'
        self.subRplidar = rospy.Subscriber(topic,ModelStates,self.callback)
        
    # end of init

    # callback to LaserScan
    def callback(self,msg):
        #self.Pose = [0,0,0]
        #self.Twist = [0,0,0]
        names=msg.name
        pose=msg.pose
        twist=msg.twist
        self.detectDObs = 0
        idx=0
        for n in names:
            if n=="mybot":
                #self.State[0].append(pose[idx])
                #self.State[1].append(twist[idx])
                (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[idx].orientation.x, msg.pose[idx].orientation.y, msg.pose[idx].orientation.z, msg.pose[idx].orientation.w])
                self.Pose[0] = pose[idx].position.x
                self.Pose[1] = pose[idx].position.y
                if y>0:
                    y=y-np.pi
                else:
                    y=y+np.pi
                self.Pose[2] = y
                self.Twist[0] = np.sqrt((twist[idx].linear.x)**2 + (twist[idx].linear.y)**2)
                self.Twist[1] = twist[idx].angular.z
                #print "yaw:",y
            elif n=="turtlebot":
                self.detectDObs = 1
                (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[idx].orientation.x, msg.pose[idx].orientation.y, msg.pose[idx].orientation.z, msg.pose[idx].orientation.w])
                self.ObsPose[0] = pose[idx].position.x
                self.ObsPose[1] = pose[idx].position.y
                if y>0:
                    y=y-np.pi
                else:
                    y=y+np.pi
                self.ObsPose[2] = y
                self.ObsTwist[0] = np.sqrt((twist[idx].linear.x)**2 + (twist[idx].linear.y)**2)
                self.ObsTwist[1] = twist[idx].angular.z
            idx+=1
    
    # end callback

# end class def

