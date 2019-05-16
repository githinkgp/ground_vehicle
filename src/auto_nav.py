#!/usr/bin/python

# basic python imports
import numpy as np
from math import *
import time

# ros imports
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from scipy.spatial import ConvexHull
import matplotlib.patches as patches
# from sensor_msgs.msg import CameraInfo

# the lidar listener class
from LidarListener import *
from StateListener import *
import sys
sys.path.insert(0,'code')
sys.path.insert(0,'NMPC')
from dompc import *
from dist import dist


class clusters:
    def __init__(self,rangeData):
        count = len(rangeData)
        #clusterXY stores cordinates of the points in the cluster
        self.clusterXY = np.zeros((count,2),dtype=np.float)
        self.clusterIdx=[]
        clusterStart = 1
        self.clusterCount = 0
        if count:
            for i in xrange(count-1):
                dist=rangeData[i][1]
                theta=rangeData[i][0]

                self.clusterXY[i,0]=dist*cos(radians(180-fabs(theta)))
                self.clusterXY[i,1]=(dist*sin(radians(theta)))
                
                if clusterStart:
                    self.clusterIdx.append([i])
                    clusterStart = 0
                d1=rangeData[i][1]
                d2=rangeData[i+1][1]
                t1=rangeData[i][0]
                t2=rangeData[i+1][0]
                dist=sqrt((d1*sin(radians(t1))-d2*sin(radians(t2)))**2 + (d1*cos(radians(t1))-d2*cos(radians(t2)))**2)


                #print self.clusterIdx
                if dist >0.5:
                    self.clusterIdx[self.clusterCount].append(i)
                    #print rangeData[self.clusterIdx[self.clusterCount][0]][0] , rangeData[i+1][0]
                    #if fabs(rangeData[self.clusterIdx[self.clusterCount][0]][0]) <150 and fabs(rangeData[self.clusterIdx[self.clusterCount][1]][0]) <150:
                        #print "remove", self.clusterCount
                    #    self.clusterIdx.pop(self.clusterCount)
                    #    clusterStart = 1
                    #else:
                        #print "here"
                    clusterStart = 1
                    self.clusterCount+=1
                    
                elif i==count-2:
                    #print self.clusterIdx[self.clusterCount][0], i+1
                    #print rangeData[self.clusterIdx[self.clusterCount][0]][0] , rangeData[i+1][0]
                    self.clusterIdx[self.clusterCount].append(i+1)

                    #if fabs(rangeData[self.clusterIdx[self.clusterCount][0]][0]) <150 and fabs(rangeData[self.clusterIdx[self.clusterCount][1]][0]) <150:
                        #print "remove", self.clusterCount
                    #    self.clusterIdx.pop(self.clusterCount)
                    #    clusterStart = 1
                    #else:
                    clusterStart = 1
                    self.clusterCount+=1
                    dist=rangeData[i+1][1]
                    theta=rangeData[i+1][0]
                    self.clusterXY[i+1,0]=dist*cos(radians(180-fabs(theta)))
                    self.clusterXY[i+1,1]=(dist*sin(radians(theta)))
            #print self.clusterIdx
            #print "clusterXY",self.clusterXY
class EnclosingEllipse:
    def __init__(self,cluster):
        clusterXY = cluster.clusterXY
        clusterIdx = cluster.clusterIdx
        #length = len(clusterIdx)
        self.centroid = np.zeros((len(clusterIdx),2),dtype=np.float)
        self.a = np.zeros((len(clusterIdx),),dtype=np.float)
        self.b = np.zeros((len(clusterIdx),),dtype=np.float)
        self.m = np.zeros((len(clusterIdx),),dtype=np.float)
        """
        for i in xrange(len(clusterIdx)):
            icluster = clusterXY[clusterIdx[i][0]:clusterIdx[i][1]+1]
            if len(icluster)>3:
                hull = ConvexHull(icluster)
                #plotting each cluster
                plt.scatter(icluster[:,0],icluster[:,1])
                CvxHull =icluster[hull.vertices,:]
                self.centroid[i][:] = [np.average(CvxHull[:,0]), np.average(CvxHull[:,1])]
                plt.plot(icluster[hull.vertices,0], icluster[hull.vertices,1], 'r--', lw=2)
                cent_temp=np.ones((len(CvxHull),2),dtype=np.float)
                cent_temp[:,0] =self.centroid[i][0]*cent_temp[:,0]
                cent_temp[:,1] =self.centroid[i][1]*cent_temp[:,1]
                diff=CvxHull-cent_temp
                distances=np.sqrt(np.square(diff[:,0])+np.square(diff[:,1]))

                self.m[i],c = np.polyfit(icluster[:,0],icluster[:,1],1)

                alpha = np.arctan(np.divide(diff[:,1],diff[:,0]))
                delta = atan(self.m[i])*np.ones((len(alpha),1),dtype=np.float)
                distX = np.fabs(np.multiply(distances,np.cos(alpha-delta)))
                distY = np.fabs(np.multiply(distances,np.sin(alpha-delta)))

                self.a[i]=np.max(distX)
                self.b[i]=np.max(distY)
        """

class Waypoint:
    def __init__(self):
        self.waypoint = [0,0,0]
        self.subWaypoint = rospy.Subscriber('/mybot/set_goal',Vector3,self.callback)

    def callback(self,msg):
        self.waypoint = [msg.x, msg.y, msg.z]

def mpc_main():
    # initialize
    RosNodeName = 'mpc_test'
    #print 'initializing {}...'.format(RosNodeName)
    rospy.init_node(RosNodeName,anonymous=True)
    
    lidar_listener = LidarListener('/laser/scan')
    state_listener = StateListener('/gazebo/model_states')
    MPC=doMPC()
    
    rate_init = rospy.Rate(10.0)
    rate_init.sleep()

    #fig=plt.figure()
    #ax=plt.axes()

    fig2=plt.figure()
    ax2=plt.axes()

    WP = Waypoint()
    velocity_publisher = rospy.Publisher('/mybot/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        waypoint = WP.waypoint
        start=time.time()
        PoseData = state_listener.Pose
        TwistData = state_listener.Twist
        rangeData = lidar_listener.RangeData        
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        cluster = clusters(rangeData)

        R = [[cos(PoseData[2]), -sin(PoseData[2])],[sin(PoseData[2]),cos(PoseData[2])]]

        
        X=np.ones((1,len(cluster.clusterXY)),dtype=np.float) * PoseData[0]
        Y=np.ones((1,len(cluster.clusterXY)),dtype=np.float) * PoseData[1]
        XY = np.concatenate((X,Y),axis=0)
        xo = np.matmul(R,np.transpose(cluster.clusterXY)) + XY
        
        #ax.scatter(cluster.clusterXY[:,0], cluster.clusterXY[:,1])
        ax2.scatter(xo[0,:], xo[1,:])
        elliHull = EnclosingEllipse(cluster)
        #print dist(PoseData[0:2], waypoint)
        if dist(PoseData[0:2], waypoint)>0.2:
            U = MPC.getOptControl(waypoint,elliHull,PoseData,TwistData)
            vel_msg.linear.x = -U[0]
            vel_msg.angular.z = -U[1]
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        end = time.time()
        #print end-start
        #print elliHull.centroid, elliHull.a, elliHull.b, degrees(atan(elliHull.m))
        for i in xrange(len(cluster.clusterIdx)):
            elli=patches.Ellipse(xy=elliHull.centroid[i],width=2*elliHull.a[i], height=2*elliHull.b[i], angle=degrees(atan(elliHull.m[i])), fc='None')
            #ax.add_artist(elli)
        #plt.plot([0,-c/m],[c,0])
        #plt.plot(elliHull.centroid[0],elliHull.centroid[1],'ro')
        #ax.grid()
        #ax.axis([0,5,-5,5])
        ax2.grid()
        ax2.axis([-10,10,-10,10])
        plt.pause(0.01)
    
        plt.draw()
        #input("Press Enter to exit do-mpc...")
        #ax.cla()
        #ax2.cla()
    #plt.cla()
        
        
        
#############################################################################################################################
# running
if __name__ == '__main__':
    try:
        mpc_main()
    except rospy.ROSInterruptException:
        pass
