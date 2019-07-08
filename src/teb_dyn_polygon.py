#!/usr/bin/python
from LidarPointsListener import *
from LidarPointsLabelledListener import *
from HumanStateListener import *
from EncoderIMUListener import *
from actuator_control import *
from ICPListener import *

import numpy as np
from math import *
import time
import tf

# ros imports
import rospy
import matplotlib.pyplot as plt
import matplotlib.markers as marker

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, Quaternion

from scipy.spatial import ConvexHull
import matplotlib.patches as patches

from ReachableSetGen import *

import sys
sys.path.insert(0,'code')
sys.path.insert(0,'NMPC')
from dist import dist
from dompc import *

doPlot = False
doGoalOnly = False
#doStaticGoal = False

if doPlot:
    fig=plt.figure()
    ax=plt.axes()

class clusters:
    def __init__(self,rangeData):
        count = len(rangeData)
        #clusterXY stores cordinates of the points in the cluster
        self.clusterXY = np.zeros((count,2),dtype=np.float)
        self.clusterIdx=[]
        clusterStart = 1
        self.clusterCount = 0
        #prev_residual = 1

        #FOV is 60 degree to (-60) degrees, i.e. 120 degs
        scan_angle_limit = 60
        if count:
            for i in xrange(count-1):
                dist=rangeData[i][1]
                theta=rangeData[i][0]

                self.clusterXY[i,0]=dist*cos(radians(theta))
                self.clusterXY[i,1]=dist*sin(radians(theta))

                if clusterStart:
                    self.clusterIdx.append([i])
                    clusterStart = 0
                d1=rangeData[i][1]
                d2=rangeData[i+1][1]
                t1=rangeData[i][0]
                t2=rangeData[i+1][0]
                dist=sqrt((d1*sin(radians(t1))-d2*sin(radians(t2)))**2 + (d1*cos(radians(t1))-d2*cos(radians(t2)))**2)

                #if i==slope_count+59:
                    #val = np.polyfit(self.clusterXY[slope_count:i,0],self.clusterXY[slope_count:i,1],1,full=True)
                    #print val[1],val[4]
                    #print self.clusterXY[i][0], self.clusterXY[i][1]
                    #ax.plot([0,(-c/m_head)],[c,0])


                CountDiff = i-self.clusterIdx[self.clusterCount][0]
                """
                #code to try and detect a corner based on the residual
                if (CountDiff)%10==0 and CountDiff:
                    val = np.polyfit(self.clusterXY[self.clusterIdx[self.clusterCount][0]:i+1,0],self.clusterXY[self.clusterIdx[self.clusterCount][0]:i+1,1],1,full=True)
                    print val[1]
                    res_jump = val[1] / prev_residual
                    if res_jump > 5.0:
                        self.clusterIdx[self.clusterCount].append(i)
                        if fabs(rangeData[self.clusterIdx[self.clusterCount][0]][0]) <150 and fabs(rangeData[self.clusterIdx[self.clusterCount][1]][0]) <150:
                            self.clusterIdx.pop(self.clusterCount)
                            clusterStart = 1
                        else:
                            clusterStart = 1
                            self.clusterCount+=1
                        continue
                    prev_residual = val[1]
                """
                """
                #Maximum cluster size is set to 50
                if CountDiff>=49:
                    self.clusterIdx[self.clusterCount].append(i)
                    if fabs(rangeData[self.clusterIdx[self.clusterCount][0]][0]) >scan_angle_limit and fabs(rangeData[self.clusterIdx[self.clusterCount][1]][0]) >scan_angle_limit:
                        self.clusterIdx.pop(self.clusterCount)
                        clusterStart = 1
                    else:
                        clusterStart = 1
                        self.clusterCount+=1
                """

                #If distance between two consecutive data points is greater than 50cm then start a new cluster
                if dist >0.5 or CountDiff>=49:
                    self.clusterIdx[self.clusterCount].append(i)
                    if fabs(rangeData[self.clusterIdx[self.clusterCount][0]][0]) >scan_angle_limit and fabs(rangeData[self.clusterIdx[self.clusterCount][1]][0]) >scan_angle_limit:
                        self.clusterIdx.pop(self.clusterCount)
                        clusterStart = 1
                    else:
                        clusterStart = 1
                        self.clusterCount+=1
                #if reached end of rangeData then add the last data and end cluster
                elif i==count-2:
                    #print self.clusterIdx[self.clusterCount][0], i+1
                    #print rangeData[self.clusterIdx[self.clusterCount][0]][0] , rangeData[i+1][0]
                    self.clusterIdx[self.clusterCount].append(i+1)

                    if fabs(rangeData[self.clusterIdx[self.clusterCount][0]][0]) >scan_angle_limit and fabs(rangeData[self.clusterIdx[self.clusterCount][1]][0]) >scan_angle_limit:
                        self.clusterIdx.pop(self.clusterCount)
                        dist=rangeData[i+1][1]
                        theta=rangeData[i+1][0]
                        self.clusterXY[i+1,0]=dist*cos(radians(theta))
                        self.clusterXY[i+1,1]=dist*sin(radians(theta))
                        clusterStart = 1
                    else:
                        clusterStart = 1
                        self.clusterCount+=1
                        dist=rangeData[i+1][1]
                        theta=rangeData[i+1][0]
                        self.clusterXY[i+1,0]=dist*cos(radians(theta))
                        self.clusterXY[i+1,1]=dist*sin(radians(theta))
            #print self.clusterIdx
            #print "clusterXY",self.clusterXY

class EnclosingEllipse:
    def __init__(self,cluster):
        clusterXY = cluster.clusterXY
        clusterIdx = cluster.clusterIdx
        #length = len(clusterIdx)
        self.centroid = np.zeros((len(clusterIdx),2),dtype=np.float)
        self.a = np.zeros((len(clusterIdx),1),dtype=np.float)
        self.b = np.zeros((len(clusterIdx),1),dtype=np.float)
        self.m = np.zeros((len(clusterIdx),1),dtype=np.float)

        for i in xrange(len(clusterIdx)):
            icluster = clusterXY[clusterIdx[i][0]:clusterIdx[i][1]+1]
            if len(icluster)>3:
                hull = ConvexHull(icluster)
                #plot only the valid clusters i.e. the onces within the 120 deg FOVr
                #plt.scatter(icluster[:,0],icluster[:,1])

                CvxHull =icluster[hull.vertices,:]
                self.centroid[i][:] = [np.average(CvxHull[:,0]), np.average(CvxHull[:,1])]
                #print "centroid", self.centroid

                """
                if DobsDetect:
                    print "obsData", obsData
                    t = -(obsData[2] - PoseData[2])

                    if t < -pi:
                        t=t+2*pi
                    elif t >pi:
                        t=t-2*pi

                    if t <0:
                        t=t+2*pi

                    print "obs", obsData[2], "bot", PoseData[2], "rel heading", t
                    X,Y = lvls.CalculateLevelSet(t)
                    l = len(X)
                    X=np.zeros((1,l),dtype=np.float)+X
                    Y=np.zeros((1,l),dtype=np.float)+Y
                    x=np.ones((1,l),dtype=np.float) * self.centroid[i][0]
                    y=np.ones((1,l),dtype=np.float) * self.centroid[i][1]

                    xy = np.concatenate((x,y),axis=0)

                    XY = np.concatenate((X,Y),axis=0)
                    R = [[cos(t), -sin(t)],[sin(t),cos(t)]]
                    XY = np.matmul(R,XY)

                    XY = XY + xy
                    XY = np.transpose(XY)

                    Obshull = ConvexHull(XY)
                    ObsCvxHull =XY[Obshull.vertices,:]
                    ax.plot(XY[Obshull.vertices,0], XY[Obshull.vertices,1], 'r--', lw=2)

                    ax.scatter(XY[:,0],XY[:,1])

                    """
                if doPlot:
                    ax.plot(icluster[hull.vertices,1], icluster[hull.vertices,0], 'r--', lw=2)
                cent_temp=np.ones((len(CvxHull),2),dtype=np.float)
                cent_temp[:,0] =self.centroid[i][0]*cent_temp[:,0]
                cent_temp[:,1] =self.centroid[i][1]*cent_temp[:,1]
                diff=CvxHull-cent_temp
                distances=np.sqrt(np.square(diff[:,0])+np.square(diff[:,1]))

                val = np.polyfit(icluster[:,0],icluster[:,1],1, full=True)
                self.m[i] = val[0][0]
                c=val[0][1]

                #print "cluster:",i,"residual:",val[1],"rcond",val[4]
                alpha = np.arctan(np.divide(diff[:,1],diff[:,0]))
                delta = atan(self.m[i])*np.ones((len(alpha),1),dtype=np.float)
                distX = np.fabs(np.multiply(distances,np.cos(alpha-delta)))
                distY = np.fabs(np.multiply(distances,np.sin(alpha-delta)))

                self.a[i]=np.max(distX)
                self.b[i]=np.max(distY)
                if self.a[i] <0.1:
                    self.a[i] =0.1
                if self.b[i] <0.1:
                    self.b[i] =0.1

class Waypoint:
    def __init__(self):
        self.waypoint = [0,0,0]
        self.subWaypoint = rospy.Subscriber('/GV/set_goal',Vector3,self.callback)

    def callback(self,msg):
        self.waypoint = [msg.x, msg.y, msg.z]

def dyn_obs_main():
        RosNodeName = 'dyn_obs_polygon'
	rospy.init_node(RosNodeName,anonymous=True)
        lidarPointsLabelled_listener = LidarPointsLabelledListener()
        #lidarPoints_listener = LidarPointsListener()
        humanState_listener = HumanStateListener()
        #encoderIMU_listener = EncoderIMUListener()
        icp_listener = ICPListener()
        lvls = LvlSetGenerator("low")
        rate_init = rospy.Rate(10.0)
        rate_init.sleep()

        pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)

        #WP = Waypoint()

        #MPC=doMPC()

        while not rospy.is_shutdown():
            start = time.time()
            #waypoint = WP.waypoint
            #if doGoalOnly == False:
            #    rangeData = lidarPoints_listener.RangeData
            #else:
            #    rangeData = []
            staticData = lidarPointsLabelled_listener.StaticData
            personData = lidarPointsLabelled_listener.PersonData

            #robot_state = encoderIMU_listener.State # wrt to global origin
            robot_state = icp_listener.State # wrt to global origin
            #robot_twist = encoderIMU_listener.Twist # wrt to global frame

            human_state = humanState_listener.State # wrt to GV body frame
            #print "robot state", robot_state

            obstacle_msg = ObstacleArrayMsg()
            obstacle_msg.header.stamp = rospy.Time.now()
            obstacle_msg.header.frame_id = "odom"

            # Add polygon obstacle
            obstacle_msg.obstacles.append(ObstacleMsg())
            obstacle_msg.obstacles[0].id = 99

            if len(human_state):
                #print human_state
                vx=human_state[0][2]
                vy=-human_state[0][3]
                if fabs(vx)<=0.1 and fabs(vy)<=0.1:
                    vx=0.0
                    vy=0.0
                if vx==0.0:
                    vx=1e-5
                t=atan(vy/vx)
                if vx*vy>=0.0:
                    if vx<=0.0:
                        t=t+pi
                else:
                    if vx<0.0:
                        t=t+pi
                    elif vy<0.0:
                        t=t+2*pi
                print "obs", human_state[0][0], human_state[0][1]
                #print "angle", t

                yaw = math.atan2(vy, vx)
                q = tf.transformations.quaternion_from_euler(0,0,yaw)
                obstacle_msg.obstacles[0].orientation = Quaternion(*q)

                obstacle_msg.obstacles[0].polygon.points = [Point32()]
                obstacle_msg.obstacles[0].polygon.points[0].x = robot_state[0]+human_state[0][0]
                obstacle_msg.obstacles[0].polygon.points[0].y = robot_state[1]-human_state[0][1]
                obstacle_msg.obstacles[0].polygon.points[0].z = 0.0

                obstacle_msg.obstacles[0].velocities.twist.linear.x = vx
                obstacle_msg.obstacles[0].velocities.twist.linear.y = vy
                obstacle_msg.obstacles[0].velocities.twist.linear.z = 0
                obstacle_msg.obstacles[0].velocities.twist.angular.x = 0
                obstacle_msg.obstacles[0].velocities.twist.angular.y = 0
                obstacle_msg.obstacles[0].velocities.twist.angular.z = 0
                """
                X,Y = lvls.CalculateLevelSet(t)
                l = len(X)
                X=np.zeros((1,l),dtype=np.float)+X
                Y=np.zeros((1,l),dtype=np.float)+Y
                X=X/3
                Y=Y/3
                x=np.ones((1,l),dtype=np.float) * (robot_state[0]+human_state[0][0])
                y=np.ones((1,l),dtype=np.float) * (robot_state[1]-human_state[0][1])

                xy = np.concatenate((x,y),axis=0)

                XY = np.concatenate((X,Y),axis=0)
                R = [[cos(t), -sin(t)],[sin(t),cos(t)]]
                XY = np.matmul(R,XY)

                XY = XY + xy
                XY = np.transpose(XY)

                Obshull = ConvexHull(XY)
                ObsCvxHull =XY[Obshull.vertices,:]
                len_poly = len(ObsCvxHull)
                #print ObsCvxHull
                #v=Point32()


                for i in xrange(len_poly):
                    obstacle_msg.obstacles[0].polygon.points.append(Point32())
                    obstacle_msg.obstacles[0].polygon.points[i].x = ObsCvxHull[i][0]
                    obstacle_msg.obstacles[0].polygon.points[i].y = ObsCvxHull[i][1]


                #print obstacle_msg.obstacles[0].polygon.points
                """
                pub.publish(obstacle_msg)


            #else:
            #    print "angle"


            """
            v1 = Point32()
            v1.x = -1
            v1.y = -1
            v2 = Point32()
            v2.x = -0.5
            v2.y = -1.5
            v3 = Point32()
            v3.x = 0
            v3.y = -1
            #print v1
            obstacle_msg.obstacles[0].polygon.points = [v1, v2, v3]
            obstacle_msg.obstacles[0].polygon.points.append(v1)
            #print obstacle_msg.obstacles[0].polygon.points
            """


            rate_init.sleep()
            #if len(human_state):
                #print "human state", human_state[0]
                #if doPlot:
                    #ax.plot(human_state[0][1], human_state[0][0],color='blue',marker='D')


            #Cluster = clusters(rangeData)
            #staticCluster = clusters(staticData)
            #personCluster = clusters(personData)

            if doPlot:
                ax.scatter(Cluster.clusterXY[:,1], Cluster.clusterXY[:,0], color='black')
                #ax.scatter(staticCluster.clusterXY[:,1], staticCluster.clusterXY[:,0], color='black')
                #ax.scatter(personCluster.clusterXY[:,1], personCluster.clusterXY[:,0], color='green')


            #elliHull = EnclosingEllipse(Cluster)
            #elliHull = EnclosingEllipse(staticCluster)


            #count = len(Cluster.clusterIdx)
            #count = len(staticCluster.clusterIdx)
            """
            for i in xrange(count):
                #print elliHull.a[i], elliHull.b[i]
                centroid = [elliHull.centroid[i][1],elliHull.centroid[i][0]]
                if fabs(elliHull.m[i]) < 1e-6:
                    if elliHull.m[i] >0:
                        elliHull.m[i] = 0.001
                    else:
                        elliHull.m[i] = -0.001
                if doPlot:
                    elli=patches.Ellipse(xy=centroid,width=2*elliHull.a[i], height=2*elliHull.b[i], angle=degrees(atan(1/(elliHull.m[i]))), fc='None')
                    ax.add_artist(elli)

            if doPlot:
                ax.grid()
                ax.axis([-5,5,0,5])
                ax.set_title('Obstacle position w.r.t to robot')
                plt.pause(0.05)

                #plt.draw()

                ax.cla()
            """
            end = time.time()

            #print "computation time:", end - start

#############################################################################################################################
# running
if __name__ == '__main__':
    try:
        dyn_obs_main()
    except rospy.ROSInterruptException:
        pass


