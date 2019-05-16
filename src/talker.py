#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    velocity_publisher = rospy.Publisher('/mybot/cmd_vel', Twist, queue_size=10)
    rospy.init_node('mybot_cmd_vel', anonymous=True)
    #rate = rospy.Rate(10) # 10hz

    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    t0=time.time()

    while not rospy.is_shutdown():
     #   hello_str = "hello world %s" % rospy.get_time()
      #  rospy.loginfo(hello_str)
       # pub.publish(hello_str)
        #while t1-t0<=1:

        velocity_publisher.publish(vel_msg)
        t1=time.time()
        #t1=rospy.Time.now().to_sec()
        #print "t1",t1
        #rate.sleep()
        if t1-t0>=1:
            #vel_msg.linear.x = 0
            #vel_msg.angular.z = 0
            #velocity_publisher.publish(vel_msg)
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
