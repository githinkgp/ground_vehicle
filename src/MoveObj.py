#!/usr/bin/env python

import rospy
import sys
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3

def talker():
    Obj_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('obj_move', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    Obj_msg = ModelState()
    Obj_msg.model_name = 'turtlebot'
    
    Obj_msg.pose.position.x = 5
    Obj_msg.pose.position.y = 5
    Obj_msg.pose.position.z = 0

    Obj_msg.pose.orientation.x = 0
    Obj_msg.pose.orientation.y = 0
    Obj_msg.pose.orientation.z = 0
    Obj_msg.pose.orientation.w = 0
    
    Obj_msg.twist.linear.x = -1.0
    Obj_msg.twist.linear.y = 0
    Obj_msg.twist.linear.z = 0

    Obj_msg.twist.angular.x = 0
    Obj_msg.twist.angular.y = 0
    Obj_msg.twist.angular.z = 0.5
    
    Obj_msg.reference_frame = ''

    while not rospy.is_shutdown():
     #   hello_str = "hello world %s" % rospy.get_time()
      #  rospy.loginfo(hello_str)
       # pub.publish(hello_str)
        #while t1-t0<=1:

        Obj_pub.publish(Obj_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
