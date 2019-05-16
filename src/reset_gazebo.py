#!/usr/bin/env python

import rospy
import sys
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3

def talker():
    reset_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    wp_pub = rospy.Publisher('/mybot/set_goal', Vector3, queue_size=10)
    rospy.init_node('gazebo_reset', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    reset_msg = ModelState()
    reset_msg.model_name = 'mybot'
    reset_msg.pose.position.x = 0
    reset_msg.pose.position.y = 0
    reset_msg.pose.position.z = 0

    reset_msg.pose.orientation.x = 0
    reset_msg.pose.orientation.y = 0
    reset_msg.pose.orientation.z = 0
    reset_msg.pose.orientation.w = 0

    reset_msg.twist.linear.x = 0
    reset_msg.twist.linear.y = 0
    reset_msg.twist.linear.z = 0

    reset_msg.twist.angular.x = 0
    reset_msg.twist.angular.y = 0
    reset_msg.twist.angular.z = 0
    
    reset_msg.reference_frame = ''

    wp_msg = Vector3()
    wp_msg.x = 0
    wp_msg.y = 0
    wp_msg.z = 0

    while not rospy.is_shutdown():
     #   hello_str = "hello world %s" % rospy.get_time()
      #  rospy.loginfo(hello_str)
       # pub.publish(hello_str)
        #while t1-t0<=1:

        reset_pub.publish(reset_msg)
        wp_pub.publish(wp_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
