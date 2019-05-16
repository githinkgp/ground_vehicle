#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Vector3

def talker(argv):
    wp_pub = rospy.Publisher('/mybot/set_goal', Vector3, queue_size=10)
    rospy.init_node('mybot_goal', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    print argv
    wp_msg = Vector3()
    wp_msg.x = float(argv[1])
    wp_msg.y = float(argv[2])
    wp_msg.z = float(argv[3])

    while not rospy.is_shutdown():
     #   hello_str = "hello world %s" % rospy.get_time()
      #  rospy.loginfo(hello_str)
       # pub.publish(hello_str)
        #while t1-t0<=1:

        wp_pub.publish(wp_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker(sys.argv)
    except rospy.ROSInterruptException:
        pass
