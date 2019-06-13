#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def reset():
    reset_pub = rospy.Publisher('/arduino_wheel_encoder/reset_encoder', String, queue_size=10)

    rospy.init_node('localization_reset', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    reset_msg = String()
    reset_msg.data = 'reset'

    while not rospy.is_shutdown():
        reset_pub.publish(reset_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        reset()
    except rospy.ROSInterruptException:
        pass

