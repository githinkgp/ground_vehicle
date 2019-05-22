import rospy
from mavros_msgs.msg import ActuatorControl

def actuator_control(U):
    actuator_publisher = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
    act_msg = ActuatorControl()

    rate = rospy.Rate(100) # 10hz

    act_msg.controls[0] = 0.0
    act_msg.controls[1] = 0.0
    act_msg.controls[2] = 0.0
    act_msg.controls[3] = 0.0
    act_msg.controls[4] = 0.0
    act_msg.controls[5] = 0.0
    act_msg.controls[6] = 0.0
    act_msg.controls[7] = 0.0

    if not rospy.is_shutdown():
        if U[0] >0.7:
            U[0] = 0.7
        act_msg.header.stamp = rospy.Time.now();
        act_msg.group_mix = 0;
        act_msg.controls[0] = U[1]
        act_msg.controls[1] = 0.0
        act_msg.controls[2] = 0.0
        act_msg.controls[3] = 0.0
        act_msg.controls[4] = 0.0
        act_msg.controls[5] = 0.0
        act_msg.controls[6] = 0.0
        act_msg.controls[7] = 0.0

        actuator_publisher.publish(act_msg)

        rate.sleep()


