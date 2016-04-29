#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def detector():
    rospy.init_node('imu_collision', anonymous=True)
    pub = rospy.Publisher('imu9250_collision', String, queue_size=10)
    rate = rospy.Rate(10)
    impact_acceleration = 0.7 #m/s^2 (earth gravity 0.98)
    while not rospy.is_shutdown():
        msg_imu = rospy.wait_for_message('/imu9250', Imu, timeout=5)

        acc_x = msg_imu.linear_acceleration.x
        acc_z = msg_imu.linear_acceleration.z

        collision = "false"
        if abs(acc_x) > impact_acceleration:
            collision = "true"
        if acc_z < 0:
            collision = "rollover"
        pub.publish(collision)
        rate.sleep()

if __name__ == '__main__':
    detector()
