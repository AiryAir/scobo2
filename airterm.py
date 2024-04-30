#!/usr/bin/env python

# this is a test file. do not run.

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

left_wheel_vel = 0.0
right_wheel_vel = 0.0

def cmd_vel_callback(msg):
    global left_wheel_vel, right_wheel_vel

    linear_x = msg.linear.x
    angular_z = msg.angular.z

    # Calculate wheel velocities for a differential drive robot
    left_wheel_vel = linear_x - (angular_z / 2.0)
    right_wheel_vel = linear_x + (angular_z / 2.0)

def main():
    rospy.init_node('velocity_publisher', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    left_vel_pub = rospy.Publisher('/left_vel', Float32, queue_size=10)
    right_vel_pub = rospy.Publisher('/right_vel', Float32, queue_size=10)

    # Ask user for linear x and angular z velocities
    linear_x = float(input("Enter linear velocity (m/s): "))
    angular_z = float(input("Enter angular velocity (rad/s): "))

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Calculate wheel velocities for a differential drive robot
        left_wheel_vel = linear_x - (angular_z / 2.0)
        right_wheel_vel = linear_x + (angular_z / 2.0)

        # Publish individual wheel velocities
        left_vel_pub.publish(left_wheel_vel)
        right_vel_pub.publish(right_wheel_vel)

        rate.sleep()

if __name__ == '__main__':
    main()

