#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import pid
import numpy as np
import time

class lane_follow():
    def __init__(self):
        rospy.Subscriber("/error", Float64, self.callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.kp = 1.0
        self.kd = 0.0
        self.ki = 0.0
        self.last_error = 0
        self.integral = 0
        self.derivative = 0
        self.last_time = rospy.get_time()

    def callback(self, data):
        pid_twist = self.pid(data.data)
        self.velocity_publisher.publish(pid_twist)

    def pid (self, error):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        if dt == 0:
            return Twist()
        self.integral += error * dt
        self.derivative = (error - self.last_error) / dt

        f = self.kp * error + self.ki * self.integral + self.kd * self.derivative

        self.last_error = error
        self.last_time = current_time

        twist = Twist()

        twist.linear.x = 0.9
        twist.angular.z = -f/100
        return twist

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    lane_follow()
    rospy.spin()
