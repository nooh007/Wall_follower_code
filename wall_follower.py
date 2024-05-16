#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.target_distance = 1.3# Desired distance from the wall
        self.kp = 0.6  # Proportional gain
        self.ki = 0 # Integral gain
        self.kd = 0.01  # Derivative gain
        
        self.prev_error = 0.0
        self.integral = 0.0
        
        rospy.spin()

    def scan_callback(self, data):
        ranges = data.ranges
        # Filter out nan values
        ranges = [r for r in ranges if not math.isnan(r)]
        
        # Choose the closest valid range reading
        min_distance = min(ranges)
        min_index = ranges.index(min_distance)
        
        # Compute error (difference between current distance and target distance)
        error = min_distance - self.target_distance
        
        # Compute PID control output
        control_output = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        
        # Update integral term for the next iteration
        self.integral += error
        
        # Save current error for the next iteration
        self.prev_error = error
        
        # Create Twist message to control the robot
        twist = Twist()
        twist.linear.x = 0.2  # Forward velocity
        twist.angular.z = -control_output  # Angular velocity (negative because turning left)
        
        # Publish Twist message
        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        WallFollower()
    except rospy.ROSInterruptException:
        pass
