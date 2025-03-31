#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, sqrt

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Subscribe to the turtle's position topic
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publish to the turtle's movement command topic
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Set the rate of publishing messages (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # This function is called every time the turtle's position is updated
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_x_y_theta(self, desired_x, desired_y, desired_theta):
        # Proportional and Derivative gains for the controller (adjustable)
        Kp_linear = 1.0  # Proportional gain for linear velocity
        Kd_linear = 0.1  # Derivative gain for linear velocity
        Kp_angular = 0.8  # Proportional gain for angular velocity
        Kd_angular = 0.1  # Derivative gain for angular velocity

        # 1. First, move towards the desired position (translation)
        while not rospy.is_shutdown():
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y

            # Calculate distance to the desired position
            distance = sqrt(error_x**2 + error_y**2)

            # Calculate the velocities considering the proportional and derivative control
            vel_x = Kp_linear * error_x + Kd_linear * (error_x - self.last_error_x)
            vel_y = Kp_linear * error_y + Kd_linear * (error_y - self.last_error_y)

            # Publish velocity commands
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            self.velocity_publisher.publish(twist_msg)

            # Logging info
            rospy.loginfo("Moving to position: x = %f, y = %f, Error x = %f, Error y = %f, Vel x = %f, Vel y = %f", 
                          self.current_x, self.current_y, error_x, error_y, vel_x, vel_y)

            if distance < 0.1:
                rospy.loginfo("Position reached")
                break

            # Update the last error for the next iteration
            self.last_error_x = error_x
            self.last_error_y = error_y

            self.rate.sleep()

        # 2. Then, rotate the turtle to the desired angle (rotation in z)
        while not rospy.is_shutdown():
            error_theta = desired_theta - self.current_theta

            # Ensure the error angle is within [-pi, pi]
            while error_theta > pi:
                error_theta -= 2 * pi
            while error_theta < -pi:
                error_theta += 2 * pi

            # Calculate angular velocity considering the proportional and derivative control
            angular_z = Kp_angular * error_theta + Kd_angular * (error_theta - self.last_error_theta)

            # Publish angular velocity command
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            self.velocity_publisher.publish(twist_msg)

            # Logging info
            rospy.loginfo("Correcting orientation: θ = %f, Error θ = %f, Angular velocity = %f", 
                          self.current_theta, error_theta, angular_z)

            if abs(error_theta) < 0.05:  # If the angle error is small enough, stop rotating
                rospy.loginfo("Orientation reached")
                break

            # Update the last error for the next iteration
            self.last_error_theta = error_theta

            self.rate.sleep()

    def get_desired_x_from_user(self):
        print("Enter the desired x position:")
        return float(input("Coordinate x: "))
        
    def get_desired_y_from_user(self):
        print("Enter the desired y position:")
        return float(input("Coordinate y: "))
    
    def get_desired_theta_from_user(self):
        print("Enter the desired orientation in theta (radians):")
        return float(input("Theta: "))
        
    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Get the desired position from the user
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()
            desired_theta = self.get_desired_theta_from_user()

            # Move the turtle to the desired position and orientation
            self.move_turtle_to_desired_x_y_theta(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
