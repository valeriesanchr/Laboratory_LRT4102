#!/usr/bin/env python3
 
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, radians

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
        self.error_accumulation_x = 0
        self.last_error_y = 0
        self.error_accumulation_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # This function is called every time the turtle's position is updated
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_x_y(self, desired_x, desired_y):
        # PID constants for linear movement (X and Y)
        Kp = 1
        Ki = 0.01
        Kd = 0.1

        while not rospy.is_shutdown():
            # Calculate the position error
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Accumulate errors for integral term
            self.error_accumulation_x += error_x
            self.error_accumulation_y += error_y
            
            # Calculate linear velocities using PID
            vel_x = Kp * error_x + Ki * self.error_accumulation_x + Kd * (error_x - self.last_error_x)
            vel_y = Kp * error_y + Ki * self.error_accumulation_y + Kd * (error_y - self.last_error_y)
            
            # Save the current error for the next iteration
            self.last_error_x = error_x
            self.last_error_y = error_y
            
            # Create a Twist message to send movement commands
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            
            # Publish the movement command
            self.velocity_publisher.publish(twist_msg)
            
            # Log the current position, error, and velocity
            rospy.loginfo("Current position x: %f, Error: %f, Linear velocity: %f", self.current_x, error_x, vel_x)
            rospy.loginfo("Current position y: %f, Error: %f, Linear velocity: %f", self.current_y, error_y, vel_y)
            
            # Check if the desired position is reached
            if abs(error_x) < 0.1 and abs(error_y) < 0.1:
                rospy.loginfo("Desired position reached")
                break
            
            # Wait for the next iteration
            self.rate.sleep()

    def move_turtle_to_desired_theta(self, desired_theta):
        # PD constants for angular movement (rotation)
        Kp_theta = 0.3  # Use a small proportional gain
        Kd_theta = 0.1  # Use a small derivative gain

        while not rospy.is_shutdown():
            # Calculate the angle error
            error_theta = desired_theta - self.current_theta
            
            # Normalize the angle error to [-pi, pi]
            while error_theta > pi:
                error_theta -= 2 * pi
            while error_theta < -pi:
                error_theta += 2 * pi

            # Calculate angular velocity using PD control (no integral term)
            angular_z = Kp_theta * error_theta + Kd_theta * (error_theta - self.last_error_theta)
            
            # Create a Twist message for angular movement
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            
            # Publish the angular velocity command
            self.velocity_publisher.publish(twist_msg)
            
            # Log the current angle, error, and angular velocity
            rospy.loginfo("Current orientation: %f, Error: %f, Angular velocity: %f", self.current_theta, error_theta, angular_z)

            # Check if the desired orientation is reached
            if abs(error_theta) < 0.05:  # Larger tolerance to avoid overshoot
                rospy.loginfo("Desired orientation reached")
                break

            # Save the current angle error for the next iteration
            self.last_error_theta = error_theta
            
            # Wait for the next iteration
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

            # Move the turtle to the desired position
            self.move_turtle_to_desired_x_y(desired_x, desired_y)

            # Rotate the turtle to the desired orientation
            self.move_turtle_to_desired_theta(desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
