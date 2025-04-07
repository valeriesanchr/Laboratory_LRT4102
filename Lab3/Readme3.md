# Laboratory 3

For this laboratory, we had to solve the following exercises:

* Calculate and display the DTG (Distance To Goal) and ATG (Angle To Goal).

* Do not move the robot; spawn it at the Goal position.

* Explain the necessary mapping for the velocities.

* Use a (free choice) controller to drive the turtle to the desired position, doing it in an infinite loop.
  
## Introduction

In this lab, we explore fundamental concepts of mobile robot navigation using the turtlesim simulator in ROS. The main objective is to develop a control strategy that enables a robot (turtle) to reach a specified goal position. The exercise is divided into multiple steps: first, we compute and display two essential metrics for navigation—Distance to Goal (DTG) and Angle to Goal (ATG)—which indicate how far and in what direction the robot must move to reach the target. Instead of allowing the robot to navigate from an arbitrary starting point, we directly spawn the robot at the goal location to ensure consistent initial conditions.

Additionally, the lab involves analyzing how linear and angular velocities must be mapped appropriately based on the robot's current state relative to the goal. Finally, we implement a custom controller that continuously updates the robot’s velocity in an infinite loop, ensuring that it gradually moves toward the target location. This experiment reinforces key robotics concepts such as coordinate transformation, motion planning, and feedback control, which are crucial for autonomous navigation.

### Exercise 1
For this exercise, we must calculate and display the Distance To Goal and Angle to Goal. The user will input the desired rotation and movement, and the program must kill the existing turtle and spawn it in the desired position. Since the turtle dies and respawns at the goal location, the DTG and ATG will be zero.

![image](https://github.com/user-attachments/assets/37175fa2-3682-4f2b-9bcd-58cb21b3176b)


```python
#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import math
```
This part of the code is just the initial setup, importing the relevant libraries for the code.

```python
def kill_turtle(name):
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(name)
    except rospy.ServiceException:
        rospy.logwarn(f"No se pudo eliminar {name}.")

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta, name)
        return x, y, theta
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al crear la tortuga: {e}")
        return None
```
These two functions are used for killing and spawning the turtle respectively. They work very similarly, first calling the service (/kill and /spawn), and then calling the service proxy that will perform the action. There is also an except command, to catch any possible exception errors.

```python
def main():
    rospy.init_node('turtle_spawn_goal', anonymous=True)


    # Pedir al usuario la posición objetivo
    x_goal = float(input("Ingresa la coordenada x: "))
    y_goal = float(input("Ingresa la coordenada y: "))
    theta_goal = float(input("Ingresa el ángulo theta: "))
```
This part of the code asks the user for the desired position and angle.

```python
    # Eliminar la tortuga inicial
    kill_turtle("turtle1")

    # Crear la tortuga en la posición deseada
    result = spawn_turtle(x_goal, y_goal, theta_goal, "turtle1")
```
This calls the kill function, effectively killing the original turtle. Then, we store the spawn_turtle function in result, and we also spawn the turtle.

```python
    if result:
        x_current, y_current, theta_current = result

        # Calcular Distance to Goal (DTG)
        dtg = math.sqrt((x_goal - x_current)**2 + (y_goal - y_current)**2)

        # Calcular Angle to Goal (ATG) 
        atg = math.atan2((y_goal - y_current), (x_goal - x_current))

        print(f"\nDistance to Goal (DTG): {dtg:.4f}")
        print(f"Angle to Goal (ATG): {atg:.4f}")
```
If result stores valid data, then it becomes valid and executes the specified actions, which are calculating DTG and ATG and printing these values.

```python
if __name__ == '__main__':
    main()
```
Calls and performs the main function.

### Exercise 2
Now, for exercise 2, we have to do something similar, but now the turtle will actually perform the trajectory. 
![image](https://github.com/user-attachments/assets/31b3e9b1-1dc5-43a7-b18f-da521a7c1092)


```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pow, radians, degrees
```
Initial set up, which imports any relevant libraries.

```python
class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('turtle_proportional_controller', anonymous=True)
        
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.x = 0
        self.y = 0
        self.theta = 0
```
This class starts defining the proportional control that will be used in the code, subscribing to the turtle and calling the cmd_vel. Then, we initialize x,y and theta to zero.

```python

    def pose_callback(self, data):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta
```
This function is a callback function for the /turtle1/pose subscriber updates the turtle's current pose each time a new message is received.

```python
    def get_user_input(self):
        print("\nIngresar nueva posición objetivo")
        x = float(input("Coordenada x del objetivo: "))
        y = float(input("Coordenada y del objetivo: "))
        theta_deg = float(input("Ángulo deseado (grados): "))
        return x, y, radians(theta_deg)
```
Function for storing the data of the desired goal.

```python
    def move_to_goal(self, goal_x, goal_y):
        vel_msg = Twist()
        Kp_linear = 1.5
        Kp_angular = 6.0

        while not rospy.is_shutdown():
            # Calcular DTG y ATG usando coordenadas euclidianas
            dtg = sqrt(pow(goal_x - self.x, 2) + pow(goal_y - self.y, 2))
            atg = atan2(goal_y - self.y, goal_x - self.x)
            angle_diff = atg - self.theta

            # Normalizar el ángulo a [-pi, pi]
            angle_diff = (angle_diff + 3.14159) % (2 * 3.14159) - 3.14159

            # Velocidades proporcional al error
            vel_msg.linear.x = Kp_linear * dtg
            vel_msg.angular.z = Kp_angular * angle_diff

            self.cmd_vel_pub.publish(vel_msg)

            rospy.loginfo("DTG: %.4f | ATG: %.4f°", dtg, degrees(angle_diff))

            # Cuando se está muy cerca del objetivo, detener
            if dtg < 0.1:
                break

            self.rate.sleep()

        # Detener completamente
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)
        rospy.loginfo("Objetivo alcanzado.\n")
```
This entire function is for calculating the distance to goal, and angle to goal, and then moving the robot towards that goal. It uses the DTG and ATG formulas, and then based on that it starts publishing the linear and angular velocities. Then, once the robot is very close to the DTG objective, it stops moving. 

```python
    def rotate_to_theta(self, desired_theta):
        vel_msg = Twist()
        Kp_theta = 4.0

        while not rospy.is_shutdown():
            error_theta = desired_theta - self.theta
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159

            vel_msg.angular.z = Kp_theta * error_theta
            self.cmd_vel_pub.publish(vel_msg)

            rospy.loginfo("Error ángulo final: %.4f°", degrees(error_theta))

            if abs(error_theta) < 0.05:
                break

            self.rate.sleep()

        # Detener rotación
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)
```
The previous function was for the linear movement, and now we calculate the angular movement needed. It calculates theta's error, and then uses that to publish the angular velocity needed.

```python
    def run(self):
        while not rospy.is_shutdown():
            goal_x, goal_y, goal_theta = self.get_user_input()
            self.move_to_goal(goal_x, goal_y)
            self.rotate_to_theta(goal_theta)
```
This function is an infinite loop that gets a new goal position from the user, moves the turtle to that position, and rotates it to the final desired orientation.

```python
if __name__ == '__main__':
    try:
        controller = MoveTurtleProportionalControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```
Calls main and the controller.
