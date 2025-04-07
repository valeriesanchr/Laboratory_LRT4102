*Laboratory 3*

For this laboratory, we had to solve the following exercises:

* Calculate and display the DTG (Distance To Goal) and ATG (Angle To Goal).

* Do not move the robot; spawn it at the Goal position.

* Explain the necessary mapping for the velocities.

* Use a (free choice) controller to drive the turtle to the desired position, doing it in an infinite loop.

**Introduction**

In this lab, we explore fundamental concepts of mobile robot navigation using the turtlesim simulator in ROS. The main objective is to develop a control strategy that enables a robot (turtle) to reach a specified goal position. The exercise is divided into multiple steps: first, we compute and display two essential metrics for navigation—Distance to Goal (DTG) and Angle to Goal (ATG)—which indicate how far and in what direction the robot must move to reach the target. Instead of allowing the robot to navigate from an arbitrary starting point, we directly spawn the robot at the goal location to ensure consistent initial conditions.

Additionally, the lab involves analyzing how linear and angular velocities must be mapped appropriately based on the robot's current state relative to the goal. Finally, we implement a custom controller that continuously updates the robot’s velocity in an infinite loop, ensuring that it gradually moves toward the target location. This experiment reinforces key robotics concepts such as coordinate transformation, motion planning, and feedback control, which are crucial for autonomous navigation.

***Exercise 1***
For this exercise, we must calculate and display the Distance To Goal and Angle to Goal. The user will input the desired rotation and movement, and the program must kill the existing turtle and spawn it in the desired position. Since the turtle dies and respawns at the goal location, the DTG and ATG will be zero.

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
