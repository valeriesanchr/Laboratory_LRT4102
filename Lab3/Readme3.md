*Laboratory 3*

For this laboratory, we had to solve the following exercises:

* Calculate and display the DTG (Distance To Goal) and ATG (Angle To Goal).

* Do not move the robot; spawn it at the Goal position.

* Explain the necessary mapping for the velocities.

* Use a (free choice) controller to drive the turtle to the desired position, doing it in an infinite loop.

**Introduction**
In this lab, we explore fundamental concepts of mobile robot navigation using the turtlesim simulator in ROS. The main objective is to develop a control strategy that enables a robot (turtle) to reach a specified goal position. The exercise is divided into multiple steps: first, we compute and display two essential metrics for navigation—Distance to Goal (DTG) and Angle to Goal (ATG)—which indicate how far and in what direction the robot must move to reach the target. Instead of allowing the robot to navigate from an arbitrary starting point, we directly spawn the robot at the goal location to ensure consistent initial conditions.

Additionally, the lab involves analyzing how linear and angular velocities must be mapped appropriately based on the robot's current state relative to the goal. Finally, we implement a custom controller that continuously updates the robot’s velocity in an infinite loop, ensuring that it gradually moves toward the target location. This experiment reinforces key robotics concepts such as coordinate transformation, motion planning, and feedback control, which are crucial for autonomous navigation.
