# Laboratory 2 Report
For this lab exercise, we created several programs that tested our knowledge in different areas.

## General considerations

All the codes used for this lab were written in python, and then executed with ROS. In order to test the codes, I wrote both the original code and its launch file. In order to run them, I typed this command in the terminal:

## Lab2 Basic
For this exercise, we had two files: listener.py, and talker.py. We had to do the following:

* Create a ROS package named Practicas_lab with dependencies on rospy, roscpp, and std_msgs.
* Place the files listener.py and talker.py in the package.
* Compile the package.
* Run the talker node.
* Run the listener node.
* Analyze and conclude its functionality..

This is the full code for the listener side: 
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
And this is the full code for the talker:

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
If we run both codes at the same time, this is what we obtain in the terminal:
![image](https://github.com/user-attachments/assets/ca0eb402-e3cd-4bf3-b4f9-ab9dc60e1749)

This code is also known as a "publisher and susbcriber". The ROS talker node starts with a shebang declaration to ensure the script runs as a Python script within the ROS environment. It then imports rospy, which is necessary for creating ROS nodes, and std_msgs.msg.String to enable the use of the standard String message type for communication.

Next, the script defines a publisher that sends messages to the "chatter" topic using the String message type. The queue_size parameter is included to manage message storage in case subscribers are slow to process them. The node is then initialized with the name "talker," and the anonymous=True flag ensures it gets a unique name, preventing conflicts when multiple instances run simultaneously.

To regulate the message transmission rate, a rate object is created to maintain a loop frequency of 10 Hz. The main loop continuously executes until the node is shut down. During each iteration, a "hello world" message is generated, including a timestamp, and logged for debugging. The message is then published to the "chatter" topic, and the loop pauses momentarily to maintain the desired frequency.

For debugging and monitoring, rospy.loginfo() is used, allowing messages to be printed to the console, logged in a file, and sent to rosout, making it easier to track node activity. Finally, the script includes exception handling to catch rospy.ROSInterruptException, which prevents errors when the node is stopped using Ctrl+C or any other shutdown command.

With the talker node in place, the next step is to implement a listener node to receive and process the published messages. The listener.py node in ROS subscribes to the "chatter" topic, which transmits messages of type std_msgs.msg.String. When a new message is received, a callback function is triggered to process it.

To ensure each node has a unique name, the anonymous=True argument is added to rospy.init_node(), preventing conflicts if multiple listeners run simultaneously.

Finally, rospy.spin() is used to keep the node running until it is manually stopped, ensuring it continuously listens for incoming messages.

## Lab2 Medium
For this part of the lab, we had to do the following:
* Create a keyboard control for turtlesim.
* Draw a square and an equilateral triangle with turtlesim (without a controller).

In order to be able to read the input from our keyboard, we had to implement the folllowing function:
```python
def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key
```
Keyboard control:
![image](https://github.com/user-attachments/assets/92481d65-3092-430e-a615-86bd898155e1)
The keys WASD control the direction of the turtle, and allow it move forward, backwards, up and down.

Square:
![image](https://github.com/user-attachments/assets/f3d95066-19f9-49cb-801e-321af4e550f7)

Triangle:
![image](https://github.com/user-attachments/assets/44a090d9-ea8f-4d6f-ad5c-803be8f128ad)

These codes were fairly simple, since I only needed to make a few changes so that the turtle could make the shapes. 

## Lab2 Advanced
For this final part, the instructions were as follows:

* Position control for turtlesim (P)
* Position control for turtlesim (PI)
* Position control for turtlesim (PID)
* Compare the performance of each controller using Plot Juggler or another plotting tool.
* Report the results in Markdown.

Proportional control (P)

![image](https://github.com/user-attachments/assets/3657a7be-19e3-4c5e-9079-3ce13f71e123)

Proportional integral control (PI)
![image](https://github.com/user-attachments/assets/43ea7afc-b54b-486c-bc9e-f74f649ff8a6)


Proportional integral derivative control (PID)

![image](https://github.com/user-attachments/assets/c8bab0ef-10ac-44da-9d22-d7a1f5bcb916)



# Referencias

1. W3Schools.com. (n.d.). https://www.w3schools.com/python/python_variables.asp
2. Amos, D. (2024, December 15). Object-Oriented Programming (OOP) in Python. https://realpython.com/python3-object-oriented-programming/#what-is-object-oriented-programming-in-python
3. ¿Qué es Python? - Explicación del lenguaje Python - AWS. (n.d.). Amazon Web Services, Inc. https://aws.amazon.com/es/what-is/python/









