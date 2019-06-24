# Sphero Robot - ROS

In this project, I wrote a ROS program to move the Sphere robot in a maze. Sphero provides the following sensors and actuators:

Sensors:
- IMU sensor data which provides information about acceleration
    and orientation. (Published on /sphero/imu/data3 topic)
- Odometry data which can be accessed through the /odom topic

Actuators:
- Speed Topic on which speed commands can be sent to move the robot through the /cmd_vel topic.

The main launch file launches the following 03 nodes:

 - Odometery Action Server: Tells if a specified time period has passed in the maze or if the robot has existed the maze, whichever happens first. The action server uses a custom message of following structure:
 ```
# goal. empty
---
#result, Odom array
nav_msgs/Odometry[] result_odom_array
---
#feedback, empty
```



 - Crash Service Server: Tells if the robot has collided with the walls of maze (with acceleration greater than a predefined threshold value) and returns the direction to move the robot to avoid collision. The service server use a message of
```std_msgs/Trigger.msg``` of following structure:

```
# request, Empty because no data is needed
---
#response
bool movement_successfull
string extra_data #for robot's direction
```
 - Main program: It has the action client and the service client to call the respective action/client server. It manage the overall flow of the program.

 To run the project, move to ```catkin_ws/``` directory and run the following commands:
 ```
 catkin_make
source devel/setup.bash
```
 To launch the sp_main.launch file, type the following command:

    roslaunch my_sphero_main sp_main.launch

