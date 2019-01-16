![imagen](https://raw.githubusercontent.com/ctaipuj/lenny_motoman/master/lenny.png)
# motoman\_sda10f\_moveit\_config Package
## Overview
This package  is forked from the original [motoman package](https://github.com/ros-industrial/motoman) kinetic devel. It contains a particular configuration for Lenny generated using MoveIt!'s setup assistant.

## Contents

1. Config  
Configuration files (YAML files).
2. launch  
Launch files.  

## Usage

### Editing Files:
It is not recommended to edit these files by hand. Instead the setup assistant should be used.

In a new terminal run:

`roslaunch motoman_sda10f_moveit_config setup_assistant.launch` 

Follow the instructions in screen.

### Demo test using MoveIt!:

`roslaunch motoman_sda10f_moveit_config demo.launch`

You can start using Rviz graphic interface.

### Using Lenny with MoveIt!:

Make sure the robot is propertly connected to the network (execute a ping test)

**When working in a distributed system three ROS variables must be configured first:**

```
cd
gedit .bashrc
```
Go to the bottom line of the file and add these three lines 

``` sh
export ROS_IP=localmachine_ip #TODO
export ROS_HOSTNAME= mastermachine_ip #TODO
export ROS_MASTER_URI= mastermachine_ip:11311 #TODO
```

Save the file and close all terminals.

**To bring up Lenny execute:**

`roslaunch motoman_sda10f_moveit_config moveit_planning_execution.launch`

You can start using Rviz graphic interface.

Default parameters:

```xml
robot_ip=10.21.7.15
controller=fs100
simple_view=true
right_gripper=false
```

To allow ROS to command the robot you must call first the **robot_enable** service.

**WARNING**

When the **robot_enable** service is called the robot will start to receive ROS commands and it could move. **Make sure the area is clear and the teach pendant is reachable.** If not sure about the robot state or unexpected behavior occurs press the emergency stop button.

* Check the teach pendant key. It must be in remote mode position.
*  Check the emergency stop button, it must be released.
*  In a terminal execute:

`rosservice call robot_enable`

The servos should power up. You will hear servos noice.

## Video

SDA10F robot used with MoveIt! framework.

[![video](https://img.youtube.com/vi/mS05tVPRCq0/0.jpg)](https://www.youtube.com/watch?v=mS05tVPRCq0)

## Publications

The particular setup of the SDA10F robot is described in [*"Setup of the Yaskawa SDA10F Robot for Industrial Applications, Using ROS-Industrial"*](https://www.researchgate.net/publication/315063645_Setup_of_the_Yaskawa_SDA10F_Robot_for_Industrial_Applications_Using_ROS-Industrial).

## NOTE

* The **demo.launch, move\_group.launch, moveit\_planning\_execution.launch, planning\_context.launch, trajectory_execution.launch.xml** files were hand-edited please check changes within each file.
*  Since Lenny works with two collision matrixes two srdf files were created. One for the simple view environment and the other one for the full environment.

***
![imagen](https://bit.ly/2QOK5D6)  
Keep up with CTAI new developments! Watch our [YouTube Channel](https://www.youtube.com/channel/UC06RetpipAkfxl98UfEc21w). 
Don't forget to subcribe!
***
*Created by Nicolas Barrero Jan 2019*    
**"Centro Tecnologico de Automatizacion Industrial" CTAI  
Perception For Industrial Robots Project**

![imagen](https://bit.ly/2qVzHyL)
