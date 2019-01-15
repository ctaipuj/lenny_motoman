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

It is not recommended to edit these files by hand. Instead the setup assistant should be used.

In a new terminal run:

`roslaunch motoman_sda10f_moveit_config setup_assistant.launch` 

## NOTE

* The **demo.launch, move\_group.launch, moveit\_planning\_execution.launch, planning\_context.launch, trajectory_execution.launch.xml** files were hand-edited please check changes within each file.
*  Since Lenny works with two collision matrixes two srdf files were created. One for the simple view environment and the other one for the full environment.

***
*Created by Nicolas Barrero Jan 2019*    
**"Centro Tecnologico de Automatizacion Industrial" CTAI  
Perception For Industrial Robots Project**

![imagen](https://bit.ly/2qVzHyL)
