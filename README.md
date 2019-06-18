![imagen](https://raw.githubusercontent.com/ctaipuj/lenny_motoman/master/lenny.png)
# Lenny Motoman Package
## Overview
This package  is forked from the original [motoman package](https://github.com/ros-industrial/motoman) kinetic devel, preserving only packages of interest. It contains a particular configuration for Lenny, Lenny's environment setup, as well it contains the package **motoman variables** created at CTAI to interact with the FS100 controller I/Os.

This metapackage contains packages developed by the [ROS-I project](http://wiki.ros.org/Industrial).

## Contents

1. motoman Package 
2. motoman_driver Package  
It is a modified version of the [motoman\_driver](https://github.com/ros-industrial/motoman) package. The modification allows the communication with FS100 controller's network variables. 
3. motoman_msgs package  
It is a modified version of the[motoman\_msgs](https://github.com/ros-industrial/motoman) package. Two new services were created the **"ReadSingleIO.srv"** and the **"WriteSingleIO.srv"**. These services are used by the [motoman\_variables](https://github.com/ctaipuj/lenny_motoman/tree/master/motoman_variables) package.
4. [motoman\_sda\_10f\_moveit\_config](https://github.com/ctaipuj/lenny_motoman/tree/master/motoman_sda10f_moveit_config)  
This Package contains the main MoveIt! configuration for Lenny.
5. [motoman\_sda\_10f\_support](https://github.com/ctaipuj/lenny_motoman/tree/master/motoman_sda10f_support)  
This Package contains the support elements required when using Lenny.
6. [motoman_variables]() package. It was developed at CTAI. It facilitates the communication whit network variables using the **"ReadSingleIO.srv"** and the **"WriteSingleIO.srv"** services.

## Requirements

* [MoveIt!](https://moveit.ros.org)
* [Industrial core](http://wiki.ros.org/industrial_core)

## Video

* Current research:  
SDA10F robot used with MoveIt! framework for plastic bottle classification tasks. 

[![video](https://img.youtube.com/vi/F76Pe-WkP3g/0.jpg)](https://youtu.be/F76Pe-WkP3g)

## NOTE

* Please follow the links for further information about each package.

***
![imagen](https://bit.ly/2QOK5D6)  
Keep up with CTAI new developments! Watch our [YouTube Channel](https://www.youtube.com/channel/UC06RetpipAkfxl98UfEc21w). 
Don't forget to subcribe!
***
*Created by Nicolas Barrero May 2018*  
*Modified by Nicolas Barrero Oct 2018*  
*Modified by Nicolas Barrero Jan 2019*  
**"Centro Tecnologico de Automatizacion Industrial" CTAI  
Perception For Industrial Robots Project**

![imagen](https://bit.ly/2qVzHyL)
