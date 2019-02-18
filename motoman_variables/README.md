![imagen](https://raw.githubusercontent.com/ctaipuj/lenny_motoman/master/lenny.png)
# motoman\_variables Package *Developed by Nicolas Barrero 2018*
## Overview
This package  was developed at CTAI. It contains a C++ class that facilitates the communication with the FS100 controller's network variables through ROS using the `ReadSingleIO.srv` and `WriteSingleIO.srv` services

## Contents

1. include/motoman_variables
2. src
3. documentation.pdf

## Usage

In the **src** directory you can find the `lenny_variable_sample.cpp` file that is a ROS node that shows how to use the different functions created.

When using this package the real robot must be connected to the system since the `ReadSingleIO.srv`and `WriteSingleIO.srv`services are required.

First launch robot connection:

`roslaunch motoman_sda10f_moveit_config moveit_planning_execution.launch`

Then in another terminal run the sample node:

`rosrun motoman_variables lenny_variable_sample`

You can see in the teach pendant and in console the behavior of network variables.

The `motoman_variables.cpp` file contains the description of the created methods.

You can include these fucntions in your c++ program: `#include<motoman_variables/motoman_variables.h>` Previous configuration of your CMakeLists.

For further information about the usage you can see the [documentation.pdf](https://github.com/ctaipuj/lenny_motoman/blob/master/motoman_variables/documentation.pdf) file. 

## NOTE

* Controller's ladder structure must be changed first in order to activate external outputs through network variables.

***
![imagen](https://bit.ly/2QOK5D6)  
Keep up with CTAI new developments! Watch our [YouTube Channel](https://www.youtube.com/channel/UC06RetpipAkfxl98UfEc21w). 
Don't forget to subscribe!
***
*Created by Nicolas Barrero Feb 2019*    
**"Centro Tecnologico de Automatizacion Industrial" CTAI  
Perception For Industrial Robots Project**

![imagen](https://bit.ly/2qVzHyL)
