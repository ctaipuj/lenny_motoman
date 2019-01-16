![imagen](https://raw.githubusercontent.com/ctaipuj/lenny_motoman/master/lenny.png)
# motoman\_sda10f\_support Package
## Overview
This package  is forked from the original [motoman package](https://github.com/ros-industrial/motoman) kinetic devel. It contains support elements for the Lenny's environment configuration. Meshes, textures, and descriptions are stored here.

## Contents

1. Config  
Configuration files (YAML files).
2. launch  
Launch files. 
3. meshes  
Geometries and textures of Lenny and the environment.
4.robotiq_c2_model_visualization  
Gripper's geometries and descriptive files.
5. test
6. urdf  
URDF files describing the robot and environment. 

## Usage

**CAUTION**

Before using the SDA10F robot with ROS make sure the **../motoman\_sda10f\_support/config/sda10f\_motion\_interface.yaml** yaml file ennumerates the groups from *0 to 3*. Otherwise the robot won't work properly with ROS.

```yaml
topic_list:
    - name: sda10f_r1_controller
      ns: sda10f
      group: 0
      joints: ['arm_left_joint_1_s','arm_left_joint_2_l','arm_left_joint_3_e','arm_left_joint_4_u','arm_left_joint_5_r','arm_left_joint_6_b','arm_left_joint_7_t']
    - name: sda10f_r2_controller
      ns: sda10f
      group: 1
      joints: ['arm_right_joint_1_s','arm_right_joint_2_l','arm_right_joint_3_e','arm_right_joint_4_u','arm_right_joint_5_r','arm_right_joint_6_b','arm_right_joint_7_t']
    - name: sda10f_b1_controller
      ns: sda10f
      group: 2
      joints: ['torso_joint_b1']
    - name: sda10f_b2_controller
      ns: sda10f
      group: 3
      joints: ['torso_joint_b2']
```

More information about robot's setup with ROS [here](https://www.researchgate.net/publication/315063645_Setup_of_the_Yaskawa_SDA10F_Robot_for_Industrial_Applications_Using_ROS-Industrial).

The meshes are organized in four folders.

1. camera: Kinect's STL files.
2. environment: Textures and STL files for the environment.
3. robotiq_c2: Gripper's geometries.
4. sda10f: Lenny's Visual and collision geometries.

 You can edit the URDF files to add new elements to the environmnet.

## NOTE

* Be careful when editing macro.xacro files. Only edit the `sda10f_macro.xacro` and the `simple_sda10f_macro.xacro` files if possible.
*  Since Lenny works with two collision matrixes two `sda10f_macro.xacro` files were created. One for the simple view environment and the other one for the full environment. When editing these files two srdf files must be generated with the setup assistant.

***
![imagen](https://bit.ly/2QOK5D6)  
Keep up with CTAI new developments! Watch our [YouTube Channel](https://www.youtube.com/channel/UC06RetpipAkfxl98UfEc21w). 
Don't forget to subscribe!
***
*Created by Nicolas Barrero Jan 2019*    
**"Centro Tecnologico de Automatizacion Industrial" CTAI  
Perception For Industrial Robots Project**

![imagen](https://bit.ly/2qVzHyL)
