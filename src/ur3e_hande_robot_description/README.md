## ur3e_hande_robot_description

This package contains 
  - descriptions files for the hand-e robotic gripper by *Robotiq*,
  - description file for integrating the gripper to ur3e robotic arm by Universal Robotics, and
  - four configuration files to be passed when initiating the ur3e robotic arm. 


**config/hande**

Files default_kinematics.yaml, physical_parameters.yaml, and visual_parameters are created both as a part of the automated process through "MoveIt Setup Assistant" and extracting the values from hand-e user manual. 
These files contain the physical properties of the hand-e gripper and pointers to the respective design files.

**config/camera_mount**
Physical_parameters.yaml file includes the physical measurements for the camera and the mount. These are derived both from physically measuring them and extracting from official Robotiq & Azure Kinect webpages. 

**config/ur3e**

Configuration files for the ur3e robot are directly copied from the humble version of the package [*Universal_Robots_ROS2_Description*](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/29e90d5095fdf4af99eba3c3eae153d7d5d769c0/config/ur3e).
These configurations are free to be adjusted. 

**meshes/hande**

These files contains designs for the hand-e robot. .dae files are directly taken from *Acutronic Robotic*'s [robotiq_modular_gripper](https://github.com/AcutronicRobotics/robotiq_modular_gripper/tree/4e708524e5dd20753f711686eb2cd1017a25a09e/robotiq_hande_gripper_description/meshes). These design files were not available in *Robotiq*'s github repositories.

**urdf/hande.xacro**

The previous version of the description file for the hand-e gripper base and fingers were included from *Acutronic Robotic*'s [robotiq_modular_gripper](https://github.com/AcutronicRobotics/robotiq_modular_gripper/blob/4e708524e5dd20753f711686eb2cd1017a25a09e/robotiq_hande_gripper_description/urdf/robotiq_hande.urdf.xacro) and extended to include paramters from config/hande. However, the design of the hande-base includes 4 screws that collides with the coupler. Hence, the new design for the base and the coupler are taken from [this github repo](https://github.com/macmacal/robotiq_hande_description).

**urdf/ur_with_hande.xacro**

This file creates the description for the ur3e robot and attaches the hande gripper. Ur3e robot module is created to be directly compatible with the humble version of the the package [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/29e90d5095fdf4af99eba3c3eae153d7d5d769c0/urdf/ur.urdf.xacro) by the following line 
```xml
<xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
```

**urdf/ur_with_camera_hande.xacro**
This file creates the description for the ur3e robot and attaches the hande gripper, camera mount, and the Azure Kienct camera.

**Azure Kinect .stl**
Design for the Azure Kinect camera can be found in [Microsoft official github repo.](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/5f79890933e1c81e325633152b2f2799df825b8b/assets)

## Execution

Start up the ur simulator program (IP to be connected via a VNC viewer: 192.168.56.101):
```bash
docker compose up ursim
```

Build and start the ur_driver program:
```bash
docker compose up urdriver 
```

Build the ur3e_hande_robot_description and ur3e_hande_moveit_config packages by running (IP to be connected via a VNC viewer: 192.168.56.103:5901):
```bash
docker build -t ur-moveit:latest .
docker compose up urmoveit
```


