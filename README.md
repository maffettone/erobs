# Extensible Robotic Beamline Scientist
Project repository for building extensible robotic beamline scientists at NSLS-II. 

## Contents 

### Hello Moveit 
Demonstrations using a combination of the MoveIt tutorials and some UR specific tools, to show how to make simple actions
that can deploy MoveIt using the MoveGroupInterface. 

## Bluesky ROS
Ongoing developments of integrating ROS2 and Bluesky. Currently targeted towards integrating Ophyd Objects as ROS2 Action Clients. 

### Containers
- ursim: Will run a simulated robot and teach pendant to be accessed by VNC. Has preloaded programs and IP configuration for reverse control by ROS2.
- ur-driver: Runs the ROS2 driver for controlling the robot in simulator
- ur-moveit: Runs MoveIt configured to the UR and optionally RViz to be accessed by VNC
- ur-example: Generally unused example movement for testing. 

## Using Docker

### Running the ur-example
In order to run the `ur-example` with Docker, follow this procedure:

1. Create the required images.
```bash
cd docker
docker build -t ursim:latest ./ursim
docker build -t ur-driver:latest ./ur-driver
docker build -t ur-example:latest ./ur-example
```
2. Start the UR Simulator. In a new terminal, run
```bash
docker compose up ursim
```
Open VNC client at `localhost:5900`. 
Turn on and start the robot. 
Go to the `Move` tab and click the `Home` button. 
Press and hold the `Move robot to: New position` button to move the robot into position. Press `Continue`.
Verify the joint position is `[0, -90, 0, -90, 0, 0]` degrees.

Note: setting initial position is requried for the `ur-example` to start, as specified in the `test_goal_publisher_config.yaml` file in the official Unviersal_Robots_ROS2_Driver repo.

3. Start the ur-driver. In a new terminal, run
```bash
docker compose up urdriver
```
Now, go back to the VNC client. In the `Program` tab, start the program.

4. Run the ur-example. In a new terminal, run
```bash
docker compose up urexample
```
The in `Program/Graphics` tab, the robot should be moving between four poses every 6 seconds.

## Notes on VSCode Workspace
VSCode ROS2 Workspace Template Borrowed from @althack. 

This template will get you set up using ROS2 with VSCode as your IDE. And help ensure consistent development across the project.

See [how she develops with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace. 

ROS2-approved formatters are included in the IDE.  

* **c++** uncrustify; config from `ament_uncrustify`
* **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

## Notes on pdf_beamtime and its tests
pdf_beamtime is a work-in-progress package aiming to deploy the UR3e robot arm + HandE gripper at the PDF beamline. 
This package depends on pdf_beamtime_interfaces. Follow the link below for information on the package and for the commands to call the servers implemented in the package.

[Link to pdf_beamtime README](./src/pdf_beamtime/README.md)