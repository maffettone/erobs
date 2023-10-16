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
A sensible workflow would be:
```bash
cd docker/ursim
docker build -t ursim:latest .
cd ../ur-driver
docker build -t urdriver:latest .
cd ../ur-moveit
docker build -t urmoveit:latest .
cd ../
docker compose up -d ursim
```
Then open VNC client at `localhost:5900` and start the robot. 

```docker compose up -d urdriver```

In the ursim start the reverse control program with play. 

```docker compose up -d urmoveit```

Optionally start another VNC client at `localhost:5901` for RViz control.




## Notes on VSCode Workspace
VSCode ROS2 Workspace Template Borrowed from @althack. 

This template will get you set up using ROS2 with VSCode as your IDE. And help ensure consistent development across the project.

See [how she develops with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace. 

ROS2-approved formatters are included in the IDE.  

* **c++** uncrustify; config from `ament_uncrustify`
* **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

## Notes on testing pdf_beamtime_server
[Link to pdf_beamtime README](./src/pdf_beamtime/README.md)