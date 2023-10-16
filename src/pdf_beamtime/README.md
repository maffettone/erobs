## Functionality

This package has two nodes. 

The first node is pdf_beam_env inside pdf_beam_env.cpp. This nodes invokes two service calls to create a new obstacle and change the parameters of an existing obstacle. 

The second node is pdf_beamtime_server inside pdf_beamtime_server.cpp. This nodes,
 - Performs motion planning for the robot,
 - Reads the parameter server and creates the obstacles,
 - Implements servers for creating new obstacles and updating existing. 

## Service to update an obstacle
name: obstacle name
property : intended property of the obstacle
value: new value for the property
```bash
ros2 service call /pdf_update_obstacles pdf_beamtime_interfaces/srv/UpdateObstacleMsg '{name: "inbeam_platform", property: "z", value: 1.5}'
```

## Service to add a new obstacle 
name: new obstacle name
type: BOX or CYLINDER
x, y, z: location for the obstacle
w, h, d, r: width, height, depth, and radius 
```bash
ros2 service call /pdf_new_obstacle pdf_beamtime_interfaces/srv/NewObstacleMsg '{request: '', name: "obstacle", type: "BOX", x: 1.5, y: 0.2, z: 0.9, w: 0.3, h: 0.3, d: 0.3, r: 0.0}'
```