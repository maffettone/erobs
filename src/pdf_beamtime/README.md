## Functionality

Node pdf_beamtime_server is implemented in pdf_beamtime_server.cpp. This nodes,
 - Performs motion planning for the robot,
 - Reads the parameter server and creates the obstacles,
 - Implements servers for creating new obstacles, deleting and updating existing ones. 

Servers can be called by following the examples below:

## Service to update an obstacle
name: obstacle name
array property : intended properties of the obstacle
array value: new values for the properties
```bash
ros2 service call /pdf_update_obstacles pdf_beamtime_interfaces/srv/UpdateObstacleMsg '{name: "inbeam_platform", property: ["z", "x"], value: [1.35, 1.0]}'
```

## Service to add a new obstacle 
name: new obstacle name
type: BOX or CYLINDER
x, y, z: location for the obstacle
w, h, d, r: width, height, depth, and radius 
```bash
ros2 service call /pdf_new_obstacle pdf_beamtime_interfaces/srv/NewObstacleMsg '{name: "obstacle", type: "BOX", x: 1.5, y: 0.2, z: 0.9, w: 0.3, h: 0.3, d: 0.3, r: 0.0}'
```

## Service to remove an obstacle 
name: obstacle name
```bash
ros2 service call /pdf_remove_obstacle pdf_beamtime_interfaces/srv/DeleteObstacleMsg "{name: 'inbeam_platform'}"
```
