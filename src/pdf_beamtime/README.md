## Functionality

Node pdf_beamtime_server is implemented in pdf_beamtime_server.cpp. This nodes,
 - Performs motion planning for the robot through a Finite State Machine (FSM)
 - Reads the parameter server and creates the obstacles,
 - Implements servers for creating new obstacles, deleting and updating existing ones. 

Services can be called by following the examples below:

## Service to update an obstacle
name: obstacle name
array property : intended properties of the obstacle
array value: new values for the properties
```bash
ros2 service call /pdf_update_obstacles pdf_beamtime_interfaces/srv/UpdateObstacleMsg '{name: "inbeam_platform", property: ["z", "x"], value: [1.35, 1.0]}'
```

## Service to add a new obstacle 
name: new obstacle name
type: BOX
x, y, z: location for the obstacle
w, h, d, r: width, height, depth, and radius 
```bash
ros2 service call /pdf_new_box_obstacle pdf_beamtime_interfaces/srv/BoxObstacleMsg '{name: "obstacle", type: "BOX", x: 1.5, y: 0.2, z: 0.9, w: 0.3, h: 0.3, d: 0.3}'
```

type: CYLINDER
x, y, z: location for the obstacle
w, h, d, r: width, height, depth, and radius 
```bash
ros2 service call /pdf_new_cylinder_obstacle pdf_beamtime_interfaces/srv/CylinderObstacleMsg '{name: "obstacle2", type: "CYLINDER", x: 1.5, y: 0.2, z: 1.9, h: 0.3, r: 0.10}'
```

## Service to remove an obstacle 
name: obstacle name
```bash
ros2 service call /pdf_remove_obstacle pdf_beamtime_interfaces/srv/DeleteObstacleMsg "{name: 'inbeam_platform'}"
```

**Testing of the FSM**

FSM is tested for its four main functionality:
1. Execute a full cycle of state transitions beginning from State:Home to State:Place_Retreat.
 - Make sure you have the containers ursim, urdriver, and urmoveit running
 - Launch pdf_beamtime node by running:
  `ros2 launch pdf_beamtime pdf_beamtime.launch.py &`
 - Execute the action client by uncommenting function send_goal() inside the main method in file /src/pdf_beamtime_client.py:
  `python3 src/pdf_beamtime/src/pdf_beamtime_client.py `
 -  Expected results: Robot performs pick up, drop off, and returns to the home position (Upright). The terminal should print "Set current state to HOME" 
 
2. Abort the FSM if move_group_interface fails to plan.
 - Make sure you have the containers ursim, urdriver, and urmoveit running
 - Launch pdf_beamtime node by running:
  `ros2 launch pdf_beamtime pdf_beamtime.launch.py &`
 - Execute the action client by uncommenting the function send_incompatible_goal inside the main method in file /src/pdf_beamtime_client.py:
  `python3 src/pdf_beamtime/src/pdf_beamtime_client.py `
 - Expected results: Robot performs pick up, but robot planning fails at the state "State:Place_Approach" . No further state transitions happen and the terminal prints "Goal aborted"
 
2. Abort the FSM if the gall is canceled by the action client.
 - Make sure you have the containers ursim, urdriver, and urmoveit running
 - Launch pdf_beamtime node by running:
  `ros2 launch pdf_beamtime pdf_beamtime.launch.py &`
 - Execute the action client by uncommenting the function send_self_cancelling_goal inside the main method in file /src/pdf_beamtime_client.py:
  `python3 src/pdf_beamtime/src/pdf_beamtime_client.py `
 - Expected results: In 15 seconds after the goal is accepted by the action server, the action client sends a goal cancellation command.  The terminal prints "Received request to cancel goal", and "State machine was RESET". No further state transitions happen, the robot is moved to the State:Home state. Upon a successful transition to the state:Home, the terminal prints "Goal Cancelled !"

4. Print the completion percentage:
- Upon running any of the functionalities in 1,2 or 3, node 'pdf_beamtime_client' should print the completion percentage after each state transition on the terminal. For 1, it should print up to 100%, and for 2, it should print only up to 55%

## Testing via Bluesky Run Engine

 - Make sure you have the containers ursim, urdriver, and urmoveit running
 - Launch pdf_beamtime node by running:
  `ros2 launch pdf_beamtime pdf_beamtime.launch.py &`
 - Activate the python virtual environment 
  `source /venv/bin/activate`
 - Run the pdf_beamtime demo example in ipython terminal to execute a full cycle of state transitions beginning from State:Home to State:Place_Retreat.
  `ipython`
  `run src/bluesky_ros/pdf_beamtime_demo.py`
 - Expected results: Robot performs pick up, drop off, and returns to the home position (Upright). The terminal should prints the completion percentage, and when the execution completes it prints "Set current state to HOME" 
 
