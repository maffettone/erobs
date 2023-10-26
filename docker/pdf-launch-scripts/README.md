# File Contents:
Docker launch scripts for ROS2 at the PDF beamline. Detailed documentation on building and deploying containers at NSLS-II
can be found in our [documentation](https://docs.nsls2.bnl.gov/docs/how-to/containers.html).

Containers should be stored in an appropriately scoped content registry and pulled into the beamline machines. For example:

```bash
cd $DOCKER_FILE_DIRECTORY
podman build -t erobs-ur-driver:latest . # Build for local testing
podman tag erobs-ur-driver:latest ghcr.io/nsls2/erobs-ur-driver:latest # Retagg for content registry
podman push ghcr.io/nsls2/erobs-ur-driver:latest # Push up
```

## Testing:
- `hello-talker.sh`: publisher node for testing
- `simple_listener.py`: Subscriber node for testing
- `ur-driver-launch.sh`: Pure UR driver for testing without gripper

## Deployment:
- `bsui-launch.sh`: Launch bsui with ROS2 functionality on an epics networked machine

### Work in Progress *Depends on ur-hande-draft container to be deprecated.*
- `ur-hande-driver-launch.sh`: Driver for UR+Hand-E launch.
  - `robotitq-driver-entrypoint.sh`: Entrypoint for the gripper. Starts tool communication UR side, then starts a (TODO: poorly named) Robotiq diver.
- `mtc-moveit-launch.sh`: Moveit movegroup launch.
- `robotiq-driver-launch.sh`: Launch file for the gripper
- `sample-movement-server.launch`: Launch custom ROS Action Server
