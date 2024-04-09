Build the docker / podman image
```bash
podman build -t azure-kinect:latest .
```

Following command executes the image without opening up the network to the host network:
```bash
podman run -it --privileged -p 5905:5905 azure-kinect:latest /bin/bash -c "Xvfb :1 -screen 0 2560x1440x16 & . /opt/ros/humble/setup.bash && . /root/ws/install/setup.sh && ros2 launch azure_kinect_ros_driver driver.launch.py" 
```

Following command executes the image opening up a the container to the host network. This needs testing
```bash
podman run -it --privileged -p 5905:5905  --network host --ipc=host --pid=host azure-kinect:latest /bin/bash -c "Xvfb :1 -screen 0 2560x1440x16 & . /opt/ros/humble/setup.bash && . /root/ws/install/setup.sh && ros2 launch azure_kinect_ros_driver driver.launch.py" 
```