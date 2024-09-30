Build the image
```bash
podman build -t azure-kinect:latest .
```

The following command runs a container with this image without opening up the container network to the host network.
```bash
podman run -it --volume="/dev/bus/usb:/dev/bus/usb" --rm azure-kinect:latest /bin/bash -c "Xvfb :1 -screen 0 2560x1440x16 & . /opt/ros/humble/setup.bash && . /root/ws/install/setup.sh && ros2 launch azure_kinect_ros_driver driver.launch.py" 
```

The following command runs a container with this image using the host network as the container network.
```bash
podman run -it --volume="/dev/bus/usb:/dev/bus/usb" --rm --network host --ipc=host --pid=host azure-kinect:latest /bin/bash -c "Xvfb :2 -screen 0 2560x1440x16 & . /opt/ros/humble/setup.bash && . /root/ws/install/setup.sh && ros2 launch azure_kinect_ros_driver driver.launch.py depth_mode:=NFOV_UNBINNED  point_cloud_in_depth_frame:=false"
```

In the above, `depth_mode:=NFOV_UNBINNED` sets a narrow field of view for increased distance in depth perception and `point_cloud_in_depth_frame = false` to render the pointcloud onto the RBG image.

A number of parameters with respect to the Kinect camera can be set via driver.launch.py. Refer [this link](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/6ffb95a56ee175e5020b5ee5983d7230befbb176/docs/usage.md) for the complete list of options.