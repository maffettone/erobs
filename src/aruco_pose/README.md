# Detecting Aruco Markers with Azure Kinect

## Camera Operation:

Run the following command to launch the camera.

```bash
podman run -it --rm --privileged --network host --ipc=host --pid=host azure-kinect:latest /bin/bash -c "Xvfb :2 -screen 0 2560x1440x16 & . /opt/ros/humble/setup.bash && . /root/ws/install/setup.sh && ros2 launch azure_kinect_ros_driver driver.launch.py depth_mode:=NFOV_UNBINNED  point_cloud_in_depth_frame:=false" 
```

## Camera Parameters

In OpenCV, the distortion coefficients are usually represented as a 1x8 matrix:

- distCoeffs_= [k1, k2, p1, p2, k3, k4, k5, k6] where
- k1, k2, k3, k3, k4, k5, k6 = radial distortion coefficients
- p1, p2 = tangential distortion coefficients

For Azure kinect, they can be found when running the ROS2 camera node and explained in [this link](https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html).

## Positioning of the Camera

Parameters ```cam_translation.x```, ```cam_translation.y```, and ```cam_translation.z``` are measures from the base of the robot and in the robot's coordinate world. 

Hint: Use the robot arm to determin the x and y axis of the robot by moving the robot along ```x=0``` and ```y=0``` Cartesian lines. Next use a tape measure to measure the distance from each x and y axises to the camera lense.

## ArUco markers

Complete guide to ArUco markers are in [this link](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

## Tag Family

In our work, we use the marker family 'tag36h11', which is a family of tags which are detectable by both ArUco and AprilTag detection. [This link](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11) points to the AprilTag GitHub repository that hosts pre-generated images.

Prior to detection, printed out tags (of equal size) needs to be measured precisley and recorded under the paramater `physical_marker_size`.

## Tag detection
When there are multiple tags present, function `estimatePoseSingleMarkers()` estimates pose of all the detected markers seperatley. At present, we only estimate the pose when only one marker is present, ignoring the cases where no-marker or more than one marker are present.

## Detection rate
Detection rate is tied to the callback of the ros topic `image_topic` (topic is defined in the fiducial_marker_param.yam file).

In the current implementation, this topic's publish rate is defined in the Azure_Kinect_ROS_Driver package. In Azure_Kinect_ROS_Driver package, where the topic 'rgb/image_raw' is published, message publish rate is same as the camera's fps rate. Default fps is 5.

## Median estimation
Median pose estimation is a multi-value estimation, where it estimates the median of the 6 DoF pose returned by the camera. 

For each DoF, it maintains a moving window of size 10 (can be changed via the parameter `number_of_observations` ) to calculate the median. When the detection rate is 5Hz and upon moving the sample or the detected object to a new pose, it takes a minimum of 1.2 seconds to return the correct pose for the updated pose. As a good practice, it is better to wait for the whole 2 seconds. 