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

Hint: Use the robot arm to determine the x and y axis of the robot by moving the robot along ```x=0``` and ```y=0``` Cartesian lines. Next use a tape measure to measure the distance (projections) from each x and y axises to the camera lens.

## ArUco markers

Complete guide to ArUco markers are in [this link](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

## Tag Family

In our work, we use the marker family 'tag36h11', which is a family of tags which are detectable by both ArUco and AprilTag detection. [This link](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11) points to the AprilTag GitHub repository that hosts pre-generated images.

Prior to detection, printed out tags (of equal size) need to be measured precisely and recorded under the paramater `physical_marker_size`.

## Tag detection
When there are multiple tags present, function `estimatePoseSingleMarkers()` estimates pose of all the detected markers seperatley. At present, we only estimate the pose when only one marker is present, ignoring the cases where no-marker or more than one marker are present.

## Detection rate
Detection rate is tied to the callback of the ros topic `image_topic` (topic is defined in the fiducial_marker_param.yam file).

In the current implementation, this topic's publish rate is defined in the Azure_Kinect_ROS_Driver package. In Azure_Kinect_ROS_Driver package, where the topic 'rgb/image_raw' is published, message publish rate is same as the camera's fps rate. Default fps is 5.

## Median estimation
Median pose estimation is a multi-value estimation, where it estimates the median of the 6 DoF pose returned by the camera. 

For each DoF, it maintains a moving window of size 10 (can be changed via the parameter `number_of_observations` ) to calculate the median. When the detection rate is 5Hz and upon moving the sample or the detected object to a new pose, it takes a minimum of 1.2 seconds to return the correct pose for the updated pose. As a good practice, it is better to wait for the whole 2 seconds. 

# Connection to the Redis DB

Redis is used to store the aruco tag ID and real-world sample information. At present, each entry has four properties. They are:
- id: integer tag ID of the printed tag from AruCo marker library
  - e.g., 0
- family: string key of AruCo tag type used 
  - e.g., 'DICT_APRILTAG_36h11', 'DICT_6X6_250'
- size  -> physical size of the marker in meters
  - e.g., 0.02665
- sample_names  -> unique string name to identify the sample used with the tag
  - e.g., 'sample_1', 'guid:dkfb6lsm228mjd'

Create a named volume `redis_data` if it doesn't exist with the following command. 
```bash
podman volume create redis_data
```

Run the Redis container with the following command:
```bash 
podman run --name redis-container --network host -d -v redis_data:/data -p 6379:6379 redis
```

## Insert new entires

Run the following if the command line interface is needed:

```bash
podman exec -it redis-container redis-cli
```

New records can be inserted into the redis server with the following command. Note the below example assumes that each entry has four attributes: id, family, size, and sample_name.

```bash
HSET tag:1 id 0 family "DICT_APRILTAG_36h11" size 0.02665 sample_name sample_1
```
Below is a python code to insert new entry.
```python
import redis
client = redis.Redis(host=IP_OF_THE_REDIS_SERVER, port=6379, db=0)
client.hset("tag:1", mapping={"id": 0, "family": "DICT_APRILTAG_36h11", "size": 0.02665, "sample_name": "sample_1"})
```
