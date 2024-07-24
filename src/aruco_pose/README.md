# Detecting Aruco Markers with Azure Kinect

## Camera Parameters

In OpenCV, the distortion coefficients are usually represented as a 1x8 matrix:

- distCoeffs_= [k1, k2, p1, p2, k3, k4, k5, k6] where
- k1, k2, k3, k3, k4, k5, k6 = radial distortion coefficients
- p1, p2 = tangential distortion coefficients

For Azure kinect, they can be found when running the ROS2 camera node and explained in [this link](https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html).

## ArUco markers

Complete guide to ArUco markers are in [this link](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

## Tag detection
When there are multiple tags present, function `estimatePoseSingleMarkers()` estimates pose of all the detected markers seperatley. At present, we only estimate the pose when only one marker is present, ignoring the cases where no-marker or more than one marker are present.

## Detection rate
Detection rate is tied to the callback of the ros topic `image_topic` (topic is defined in the fiducial_marker_param.yam file).

In the current implementation, this topic's publish rate is defined in the Azure_Kinect_ROS_Driver package. In Azure_Kinect_ROS_Driver package, where the topic 'rgb/image_raw' is published, message publish rate is same as the camera's fps rate. Default fps is 5.

## Median estimation
Median pose estimation is a multi-value estimation, where it estimates the median of the 6 DoF pose returned by the camera. 

For each DoF, it maintains a moving window of size 10 (can be changed via the parameter `number_of_observations` ) to calculate the median. When the detection rate is 5Hz and upon moving the sample or the detected object to a new pose, it takes a minimum of 1.2 seconds to return the correct pose for the updated pose. As a good practice, it is better to wait for the whole 2 seconds. 