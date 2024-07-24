# Detecting Aruco Markers with Azure Kinect

## Camera Parameters

In OpenCV, the distortion coefficients are usually represented as a 1x8 matrix:

- distCoeffs_= [k1, k2, p1, p2, k3, k4, k5, k6] where
- k1, k2, k3, k3, k4, k5, k6 = radial distortion coefficients
- p1, p2 = tangential distortion coefficients

For Azure kinect, they can be found when running the ROS2 camera node and explained in [this link](https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html).

## ArUco markers

Complete guide to ArUco markers are in [this link](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

## Detection rate
Detection rate is tied to the callback of the ros topic `image_topic` (topic is defined in the fiducial_marker_param.yam file).

In the current implementation, this topic's publish rate is defined in the Azure_Kinect_ROS_Driver package. In Azure_Kinect_ROS_Driver package, where the topic 'rgb/image_raw' is published, message publish rate is same as the camera's fps rate. Default fps is 5.