**Camera Parameters**
In OpenCV, the distortion coefficients are usually represented as a 1x8 matrix:
distCoeffs_= [k1, k2, p1, p2, k3, k4, k5, k6] where
k1, k2, k3, k3, k4, k5, k6 = radial distortion coefficients
p1, p2 = tangential distortion coefficients
For Azure kinect, they can be found when running the ROS2 camera node and explained in the following:
https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html