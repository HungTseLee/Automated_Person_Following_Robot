# Automated Person-following Vehicle  

## Requirement
The following instructions are written for ROS Melodic on Ubuntu 18.04  

## Description  
**1. Subscribed Topics**  
* /human_pose (geometry_msgs/Pose)  --> the data of person position from RGB-D camera
* /laser_track_point (geometry_msgs/PointStamped)  --> the data of person position from 2-D Lidar

**2. Published Topics**  
* /cmd_vel (geometry_msgs/Twist)  --> the output command used to control the robot to follow the front person

**To start the node in ROS:**  
`$ roslaunch path_planning planning.launch`  
This command will open two RVIZ visualization windows.  
One is the perspective fixed on robot frame (/my_frame)  
The second one is used to simulate the control signal by virtual mobile robot (rbx1 package) --> [https://github.com/pirobot/rbx1](https://github.com/pirobot/rbx1)  

**Simply, there are just four .cpp files in the launch file**  
* `planning1.cpp`  --> main function & listen lidar data & do sensor fusion & publish the controller signal (/cmd_vel)
* `tf_broadcaster.cpp`  --> static tf brocaster
* `camera_listener.cpp`  --> listen the data from camera and do the calibration. Then transform the camera data from camera frame to robot frame (/my_frame)  
* `multi_controller.cpp`  --> define the several mobile robot controller for auto person following. `multi_controller.h` should be included  

## Sensor fusion
Do the late fusion in competitive fusion area. Late fusion is about fusing the results after independent detections.  
Then, by Central Limit Theorem, the estimated position of person can then be determined.  
The corresponding funsion is as follows:  
`geometry_msgs::Point sensors_fusion(geometry_msgs::Point p_lidar, Eigen::VectorXd var_lidar, geometry_msgs::Point p_camera, Eigen::VectorXd var_camera);`  
before calling the function, the covariance of the sensor should be given.  
<img src="/src/photo/fusion.png" alt="test image size" height="30%" width="30%">
<img src="/src/photo/fusion2.png" alt="test image size" height="30%" width="30%"><br>


### Coordinate relationship

The following fugire explain the relationship between sensor frames.  
<img src="/src/photo/lidar_frame.JPG" alt="test image size" height="25%" width="25%">
<img src="/src/photo/RGBD_camera_frame.JPG" alt="test image size" height="30%" width="30%">
<img src="/src/photo/frame_relation.JPG" alt="test image size" height="35%" width="35%"><br>


## Local path planning 
We use _**polynomial curve fitting**_ method to do the local pathplanning between robot and person.  
The related functions are as follows:  
```
double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
Eigen::VectorXd path_planning(geometry_msgs::Point estimated_p);
```
<img src="/src/photo/planning.png" alt="test image size" height="20%" width="20%"><br>


## Sensor failure  
If there is one sensor cannot detet the right position of person, the robot will follow the following flowchar to get a best strategy.  
<img src="/src/photo/flowchart.png" alt="test image size" height="50%" width="50%"><br>


## Controller design  
The final part is generating the control signal. The input of the controller is the `distance error` and `angle error` between robot and person.
Here, we develop several kinds of controller for the user, which are:  
* P controller
* PD controller
* Feedback linearization controller  --> consider the person's velocity
* Sigmoid function controller
* Double Sigmoid function controller
* Fuzzy controller
* MPC controller

The corresponding functions are as follows. Just uncomment the controller would like to bo be used.
```
controller my_controller(Max_V, Max_omega, safety_R);  // create a controller 
geometry_msgs::Twist velocity_input;  // encapsualize the system input 
velocity_input = my_controller.Simple_P_controller(R_err, theta_err);
velocity_input = my_controller.Simple_PD_controller(R_err, theta_err, R_err_pre, theta_err_pre);
velocity_input = my_controller.Nonlinear_controller_Feedback_linearization(estimated_position.x, estimated_position.y, people_relative_linear_velocity, people_relative_shift_velocity, robot_linear_velocity);
velocity_input = my_controller.Sigmoid_function_controller(R_err, theta_err);
velocity_input = my_controller.Improved_double_Sigmoid_controller(R_err, theta_err);
velocity_input = my_controller.Fuzzy_controller(R_err, theta_err);
velocity_input = my_controller.MPC_controller(R_err, theta_err);
```  
If you need to use fuzzy controller in this project, it is necessary to download the `FuzzyLite` C++ libraries [FuzzyLite Libraries](FuzzyLite Libraries).  
There are some existing visualized fuzzy logic design toolboxs, like Matlab. We can use it to generate a .fis file in advance.  
Then, include headfile `"fl/Headers.h"` in `multi_controller.cpp`, and uncomment the corresponding function `geometry_msgs::Twist Fuzzy_controller(double Re, double Te);`.  

## Demo  
<img src="/src/photo/Screenshot from 2020-09-02 02-19-47.png" alt="test image size" height="80%" width="80%"><br>


