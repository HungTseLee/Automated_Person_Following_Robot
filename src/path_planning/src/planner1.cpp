#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
 
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#include "multi_controller.h"   // our own

#define PI 3.14159265

using namespace std;
using namespace Eigen;

double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
Eigen::VectorXd path_planning(geometry_msgs::Point estimated_p);
void show_lidar_people_position_text_msg(string my_text, geometry_msgs::Point p);
void show_the_robot_position();
void show_text_msg(string my_text, int id_num, double x, double y, double z);   // display the msg in robot frame
void show_estimated_position(geometry_msgs::Point estimated);
void show_moving_average_position(geometry_msgs::Point average_pos);
Eigen::Vector2d show_the_local_planning_path(Eigen::VectorXd coeff, geometry_msgs::Point estimated);
void show_the_direction_arrow(Eigen::Vector2d direction);

geometry_msgs::Point sensors_fusion(geometry_msgs::Point p_lidar, Eigen::VectorXd var_lidar, geometry_msgs::Point p_camera, Eigen::VectorXd var_camera);

bool feedback_enable = 0;  // check if the sensor start to publish data to the topic

#define Fs 10  // sampling frequency
#define Ts 1.0/Fs  // sampling period (we have to confirm that the calculation can be finished within Ts)

ros::Publisher marker_pub;

// void laserCallback(const geometry_msgs::PointStamped::ConstPtr& msg){}
// void CameraPoseCallback(const geometry_msgs::Point::ConstPtr& msg){;}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner1");

	ros::NodeHandle n;
	
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Publisher Velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);   // used to publish the velocity input to topic
	
	geometry_msgs::Point laser_pose;
	auto laserCallback = [&laser_pose](const geometry_msgs::PointStamped::ConstPtr& msg) mutable -> void {
		// show the data from Lidar(laser)
		cout << "Lidar data: " << endl;
		ROS_INFO("people position: [%.4f, %.4f, %.4f]", msg->point.x/100, msg->point.y/100, msg->point.z/100);
		///// The position of people is relative to the robot's coordinate
		laser_pose.x = 16-msg->point.x/100-8;
		laser_pose.y = msg->point.y/100-8;
		laser_pose.z = msg->point.z/100;
		ROS_INFO("After transform: [%.4f, %.4f, %.4f]", laser_pose.x, laser_pose.y, laser_pose.z);
		cout << "--------------------------------------------------" << endl;
		feedback_enable = 1;
	};
    auto sub = n.subscribe<geometry_msgs::PointStamped>("laser_track_point", 1000, laserCallback);
	//ros::Subscriber
	
	
	geometry_msgs::Point camera_pose;
	auto camera_sub = n.subscribe<geometry_msgs::Point>("camera_pose_calibration", 1000, [&camera_pose](const geometry_msgs::Point::ConstPtr& msg) mutable -> void {camera_pose = *msg;} );    // CameraPoseCallback
	
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	
		
	//// Controller design ////////////////////////////////
	//// use tracking error as the input of controller ////
	double Max_V = 1;   // the maxima speed of the robot is 1 m/s 
	double Max_omega = 75*PI/180;   // the maxima omega of the robot rad/s 
	double safety_R = 0.5;    // decide a safety distance between robot and people (unit:m)
	controller my_controller(Max_V, Max_omega, safety_R);  // create a controller 
	//// some parameter used in controller
	double theta_err, R_err;  // the tracking error of the estimated person position
	double theta_err_pre, R_err_pre;  // previous tracking error be usedin PD controller
	double robot_linear_velocity = 0;
	
	
	ros::Rate loop_rate(Fs);
	
	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "I want to give a control signal " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
		
		//////////////////////////////////////////////////////////////////////////
		//// Do the moving average filter for filtering the strolling shaking ////
		[&laser_pose]() mutable -> void{
			static vector<geometry_msgs::Point> previous_point_vec = {laser_pose, laser_pose, laser_pose, laser_pose, laser_pose, laser_pose};
			previous_point_vec.erase(previous_point_vec.begin());
			previous_point_vec.push_back(laser_pose);
			double sum_x = 0, sum_y = 0;
			for (auto &it : previous_point_vec)	{ sum_x += it.x;	sum_y += it.y; }
			laser_pose.x = sum_x/previous_point_vec.size();  
			laser_pose.y = sum_y/previous_point_vec.size();
			laser_pose.z = 0;
			cout << "vector size = " << previous_point_vec.size() << endl;
			cout << "point after averaging = (" << laser_pose.x << ", " << laser_pose.y << ")" << endl; 

			//p.x = accumulate(previous_point_vec.begin(), previous_point_vec.end(), 0.0f, [](geometry_msgs::Point p1, geometry_msgs::Point p2) {return p1.x+p2.x/previous_point_vec.size();});
			//p.y = accumulate(previous_point_vec.begin(), previous_point_vec.end(), 0.0f, [](geometry_msgs::Point p1, geometry_msgs::Point p2) {return p1.y+p2.y/previous_point_vec.size();});
			//cout << "point after averaging = (" << p.x << ", " << p.y << ")" << endl; 
		}();
		
		//// Mark the Robot/////////////////////////////////////////
		show_the_robot_position();  // show the robot in RVIZ
		
		// show the moving average people position in RVIZ
		show_moving_average_position(laser_pose);

		// show the lidar text in RVIZ
		laser_pose.z += 1.0;
		show_lidar_people_position_text_msg("pos from Lidar", laser_pose);

		// calculate the position error
		double pos_err = [](geometry_msgs::Point p1, geometry_msgs::Point p2) -> double {
			double pos_err_x = p1.x-p2.x;
			double pos_err_y = p1.y-p2.y;
			return sqrt(pow(pos_err_x,2)+pow(pos_err_y,2));
		}(camera_pose, laser_pose);
		cout << "The position error = " << pos_err << endl;
		// show the error in RVIZ
		show_text_msg("Two Frame Pos Error: "+to_string(pos_err)+" m", 50, 1, -0.2, 0);

		
		/////////////////////////////////////////////////////////////
		//// Do the sensor fusion of camera and lidar ///////////////
		geometry_msgs::Point estimated_position;
		Eigen::VectorXd var_lidar(2), var_camera(2);
		var_lidar << 0.0001, 0.0001;  // assign the covariance of lidar
		var_camera << 0.0004, 0.0004;  // assign the covarianceod camera
		double sensor_err_threshold = 0.3;   // decide whether the lidar still work
		if (pos_err < sensor_err_threshold)   // If the error between lidar and camera is within 0.3 meter, it means that the lidar still work.
			estimated_position = sensors_fusion(laser_pose, var_lidar, camera_pose, var_camera);  // call sensor fusion function
		else
			estimated_position = camera_pose;   // lidar fail, we believe the position data provided by camera
		cout << "Estimated position = (" << estimated_position.x << ", " << estimated_position.y << ")" << endl;
		show_estimated_position(estimated_position);   // show the estimated position in RVIZ
		
		
		/////////////////////////////////////////////////////////////////
		////// Do the 2nd order curve fitting for the path planning /////
		auto coeffs = path_planning(estimated_position);
		//// show the path in RVIZ /////////////////////////
		Eigen::Vector2d direction_points = show_the_local_planning_path(coeffs, estimated_position);
		//// Adding the direction arrow to point out the data ////////////
		show_the_direction_arrow(direction_points);  // show direction in RVIZ
		
		
		////////////////////////////////////////////////////
		//// Compute the angle error of the robot //////////
		double theta = atan2(direction_points(1),direction_points(0))-PI/2;   // unit: rad
		theta_err = theta - 0;
		double x_err = estimated_position.x;
		double y_err = estimated_position.y;     
		double R = pow(x_err*x_err+y_err*y_err,0.5);
		R_err = R-safety_R;
		cout << "(R, theta) = (" << R << ", " << theta << ")" << endl;
		cout << "Tracking Error (Err_R, Err_theta) = (" << R_err << ", " << theta_err << ")" << endl;
		show_text_msg("(R_err,theta_err)=("+to_string(R_err)+","+to_string(theta_err)+")", 57, 1, -0.5, 0);  // show the tracking error in RVIZ
		
		
		//// calaulate the relative velocity between person and robot
		static double pre_people_x = estimated_position.x;    //previous person position
		static double pre_people_y = estimated_position.y;  
		double people_linear_diff = estimated_position.y-pre_people_y;
		double people_shift_diff = estimated_position.x-pre_people_x;
		double people_relative_linear_velocity, people_relative_shift_velocity;
		people_relative_linear_velocity = people_linear_diff*Fs;
		people_relative_shift_velocity = people_shift_diff*Fs;
		show_text_msg("people relative V = "+to_string(people_relative_linear_velocity), 56, 1, -1.1, 0);
		
		
		//// Choose the controller to be used
		geometry_msgs::Twist velocity_input;  // encapsualize the system input 
		//velocity_input = my_controller.Simple_P_controller(R_err, theta_err);
		velocity_input = my_controller.Simple_PD_controller(R_err, theta_err, R_err_pre, theta_err_pre);
		//velocity_input = my_controller.Improved_P_controller(R_err, theta_err, people_relative_linear_velocity, robot_linear_velocity);
		//velocity_input = my_controller.Nonlinear_controller_Feedback_linearization(estimated_position.x, estimated_position.y, people_relative_linear_velocity, people_relative_shift_velocity, robot_linear_velocity);
		//velocity_input = my_controller.Sigmoid_function_controller(R_err, theta_err);
		//velocity_input = my_controller.Improved_double_Sigmoid_controller(R_err, theta_err);
		//velocity_input = my_controller.Fuzzy_controller(R_err, theta_err);
		//velocity_input = my_controller.MPC_controller(R_err, theta_err);
		
		cout << "Control input (V, omega) = (" << velocity_input.linear.x << ", " << velocity_input.angular.z << ")" << endl;
		// show the control input in the RVIZ
		show_text_msg("Controller(V,omega) = ("+to_string(velocity_input.linear.x)+","+to_string(velocity_input.angular.z)+")", 51, 1, -0.8, 0);

		if (feedback_enable == 0)
		{
			velocity_input.linear.x = 0;
			velocity_input.angular.z = 0;
		}
		Velocity_pub.publish(velocity_input);
		robot_linear_velocity = velocity_input.linear.x;
		
		R_err_pre = R_err;   // save as the previous parameters
		theta_err_pre = theta_err;
		pre_people_x = estimated_position.x;
		pre_people_y = estimated_position.y;
		cout << "--------------------------------------------------" << endl;
		ros::spinOnce();
 		loop_rate.sleep();
		++count;
	}

	return 0;
}


double polyeval(Eigen::VectorXd coeffs, double x) 
{
  	double result = 0.0;
  	for (int i = 0; i < coeffs.size(); i++) {result += coeffs[i]*pow(x, i);}
  	return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
  	assert(xvals.size() == yvals.size());
  	assert(order >= 1 && order <= xvals.size() - 1);
  	Eigen::MatrixXd A(xvals.size(), order + 1);
  	for (int i = 0; i < xvals.size(); i++) 
	{
    	A(i, 0) = 1.0;
  	}
  	for (int j = 0; j < xvals.size(); j++) 
	{
    	for (int i = 0; i < order; i++) 
		{
      		A(j, i + 1) = A(j, i) * xvals(j);
    	}
  	}
  	auto Q = A.householderQr();
  	auto result = Q.solve(yvals);
  	return result;
}

Eigen::VectorXd path_planning(geometry_msgs::Point estimated_p)
{
	double robot_x = 0;
	double robot_y = 0;  // the robot frame origin

	double people_x = estimated_p.x;   // lidar after averaging (p.x, p.y)   // camera (camera_pose.x, camera_pose.y)
	double people_y = estimated_p.y;
	
	//anchor point
	double anchorF_x = 0+(people_x)*2/15;
	double anchorF_y = 0.2;
	
	double anchorN_x = 0+(people_x)/15;
	double anchorN_y = 0.1;
	
	Eigen::VectorXd xvals(4);
	Eigen::VectorXd yvals(4);
	xvals << robot_x, anchorN_x, anchorF_x, people_x;
	yvals << robot_y, anchorN_y, anchorF_y, people_y;
	
	return polyfit(yvals,xvals,2);
}

void show_lidar_people_position_text_msg(string my_text, geometry_msgs::Point p)
{
	visualization_msgs::Marker label_text;
	label_text.header.frame_id = "/my_frame";
    label_text.header.stamp = ros::Time::now();
    label_text.ns = "planner1";
    label_text.action = visualization_msgs::Marker::ADD;
    label_text.pose.orientation.w = 1.0;	
    label_text.id = 10;  // %Tag(ID)%
    label_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING; // %Tag(TYPE)%
    label_text.scale.z = 0.15;  //
	// %Tag(COLOR)%
    label_text.color.r = 1.0;
	label_text.color.g = 1.0;
	label_text.color.b = 1.0;
    label_text.color.a = 1.0;
	geometry_msgs::Pose pose;
	pose.position.x = p.x;
	pose.position.y = p.y;
	pose.position.z = p.z;

	label_text.text = my_text;
	label_text.pose = pose;

	marker_pub.publish(label_text);
	return;
}

void show_text_msg(string my_text, int id_num, double x, double y, double z)
{
	visualization_msgs::Marker error_text;
	error_text.header.frame_id = "/my_frame";
    error_text.header.stamp = ros::Time::now();
    error_text.ns = "planner1";
    error_text.action = visualization_msgs::Marker::ADD;
    error_text.pose.orientation.w = 1.0;	
    error_text.id = id_num;  // %Tag(ID)%
    error_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING; // %Tag(TYPE)%
    error_text.scale.z = 0.15;  //
	// %Tag(COLOR)%
    error_text.color.r = 1.0;
	error_text.color.g = 1.0;
	error_text.color.b = 1.0;
    error_text.color.a = 1.0;
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;

	error_text.text = my_text;
	error_text.pose = pose;

	marker_pub.publish(error_text);
	
	return;
}

geometry_msgs::Point sensors_fusion(geometry_msgs::Point p_lidar, Eigen::VectorXd var_lidar, geometry_msgs::Point p_camera, Eigen::VectorXd var_camera)
{
	geometry_msgs::Point estimated_point;
	estimated_point.x = (var_camera[0]*p_lidar.x+var_lidar[0]*p_camera.x)/(var_camera[0]+var_lidar[0]);
	estimated_point.y = (var_camera[1]*p_lidar.y+var_lidar[1]*p_camera.y)/(var_camera[1]+var_lidar[1]);
	estimated_point.z = 0;
	
	return estimated_point;
}

void show_estimated_position(geometry_msgs::Point estimated)
{
	visualization_msgs::Marker fianl_pos;
	fianl_pos.header.frame_id = "/my_frame";
    fianl_pos.header.stamp = ros::Time::now();
    fianl_pos.ns = "planner1";
    fianl_pos.action = visualization_msgs::Marker::ADD;
    fianl_pos.pose.orientation.w = 1.0;	
    fianl_pos.id = 12356;  // %Tag(ID)%
    fianl_pos.type = visualization_msgs::Marker::LINE_LIST; // %Tag(TYPE)%
    fianl_pos.scale.x = 0.05;  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width	
	// %Tag(COLOR)%
    fianl_pos.color.r = 191./255;
	fianl_pos.color.g = 22./255;
	fianl_pos.color.b = 248./255;
    fianl_pos.color.a = 1.0;
	
	fianl_pos.points.push_back(estimated);
	estimated.z += 1.0;
    fianl_pos.points.push_back(estimated);
	marker_pub.publish(fianl_pos);
	
	return;
}

void show_moving_average_position(geometry_msgs::Point average_pos)
{
	// %Tag(MARKER_INIT)%
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/my_frame";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "planner1";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;	
    line_list.id = 0;  // %Tag(ID)%
    line_list.type = visualization_msgs::Marker::LINE_LIST; // %Tag(TYPE)%
    line_list.scale.x = 0.05;  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width	
	// %Tag(COLOR)%
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
	
	line_list.points.push_back(average_pos);
	average_pos.z += 1.0;
    line_list.points.push_back(average_pos);
	marker_pub.publish(line_list);
	
	return;
}

void show_the_robot_position()
{
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
	
	marker.ns = "planner1";  // %Tag(NS_ID)%
	marker.id = 1;
	
	marker.type = visualization_msgs::Marker::CUBE; // %Tag(TYPE)%   CYLINDER
	
	marker.action = visualization_msgs::Marker::ADD; // %Tag(ACTION)%
	
	marker.pose.position.x = 0;  // %Tag(POSE)%
	marker.pose.position.y = 0-0.1;
	marker.pose.position.z = 0+0.1;
	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( PI/2, 0, 0 );
	marker.pose.orientation = tf2::toMsg(myQuaternion);

	marker.scale.x = 0.2;  // %Tag(SCALE)%
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	marker.color.r = 255./255; // %Tag(COLOR)%
	marker.color.g = 106./255;
	marker.color.b = 0./255;
	marker.color.a = 0.8;

	marker.lifetime = ros::Duration();  // %Tag(LIFETIME)%

	marker_pub.publish(marker);  // Publish the marker

	return;
}

Eigen::Vector2d show_the_local_planning_path(Eigen::VectorXd coeff, geometry_msgs::Point estimated)
{
	visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/my_frame";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "planner1";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w  = 1.0;
	
	line_strip.id = 2;
	
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
	line_strip.scale.x = 0.025;
	// Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
	
	geometry_msgs::Point p2;
	for (int i = 0; i<=(int)(estimated.y/0.1); i++)
	{	
      	p2.x = polyeval(coeff,0.1*i);
      	p2.y = 0.1*i;
      	p2.z = 0;
		line_strip.points.push_back(p2);
	}
	p2.x = polyeval(coeff,estimated.y);
    p2.y = estimated.y;
	line_strip.points.push_back(p2);

	marker_pub.publish(line_strip);
	
	Eigen::Vector2d direction(line_strip.points[2].x, line_strip.points[2].y);
	
	return direction;
}

void show_the_direction_arrow(Eigen::Vector2d direction)
{
	visualization_msgs::Marker arrow;
	arrow.header.frame_id = "/my_frame";
	arrow.header.stamp = ros::Time::now();
	arrow.ns = "planner1";
	arrow.action = visualization_msgs::Marker::ADD;
	arrow.pose.orientation.w = 1.0;
	arrow.id = 3;
	//arrow.type = visualization_msgs::Marker::ARROW;
    arrow.type = visualization_msgs::Marker::LINE_LIST;

    arrow.scale.x = 0.02;
    arrow.color.g = 1.0;
    arrow.color.a = 1.0;

	geometry_msgs::Point p_orientation;
    p_orientation.x = 0;
    p_orientation.y = 0;
    p_orientation.z = 0;
	arrow.points.push_back(p_orientation);
	p_orientation.x = 0+(direction(0))*3;
    p_orientation.y = 0+(direction(1))*3;
    arrow.points.push_back(p_orientation);

	marker_pub.publish(arrow);
	
	return;
}

