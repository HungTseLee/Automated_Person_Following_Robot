#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <sstream>
#include <iostream>
#include <cmath>

#define PI 3.14159265

using namespace std;
//using namespace Eigen;

void tf_transfrom_from_camera_to_base(geometry_msgs::Point camera_pos, const tf::TransformListener& Listener);
void show_camera_people_position(geometry_msgs::Point camera2base_point);
void show_camera_people_position_text_msg(string my_text, geometry_msgs::Point p);

ros::Publisher marker_pub;
ros::Publisher camera_pos_pub;

void CameraCallback(const geometry_msgs::Pose::ConstPtr& msg, const tf::TransformListener& listener)
{
	cout << "Camera data: " << endl;
	
	geometry_msgs::Point now_points;
	[msg, &now_points]() mutable -> void{
		//// transform RPY to XYZ position 
		double R = msg->position.z;
		tf2::Quaternion myQuaternion;
		tf2::convert(msg->orientation, myQuaternion);
		tf2::Matrix3x3 matrix(myQuaternion);
		double roll, pitch, yaw;
		matrix.getRPY(roll, pitch, yaw);
		double theta = -yaw;
		cout << "(R, theta) = (" << R << ", " << theta << ")" << endl;
		now_points.x =  R/tan(PI/2. + theta); //  R*cos(PI/2. + theta);
		now_points.y =  R; //  R*sin(PI/2. + theta);
		now_points.z = 0;
		ROS_INFO("people position: [%.4f, %.4f, %.4f]", now_points.x, now_points.y, now_points.z);

		//// Doing the error compensation by Lidar
		double R_gain = 1.05;  //1.3
		double R_shift = 0.27; //0
		R = R_gain*R + R_shift;
		double theta_gain = 1.0;   // 1.6
		double theta_shift = 10*PI/180;   //     -0.75    8*PI/180
		theta = theta_gain*theta + theta_shift;
		cout << "After compensation: (R, theta) = (" << R << ", " << theta << ")" << endl;
		now_points.x = R*cos(PI/2. + theta);
		now_points.y = R*sin(PI/2. + theta);
		ROS_INFO("After compensation: [%.4f, %.4f, %.4f]", now_points.x, now_points.y, now_points.z);
	}();
		
	//// do the tf transfrom from camera frame to base frame (my_frame)
	tf_transfrom_from_camera_to_base(now_points, listener);
	
	cout << "--------------------------------------------------" << endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cameara_listener");
	
	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_camera", 10);
	camera_pos_pub = n.advertise<geometry_msgs::Point>("camera_pose_calibration", 10);
	tf::TransformListener listener(ros::Duration(10));
	ros::Subscriber sub = n.subscribe<geometry_msgs::Pose>("human_pose", 1000,  boost::bind(CameraCallback, _1, boost::ref(listener)));
	
	ros::spin();

	return 0;
}

void tf_transfrom_from_camera_to_base(geometry_msgs::Point camera_pos, const tf::TransformListener& Listener)
{
	geometry_msgs::PointStamped camera_point;  //這裏定義了一個相對於camera_link座標系的點
	camera_point.header.frame_id="base_camera";  //   camera_depth_optical_frame
	camera_point.header.stamp=ros::Time();
	camera_point.point.x = camera_pos.x;     //設置相對於camera_link座標系的座標
	camera_point.point.y = camera_pos.y;
	camera_point.point.z = 0;

	try{
		geometry_msgs::PointStamped base_point;
		Listener.transformPoint("my_frame", camera_point, base_point); //Transform a Stamped Point Message into the target frame

		ROS_INFO("base_camera:(%.4f, %.4f, %.4f) -----> my_frame:(%.4f, %.4f, %.4f) at time %.2f",camera_point.point.x, camera_point.point.y, camera_point.point.z, base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
		base_point.point.z = 0;
		camera_pos_pub.publish(base_point.point); // publish the camera position to calculate the error
		
		// show the position in RVIZ
		show_camera_people_position(base_point.point);  
		// show the text in RVIZ
		show_camera_people_position_text_msg("pos from Camera", base_point.point);
	}
	catch(tf::TransformException &ex) {
		ROS_ERROR("Received an exception trying to transform a point form \"base_camera\" to \"my_frame\": %s", ex.what());
	}
	return;
}

void show_camera_people_position(geometry_msgs::Point camera2base_point)
{
	///// Mark the moving people from camera data ////
	///// The position of person is relative to the robot frame
	// %Tag(MARKER_INIT)%
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/my_frame";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "cameara_listener";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 6431;  // %Tag(ID)%
    line_list.type = visualization_msgs::Marker::LINE_LIST; // %Tag(TYPE)%
    line_list.scale.x = 0.05;  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width	
	// %Tag(COLOR)%
    line_list.color.r = 67./255;
	line_list.color.g = 246./255;
	line_list.color.b = 249./255;
    line_list.color.a = 1.0;
	
	line_list.points.push_back(camera2base_point);
	camera2base_point.z += 1.0;
	line_list.points.push_back(camera2base_point);
	marker_pub.publish(line_list);	
	
	return;
}

void show_camera_people_position_text_msg(string my_text, geometry_msgs::Point p)
{
	visualization_msgs::Marker label_text;
	label_text.header.frame_id = "/my_frame";
    label_text.header.stamp = ros::Time::now();
    label_text.ns = "planner1";
    label_text.action = visualization_msgs::Marker::ADD;
    label_text.pose.orientation.w = 1.0;	
    label_text.id = 20;  // %Tag(ID)%
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
	pose.position.z = p.z + 1.0;

	label_text.text = my_text;
	label_text.pose = pose;

	marker_pub.publish(label_text);
	return;
}
