#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;
	//tf::TransformBroadcaster broadcaster2;
	tf::TransformBroadcaster broadcaster3;
	tf::TransformBroadcaster broadcaster4;
	
	//tf::TransformListener listener;
	//tf::StampedTransform transform;

	while(n.ok())
	{
		
		
		broadcaster.sendTransform(
			tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.175, -0.40, 0.85)), ros::Time::now(),"my_frame", "base_camera"));    // Z(0.85)    camera_depth_optical_frame
		//broadcaster2.sendTransform(
			//tf::StampedTransform( tf::Transform(tf::Quaternion(-0.707, 0, 0, 0.707), tf::Vector3(0, 0, 0)), ros::Time::now(),"base_camera", "camera_depth_optical_frame"));    // Z(0.85)    camera_depth_optical_frame
		broadcaster3.sendTransform(
			tf::StampedTransform( tf::Transform(tf::Quaternion(0, 1, 0, 0), tf::Vector3(8, -8, 0)), ros::Time::now(),"my_frame", "Lidar_frame"));    // Z(0.85)    camera_depth_optical_frame
		broadcaster4.sendTransform(
			tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, -0.707, 0.707), tf::Vector3(0.1, 0, 0)), ros::Time::now(),"base_link", "my_frame"));    //   odom  base_footprint
		
		/*tf::Quaternion rotate;
		rotate.setRPY(0,0,-90);
		rotate = rotate*transform.getOrigin();*/
		
		/*try
		{
			listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			broadcaster4.sendTransform(
			tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, -0.707, 0.707)*transform.getRotation(), transform.getOrigin()), ros::Time::now(),"odom", "my_frame"));    //   odom  base_footprint
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}*/
		
		
		r.sleep();
	}
}