#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


int main (int argc, char **argv){

  ros::init (argc, argv, "path_planner");
  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path",10);

	int num =1;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		nav_msgs::Path path;
		path.header.stamp=ros::Time::now();
		path.header.frame_id="odom";
		static int i = 0;
		while (path.poses.size() <= num)
		{
			float angle = i * 2 * M_PI / num;
			i++;
			//float x = 5 * cos(angle);
			//float y = 5 * sin(angle);
			float	x= 10.0;
			float y= 10.0;
			geometry_msgs::PoseStamped p;
			p.pose.position.x = x;
			p.pose.position.y = y;

			//geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(1);
			p.pose.orientation.x = 0;
			p.pose.orientation.y = 0;
			p.pose.orientation.z = 0;
			p.pose.orientation.w = 1;

			p.header.stamp=ros::Time::now();;
			p.header.frame_id="odom";

			path.poses.push_back(p);
		}
		path_pub.publish(path);
		//ros::spinOnce();               // check for incoming messages
		loop_rate.sleep();
	}

	return 0;
}
