#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sstream>
#include "geometry.h"
#include "pid.h"


// global vars
tf::Point Odom_pos;    //odometry position (x, y, z)
tf::Point Path_pos;		//path position (x, y, z)

double Odom_yaw;    //odometry orientation (yaw)

double Odom_v = 0.0; 
double Odom_w = 0.0;    //odometry linear and angular speeds

double Path_x = 0.0;
double Path_y = 0.0; 

// ROS Topic Publishers
ros::Publisher cmd_vel;
//ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;
ros::Subscriber path_sub;



//Global variables for PID

double maxSpeed = 1;
double minSpeed = 0.01;
double distanceConst = 0.5;
double dt = 0.02, maxT = M_PI, minT = -M_PI, Kp = 0.3, Ki = 0.05, Kd = 0.01;
double dtS = 0.02, maxS = maxSpeed, minS = minSpeed, KpS = 0.5, KiS = 0.01, KdS = 0.01;



/*
 * Callback function for odometry msg, used for odom subscriber
 */
 
 
void odomCallback(const nav_msgs::Odometry odom_msg) {
    
    tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);//convert Point msg to Point
    Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);//tf helper function for getting yaw from a Quaternion

    //update observed linear and angular speeds
    Odom_v = odom_msg.twist.twist.linear.x;
    Odom_w = odom_msg.twist.twist.angular.z;

}



/*
 * Callback function for path msg, used for path subscriber
 */

void pathCallback(const nav_msgs::Path path_msg){
		
		tf::pointMsgToTF(path_msg.poses[0].pose.position, Path_pos);
		
		Path_x=path_msg.poses[0].pose.position.x;
		Path_y=path_msg.poses[0].pose.position.y;

}



/*
 * main function 
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "control");
    ros::NodeHandle nh("~");

    cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    odom_sub = nh.subscribe("odom", 50, odomCallback);
    path_sub = nh.subscribe("path",50, pathCallback);

    ros::Rate loop_rate(10); // ros spins 10 frames per second

    geometry_msgs::Twist gtwist;


    //Trajectory details are here
    Geometry geometry;

    double angleError = 0.0;
    double speedError = 0.0;
    static int i =0;
    
    VECTOR2D *prev = NULL, *current = NULL;
    	
    while (ros::ok()) {
          
    		if(i>=1)
    		{   
        VECTOR2D *temp = new VECTOR2D(Path_x, Path_y);

        geometry.trajectory.push_back(*temp);

        current = temp;

        if (prev != NULL) {

            geometry.path.push_back(geometry.getLineSegment(*prev, *current));

        	}
        prev = current;
        }
        

/*

Control strategy is here

*/



        double omega = 0.0;
        double speed = 0.0;
        double speedSet = 0.0;
        double prevDistError = 0.0;


        PID pidTheta = PID(dt, maxT, minT, Kp, Kd, Ki);

        PID pidVelocity = PID(dtS, maxS, minS, KpS, KdS, KiS);

        /*Error calculation*/
        VECTOR2D current_pos, pos_error;
        current_pos.x = Odom_pos.x();
        current_pos.y = Odom_pos.y();
        
        //ROS_INFO("p.x %f", Path_x);
				//ROS_INFO("p.y %f", Path_y);
        
        ROS_INFO("GoalPoint %f,%f ", Path_x, Path_y);
        ROS_INFO("CurrentPosition %f,%f ", current_pos.x, current_pos.y);

        Geometry::LineSegment *linesegment = geometry.getNearestLine(current_pos);
     

        Geometry::LineSegment linesegmentPerpen = geometry.getMinimumDistanceLine(*linesegment, current_pos);


        Geometry::LineSegment * NextLinesegment = geometry.getNextLineSegment(linesegment);

        double targetDistanceEnd = geometry.getDistance(current_pos, linesegment->endP);
        double targetDistanceStart = geometry.getDistance(current_pos, linesegment->startP);

        //Distance Error
        double distError = 0.0;

        double targetAnglePerpen = geometry.getGradient(current_pos, linesegmentPerpen.endP);

        VECTOR2D target = linesegment->endP;
        double targetAngle = geometry.getGradient(current_pos, target);
        double distanceToClosestPath = abs(linesegment->disatanceToAObj);
        
				
        //Error calculation based on angles
        if (distanceToClosestPath < distanceConst) {

            angleError = targetAngle - Odom_yaw;
            

        } else {

            angleError = targetAnglePerpen - Odom_yaw;

        }

        speedSet = 0.5*maxSpeed;
        
       //ROS_INFO("linesegment->disatance: %f",linesegment->disatance);
       //ROS_INFO("NextLinesegment->disatance: %f",NextLinesegment->disatance);

//If lines are long and it has sharp edges
			/*
        // if (NextLinesegment->disatance > 3.0 && linesegment->disatance > 3.0) //
        //{
            
            //angleError correction for sharp turns
            if (targetDistanceEnd < 0.5) {
                double futureAngleChange = NextLinesegment->gradient - linesegment->gradient;
             
                futureAngleChange = geometry.correctAngle(futureAngleChange);

                double combined = targetAngle + futureAngleChange;
                
                ROS_INFO("test");

                angleError = combined - Odom_yaw;
            }

						
            //Velocity Error Calculation for sharp turns
            if (targetDistanceStart < 0.7 || targetDistanceEnd < 0.7) {

                double targetDistance = targetDistanceEnd;

                if (targetDistanceStart < targetDistanceEnd)
                    targetDistance = targetDistanceStart;

                double speedError = 0.3 * maxSpeed * exp(-abs(targetDistance));

                speed = pidVelocity.calculate(maxSpeed, speedError);
            }
            

       // }
        
  */
        //Error Angle correction for large angles
        angleError = geometry.correctAngle(angleError);
        
        speedError =speedSet - Odom_v;
        
        //PID control
        omega = pidTheta.calculate(0, -angleError);
        
        speed=speedSet;
        
        //speed = pidVelocity.calculate(0, -speedError);

        ROS_INFO("Odom_yaw %f, Angle Error: %f , omega: %f Speed %f, Speed Error: %f , speedSet: %f", Odom_yaw, angleError, omega, Odom_v, speedError, speedSet);

        gtwist.linear.x = speed;
        gtwist.angular.z = omega;

        //publish the message
        cmd_vel.publish(gtwist);

				i++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}



