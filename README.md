## Working Report of 2019 Summer
对所设计Path，Odom_fake，Controller部分进行一些简要的readme说明，以及目前存在的一些问题。
#### 1. Path部分：
path代码：
```c++
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
  //ros::spinOnce(); // check for incoming messages
  loop_rate.sleep();
 }
 return 0;

}
```
现路径信息只是发布一个目标点(10,10)，如需发布一个圆心为(0,0)，半径为5的圆形轨迹路径，则将上述代码修改部分为：
```
  int num =50;// 修改为4为正方形，3则为三角形
...
  float x = 5 * cos(angle);
  float y = 5 * sin(angle);
...
```

然后在终端输入以下命令发布路径
```
cd catkin_littlebot_ws
source devel/setup.bash
catkin_make
roslaunch path path.launch
```
若在rviz中可视化路径Path 在终端输入以下命令
```
rosrun rviz rviz
```
打开rviz然后，修改Globa Option中的Fixed Frame 为 odom，如图所示：

![](https://github.com/bryceustc/ROS/blob/master/img/1.png)

然后点击左下角Add，添加Path，设置Topic 为Path，如图所示：

![](https://github.com/bryceustc/ROS/blob/master/img/2.png)

则会在rviz中显示路径信息（以圆形轨迹为例），如图所示：

![](https://github.com/bryceustc/ROS/blob/master/img/3.png)

#### 2. Odom_fake部分：
turtlebot3_fake代码:
```c++
#include <turtlebot3_fake/turtlebot3_fake.h>

Turtlebot3Fake::Turtlebot3Fake()
: nh_priv_("~")
{
  bool init_result = init();
  ROS_ASSERT(init_result);
}

Turtlebot3Fake::~Turtlebot3Fake()
{
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Fake::init()
{
  // initialize ROS parameter

  std::string robot_model = nh_.param<std::string>("tb3_model", "");
   wheel_seperation_ = 0.350;
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_link"));

  // initialize variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_ = 0.0;
  goal_angular_velocity_ = 0.0;
  cmd_vel_timeout_ = 1.0;
  last_position_[LEFT] = 0.0;
  last_position_[RIGHT] = 0.0;
  last_velocity_[LEFT] = 0.0;
  last_velocity_[RIGHT] = 0.0;

  double pcov[36] = { 0.1, 0, 0, 0, 0, 0,
                        0, 0.1, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 0.2};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);

  // initialize subscribers
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 50, &Turtlebot3Fake::commandVelocityCallback, this);

  prev_update_time_ = ros::Time::now();

  return true;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void Turtlebot3Fake::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT] = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool Turtlebot3Fake::updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / WHEEL_RADIUS; // w = v / r
  //ROS_INFO("WHEEL_RADIUS: %f", WHEEL_RADIUS);
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec(); // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}


/*******************************************************************************
* Update function
*******************************************************************************/
bool Turtlebot3Fake::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // zero-ing after timeout
  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_fake_node");
  Turtlebot3Fake tb3fake;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    tb3fake.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
```

在终端输入以下命令发布控制器指令
```
cd catkin_littlebot_ws
catkin_make
source devel/setup.bash
roslaunch turtlebot3_fake turtlebot3_fake.launch
```
同样可在rviz中可视化Odometry
在终端输入命令：
```
rosrun rviz rviz`
```
打开rviz后，点击左下角Add，添加Odometry，并订阅topic 为/odom，如图所示：

![](https://github.com/bryceustc/ROS/blob/master/img/4.png)

根据右手坐标系即可看到，odom的朝向为x轴正方向，左边为y轴正方向

![](https://github.com/bryceustc/ROS/blob/master/img/5.png)

此odom会反馈当前小车的位置信息，速度信息，传给控制器以便控制小车运动。

#### 3. Controller部分：
controller 代码：
```c++
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
tf::Point Odom_pos; //odometry position (x, y, z)
tf::Point Path_pos;	//path position (x, y, z)

double Odom_yaw; //odometry orientation (yaw)

double Odom_v = 0.0; 
double Odom_w = 0.0; //odometry linear and angular speeds

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
        //Error Angle correction for large angles
        angleError = geometry.correctAngle(angleError);
        
        speedError =speedSet - Odom_v;
        
        //PID control
        omega = pidTheta.calculate(0, -angleError);
        
        //speed=speedSet;
        
        speed = pidVelocity.calculate(0, -speedError);

        ROS_INFO("Odom_yaw %f, Angle Error: %f , omega: %f Speed %f, Speed Error: %f , speedSet: %f", Odom_yaw, angleError, omega, Odom_v, speedError, speedSet);

        gtwist.linear.x = speed;
        gtwist.angular.z = omega;

        //publish the message
        cmd_vel.publish(gtwist);
        i++;
        ros::spinOnce();
        loop_rate.sleep();
    }
```

然后在终端输入以下命令发布控制器指令
```
cd catkin_littlebot_ws
catkin_make
source devel/setup.bash
roslaunch pid_control control.launch
```
注：pid_control中的control.launch 已经包含了path.launch，turtlebot3_fake.launch，可以实现一键launch。

控制器的效果如下图所示：

（1）单独目标点（10,10）

转45°，向目标点行进
    
![](https://github.com/bryceustc/ROS/blob/master/img/6.png)

到达目标点（10,10）

![](https://github.com/bryceustc/ROS/blob/master/img/7.png)

odom实时信息：

![](https://github.com/bryceustc/ROS/blob/master/img/8.png)

（2）圆心为（0,0），半径为5的圆形轨迹

沿所给轨迹点运动

![](https://github.com/bryceustc/ROS/blob/master/img/9.png)

odom实时信息：

![](https://github.com/bryceustc/ROS/blob/master/img/11.png)

整体一个rqt_graph如下图所示：

![](https://github.com/bryceustc/ROS/blob/master/img/10.jpg)

PS：

控制器设定目前最大速度为1m/s，速度设定为0.5m/s。

Path发布规划路径点的坐标，以便控制器接收。

Odom_fake 设定轮间距为0.35m，仿真车轮半径与实际小车类似，大约为6.5cm。

三者频率都为10Hz。

手柄控制急停，长按B键急停制动锁死，按X键解除锁死，以及手柄与pid_controller的切换也在相关文件中有说明。

以上一些代码都做了部分注释，且所需头文件已放在相关文件夹中。

#### 4. 当前存在的问题：

（1）实际Odom计算暂时不够准确。目标点设定为（3,0）实际也是沿x轴走了3m，但实际里程计显示只有（2.2，-0.6）左右，还需进一步调试odom的精准度。

（2）目前剑灏设计的dwa_planner与pid_controller一起运行时会报错，dwa_planner设计需要odom信息知道当前位置信息来规划下一时刻路径，而pid_controller需要路径规划好的目标点才会运动，这有一点矛盾，而且发布的路径规划的坐标点过多过快不知controller是否没有成功读取。

（3）pid_controller部分还需要根据实际调试效果进行调参。
