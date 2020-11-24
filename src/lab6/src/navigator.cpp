#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"
#include "vector"
#include "geometry_msgs/Pose.h"

geometry_msgs::Point robot_position;
geometry_msgs::Point goal_position;
float robot_orientation;
float goal_orientation;
float distance_to_goal;
float p_component = 0.0; 
float i_component = 0.0;
float d_component = 0.0;
float k_p = 9.5*0.6; 
float k_i = 0.5*(7.13);
float k_d = 0.125*(7.13);
float past = 0.0;
float past_dist = 0.0;

float remap(float value, float istart, float istop, float ostart, float ostop) {
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

void odomcallback(nav_msgs::Odometry msg)
{
  robot_position = msg.pose.pose.position;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  robot_orientation = yaw;
}

void goalCallback(geometry_msgs::Pose msg)
{
  goal_position.x = msg.position.x;
  goal_position.y = msg.position.y;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  goal_orientation = yaw;
}

float angular_controller(float angle)
{
  float final_speed = 0.0;
  p_component = angle;
  i_component += angle;
  d_component = angle-past;
  past = angle;

  final_speed = k_p*p_component + k_i*i_component + k_d*d_component;

  return final_speed;
}
float linear_controller(float distance)
{
  if (distance < 3.0)
  {

    p_component = distance;
    i_component += distance;
    d_component = distance-past_dist;
  }
  else
  {
    return 0.0;
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Goal_Navigation_v3");
  ros::NodeHandle _nh;
  ros::Publisher move_pub = _nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);
  ros::Subscriber sub = _nh.subscribe("odom", 1, odomcallback);
  ros::Subscriber goal_sub = _nh.subscribe("turtlebot/goal_pose", 1, goalCallback);
  ros::Rate sleep_rate(50);

  ros::Duration(1).sleep();

  while(ros::ok())
  {
    ros::spinOnce();
    geometry_msgs::Twist speeds;
    float orientation_to_goal = atan2(goal_position.y-robot_position.y, goal_position.x-robot_position.x) - robot_orientation;
    distance_to_goal = abs(sqrt(pow(goal_position.y-robot_position.y,2)+pow(goal_position.x-robot_position.x,2)));
    if (distance_to_goal < 0.05)
    {
      orientation_to_goal = goal_orientation - robot_orientation;
    }
    if (abs(orientation_to_goal) > M_PI)
    {
      if (orientation_to_goal > 0)
      {
        orientation_to_goal = -2*M_PI + orientation_to_goal;
      }
      else
      {
        orientation_to_goal = 2*M_PI + orientation_to_goal;
      }
      
    }

    std::cout << orientation_to_goal << std::endl;
    speeds.angular.z = angular_controller(orientation_to_goal);
    speeds.linear.x = linear_controller(distance_to_goal);
    move_pub.publish(speeds);

    sleep_rate.sleep();
  }
  return 0;
}