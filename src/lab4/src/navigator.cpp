#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"
#include "vector"

geometry_msgs::Point robot_position;
geometry_msgs::Point goal_position;
float robot_orientation;


void odomcallback(nav_msgs::Odometry msg)
{
  robot_position = msg.pose.pose.position;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  robot_orientation = yaw;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Goal_Navigation");
  ros::NodeHandle _nh;
  ros::Publisher move_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::Subscriber sub = _nh.subscribe("odom", 1, odomcallback);
  ros::Rate sleep_rate(5);
  std::cout << "Please enter x and y coordinates of the goal:" << std::endl;
  std:: cout << "x: ";
  std::cin >> goal_position.x;
  std:: cout << "y: ";
  std::cin >> goal_position.y;

  ros::Duration(1).sleep();

  while(ros::ok())
  {
    ros::spinOnce();
    float orientation_to_goal = atan2(goal_position.y-robot_position.y, goal_position.x-robot_position.x);
    std::cout << "Orientation to goal: " << orientation_to_goal << std::endl;
    std::cout << "Robot orientation: " << robot_orientation << std::endl;
    sleep_rate.sleep();
  }
  return 0;
}