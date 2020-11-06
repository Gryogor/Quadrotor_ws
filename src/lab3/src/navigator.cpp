#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "vector"

ros::Publisher move_pub;

bool rotate = false;

void scanCallback(sensor_msgs::LaserScan msg)
{
  geometry_msgs::Twist speed;
  for (int i = 0; i < 15; i++)
  {
    if (msg.ranges[i] < 0.5 || msg.ranges[345+i] < 0.5)
    {
      rotate = true;
      std::cout << msg.ranges[i] << "\t" << msg.ranges[345+i] <<std::endl;
    }
  }
  if (speed.angular.z == 0)
  {
    speed.linear.x = 0.4;
  }
  move_pub.publish(speed);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Goal_Navigation");
  ros::NodeHandle _nh;
  move_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::Subscriber sub = _nh.subscribe("scan", 1, scanCallback);
  ros::Rate sleep_rate(50);

  ros::Duration(1).sleep();

  while(ros::ok())
  {
    if (rotate == true)
    {
      geometry_msgs::Twist speed;
      speed.angular.z = 0.315;
      for (int i = 0; i < 5; i++)
      {
        move_pub.publish(speed);
        ros::Duration(1).sleep();
      }
      rotate = false;
    }
    
    ros::spinOnce();
    sleep_rate.sleep();
  }
  return 0;
}