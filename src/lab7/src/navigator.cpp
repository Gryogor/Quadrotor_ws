#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "ds4_driver/Status.h"
#include "std_msgs/Empty.h"

geometry_msgs::Twist drone_control;
bool takenoff = false;
std_msgs::Empty to_send;
ros::Publisher takenoff_pub;
ros::Publisher land_pub;

void DS4callback(ds4_driver::Status msg)
{
  drone_control.linear.z = msg.axis_left_y;
  drone_control.linear.y = msg.axis_left_x;
  drone_control.linear.x = msg.axis_right_y;
  drone_control.angular.z = msg.axis_l2 - msg.axis_r2;
  if ((msg.button_cross == 1) and (!takenoff))
  {
    takenoff = true;
    takenoff_pub.publish(to_send);
  }
  if ((msg.button_circle == 1) and (takenoff))
  {
    takenoff = false;
    land_pub.publish(to_send);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drone_Navigation");
  ros::NodeHandle _nh;
  ros::Publisher move_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  takenoff_pub = _nh.advertise<std_msgs::Empty>("drone/takeoff",1);
  land_pub = _nh.advertise<std_msgs::Empty>("drone/land",1);
  ros::Subscriber sub = _nh.subscribe("status", 1, DS4callback);

  ros::Rate sleep_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    move_pub.publish(drone_control);
    sleep_rate.sleep();
  }
  return 0;
}