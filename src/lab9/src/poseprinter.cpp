#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"

ros::Publisher pub;

void callBack(visualization_msgs::Marker msg)
{
  std::cout << msg.pose << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Pose_Printer");
  ros::NodeHandle _nh;
  ros::Subscriber sub = _nh.subscribe("visualization_marker",1,callBack);

  ros::spin();

  return 0;
}