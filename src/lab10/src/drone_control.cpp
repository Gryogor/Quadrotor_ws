#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Empty.h"
#include "vector"

ros::Publisher pub_takeoff;
geometry_msgs::Pose marker_pose;

double marker_orientation;
double marker_z;
double marker_x;

double last_message;

int cnt = 0;





class P_control
{
private:
  double _p_coefficent_z = 0.5;
  double _p_coefficent_x = 0.5;
  double _p_coefficent_or = 0.5;
  double _P_cont_linear (double target_z)
  {
    double final_speed = (target_z - 1.0)*_p_coefficent_z;
    if (final_speed > 0.15)
    {
      final_speed = 0.15;
    }
    else if (final_speed < -0.15)
    {
      final_speed = -0.15;
    }
    return final_speed;
  }
  double _P_cont_linear_step (double target_x)
  {
    double final_speed = -target_x*_p_coefficent_x;
    if (final_speed > 0.15)
    {
      final_speed = 0.15;
    }
    else if (final_speed < -0.15)
    {
      final_speed = -0.15;
    }
    return final_speed;
  }
  double _P_cont_angular (double target_or)
  {
    double final_speed = target_or*_p_coefficent_or;
    if (final_speed > 0.15)
    {
      final_speed = 0.15;
    }
    else if (final_speed < -0.15)
    {
      final_speed = -0.15;
    }
    return final_speed;
  }
public:
  geometry_msgs::Twist P_controller (double target_z, double target_x, double target_or)
  {
    geometry_msgs::Twist controller_speed;
    controller_speed.linear.x = _P_cont_linear(target_z);
    controller_speed.linear.y = _P_cont_linear_step(target_x);
    controller_speed.angular.z = _P_cont_angular(target_or);
    return controller_speed;
  }
};



void callBack_marker(visualization_msgs::Marker msg);
void drone_prep();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drone_control");
  ros::NodeHandle _nh;
  ros::Subscriber sub_marker = _nh.subscribe("visualization_marker",1,callBack_marker);
  //pub_takeoff = _nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
  ros::Publisher control_pub = _nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);

  P_control p_control;

  //drone_prep();
  last_message = ros::Time::now().toSec();

  ros::Rate sleepRate(10);

  while (ros::ok())
  {
    double difference = ros::Time::now().toSec() - last_message;
    geometry_msgs::Twist to_send;
    if (difference > 0.5)
    {
      marker_z = 0.0;
      marker_x = 0.0;
      marker_orientation = 0.0;
      
      to_send = p_control.P_controller(marker_z, marker_x, marker_orientation);
      to_send.linear.x = 0.0;
    }
    else
    {
      to_send = p_control.P_controller(marker_z, marker_x, marker_orientation);
    }

    control_pub.publish(to_send);
    
    sleepRate.sleep();
    ros::spinOnce();

  }
  
  

  return 0;
}


void callBack_marker(visualization_msgs::Marker msg)
{
  last_message = ros::Time::now().toSec();
  marker_z = msg.pose.position.z;
  marker_x = msg.pose.position.x;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  marker_orientation = pitch;
  std::cout << "Orientation of the marker is: " << marker_orientation <<std::endl;
  cnt = 0;
}

void drone_prep()
{
  std_msgs::Empty to_send;
  pub_takeoff.publish(to_send);
}

/*
class PID_control
{
private:
  double _k_p_l = 0.0;
  double _k_i_l = 0.0;
  double _k_d_l = 0.0;
  double _PID_cont_linear (double target_z)
  {
    double final_speed = 0.0;
    return final_speed;
  }
  double _PID_cont_linear_step (double target_y)
  {
    double final_speed = 0.0;
    return final_speed;
  }
  double _PID_cont_angular (double target_or)
  {
    double final_speed = 0.0;
    return final_speed;
  }

public:
  geometry_msgs::Twist PID_controller (double target_z, double target_y, double target_or)
  {
    geometry_msgs::Twist controller_speed;
    controller_speed.linear.x = _PID_cont_linear(target_z);
    controller_speed.linear.y = _PID_cont_linear_step(target_y);
    controller_speed.angular.z = _PID_cont_angular(target_or);
  }
};
*/