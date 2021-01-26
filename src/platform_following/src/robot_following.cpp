#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Empty.h"
#include "vector"

ros::Publisher pub_takeoff;
ros::Publisher pub_land;
geometry_msgs::Pose marker_pose;

double marker_nav_orientation;
double marker_nav_y;
double marker_nav_x;

double marker_find_z;
double marker_find_x;

double to_travel;
double begin_timer;


double last_message;

int cnt = 0;
int step = 1;
double d_speed = 0.15;
bool tag_found = false;




class P_control
{
private:
  double _p_coefficent_y = 0.5;
  double _p_coefficent_x = 0.5;
  double _p_coefficent_or = 0.5;
  double _P_cont_linear (double target_y)
  {
    double final_speed = (target_y - 1.0)*_p_coefficent_y;
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
  geometry_msgs::Twist P_controller (double target_y, double target_x, double target_or)
  {
    geometry_msgs::Twist controller_speed;
    controller_speed.linear.x = _P_cont_linear(target_y);
    controller_speed.linear.y = _P_cont_linear_step(target_x);
    controller_speed.angular.z = _P_cont_angular(target_or);
    return controller_speed;
  }
};


class start_point
{
private:
  /* data */
public:
  
};





void callBack_marker(visualization_msgs::Marker msg);
void drone_prep();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drone_control");
  ros::NodeHandle _nh;
  ros::Subscriber sub_marker = _nh.subscribe("visualization_marker",1,callBack_marker);
  //pub_takeoff = _nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
  pub_land = _nh.advertise<std_msgs::Empty>("bebop/land",1);
  ros::Publisher control_pub = _nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);

  //add change the camera angle to default one

  P_control p_control;

  drone_prep();
  last_message = ros::Time::now().toSec();

  ros::Rate sleepRate(10);

  while (ros::ok())
  {
    switch (step)
    {
    case 1: //find the tag
      geometry_msgs::Twist to_send;
      if (!tag_found)
      {
        to_send.angular.z = 0.05;
      }
      else
      {
        double final_speed = -marker_find_x*0.5;
        if (final_speed > 0.15)
        {
          final_speed = 0.15;
        }
        else if (final_speed < -0.15)
        {
          final_speed = -0.15;
        }
        to_send.angular.z = final_speed;
      }
      if (to_send.angular.z <= 0.01)
      {
        step++;
        to_travel = marker_find_z/d_speed;
        begin_timer = ros::Time::now().toSec();
      }
      
      control_pub.publish(to_send);
      
      break;
    case 2: //get to the robot
      geometry_msgs::Twist to_send;
      
      if (ros::Time::now().toSec() - begin_timer < to_travel)
      {
        to_send.linear.x = d_speed;
      }
      else
      {
        step++;
        //add the camera angle change
      }
      control_pub.publish(to_send);
      
      break;

    case 3: //following the robot
      double difference = ros::Time::now().toSec() - last_message;
      geometry_msgs::Twist to_send;
      if (difference > 0.5)
      {
        marker_nav_y = 0.0;
        marker_nav_x = 0.0;
        marker_nav_orientation = 0.0;
        
        to_send = p_control.P_controller(marker_nav_y, marker_nav_x, marker_nav_orientation);
        to_send.linear.x = 0.0;
      }
      else
      {
        to_send = p_control.P_controller(marker_nav_y, marker_nav_x, marker_nav_orientation);
        if ((to_send.linear.x <= 0.03) && (to_send.linear.x >= -0.03) && (to_send.linear.y <= 0.03) && (to_send.linear.y >= -0.03) && (to_send.angular.z <= 0.03) && (to_send.angular.z >= -0.03))
        {
          step++;
        }
        
      }
      control_pub.publish(to_send);
      break;
    
    case 4: //landing
      std_msgs::Empty t_land;
      step++;
      pub_land.publish(t_land);


    default:
      std::cout << "Done" << std::endl;
      break;
    }
    
    sleepRate.sleep();
    if (step > 5)
    {
      break;
    }
    
    ros::spinOnce();

  }
  return 0;
}


void callBack_marker(visualization_msgs::Marker msg)
{
  if (msg.id == 0)
  {
    last_message = ros::Time::now().toSec();
    marker_nav_y = msg.pose.position.y;
    marker_nav_x = msg.pose.position.x;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    marker_nav_orientation = yaw;
    std::cout << "Orientation of the marker is: " << marker_nav_orientation <<std::endl;
    cnt = 0;
    
  }
  else if (msg.id)
  {
    last_message = ros::Time::now().toSec();
    marker_find_z = msg.pose.position.z;
    marker_find_x = msg.pose.position.x;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    std::cout << "Orientation of the marker is: " << marker_nav_orientation <<std::endl;
    cnt = 0;
    tag_found = true;
  }
  
}

void drone_prep()
{
  std_msgs::Empty to_send;
  pub_takeoff.publish(to_send);
}



//find the tag
//go to the tag
//follow the tag x
//land
