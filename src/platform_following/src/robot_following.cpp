#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Empty.h"
#include "vector"

ros::Publisher takeoff_pub;
ros::Publisher pub_land;
geometry_msgs::Pose marker_pose;



double marker_nav_orientation;
double marker_nav_y;
double marker_nav_x;

double marker_find_z;
double marker_find_x;

double to_travel;
double movement_timer;
double waiting_timer;
double to_wait = 4.0;

double to_wait_2 = 3.0;
double waiting_timer_2;
bool start_waiting_2;

double landing_timer;

double last_message;

int cnt = 0;
int step = 1;
double d_speed = 0.05;
bool tag_found = false;
bool timer_start = false;
int cnt_movement = 30; 



class P_control
{
private:
  double _p_coefficent_y = 0.15; //0.15
  double _i_coefficent_y = 0.0;
  double _d_coefficent_y = 0.0;
  double _p_coefficent_x = 0.1;
  double _d_coefficent_x = 0.0;
  double _p_coefficent_or = 0.5;
  double _d_coefficent_or = 10.09;
  double i_part = 0.0;
  double prev_target_y = 0.0;
  double prev_target_x = 0.0;
  double _P_cont_linear (double target_y)
  {
    double p_part = -target_y *_p_coefficent_y;
    i_part += -target_y*0.1;
    double d_part = ((-target_y) - (-prev_target_y))/0.1;
    double final_speed = p_part + i_part*_i_coefficent_y + d_part*_d_coefficent_y;
    prev_target_y = target_y;
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
    double p_part = -target_x*_p_coefficent_x;
    double d_part = ((-target_x) - (-prev_target_x))/0.1;
    double final_speed = p_part + d_part*_d_coefficent_x;
    prev_target_x = target_x;
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
    double final_speed = -target_or*_p_coefficent_or;
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




void callBack_marker(visualization_msgs::Marker msg);
void drone_prep();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drone_control");
  ros::NodeHandle _nh;
  ros::Subscriber sub_marker = _nh.subscribe("visualization_marker",1,callBack_marker);
  takeoff_pub = _nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
  pub_land = _nh.advertise<std_msgs::Empty>("bebop/land",1);
  ros::Publisher control_pub = _nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);
  ros::Publisher camera_control_pub = _nh.advertise<geometry_msgs::Twist>("bebop/camera_control",1);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  //add change the camera angle to default one
  geometry_msgs::Twist camera_angle;
  camera_angle.angular.y = -30.0;
  camera_control_pub.publish(camera_angle);

  P_control p_control;

  ros::Duration(1.0).sleep();

  //drone_prep();

  std_msgs::Empty to_takeoff;
  takeoff_pub.publish(to_takeoff);
  
  last_message = ros::Time::now().toSec();

  ros::Rate sleepRate(10);

  while (ros::ok())
  {
    switch (step)
    {
    case 1: //find the tag
      {
        camera_angle.angular.y = -30.0;
        camera_control_pub.publish(camera_angle);
        geometry_msgs::Twist to_send;
        if (!tag_found)
        {
          //to_send.angular.z = 0.3;
        }
        else
        {
          double final_speed = -marker_find_x*0.4;
          if (final_speed > 0.5)
          {
            final_speed = 0.5;
          }
          else if (final_speed < -0.5)
          {
            final_speed = -0.5;
          }
          to_send.angular.z = final_speed;

          std::cout << final_speed << std::endl;

          if (abs(final_speed) <= 0.07)
          {
            if (!start_waiting_2)
            {
              start_waiting_2 = true;
              waiting_timer_2 = ros::Time::now().toSec();
            }
            else
            {
              if (ros::Time::now().toSec() - waiting_timer_2 >= to_wait_2)
              {
                step++;
                std::cout << "Ready to countinue!" << std::endl;
                to_travel = marker_find_z/d_speed/4.5;
                movement_timer = ros::Time::now().toSec();
              }
              
            }
            
          }
          else
          {
            start_waiting_2 = false;
          }
          
        }
        
        control_pub.publish(to_send);
      }
      break;

    case 2: //get to the robot
      {
        geometry_msgs::Twist to_send;
        
        if (ros::Time::now().toSec() - movement_timer < to_travel)
        {
          std::cout << "Time to travel: " << to_travel << std::endl;
          to_send.linear.x = d_speed;
          to_send.linear.z = 0.2;


          if (ros::Time::now().toSec() - movement_timer < to_travel-2)
          {
            double final_speed = -marker_find_x*0.4;
            if (final_speed > 0.5)
            {
              final_speed = 0.5;
            }
            else if (final_speed < -0.5)
            {
              final_speed = -0.5;
            }
            to_send.angular.z = final_speed;
          }

        }
        else
        {
          step++;
          camera_angle.angular.y = -90;
          camera_control_pub.publish(camera_angle);
        }
        
        control_pub.publish(to_send);
      }
      break;

    case 3: //following the robot
      {
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
          to_send = p_control.P_controller(marker_nav_y, marker_nav_x, marker_nav_orientation); //add init for checking!!
          if ((to_send.linear.x > 0.05) && (cnt_movement < 30))
          {
            cnt_movement++;
          }
          
          std::cout << to_send << std::endl;
          if ((to_send.linear.x <= 0.03) && (to_send.linear.x >= -0.03) && (to_send.linear.y <= 0.03) && (to_send.linear.y >= -0.03) && (to_send.angular.z <= 0.03) && (to_send.angular.z >= -0.03) && (cnt_movement >= 30))
          {
            if (timer_start)
            {
              if (ros::Time::now().toSec() - waiting_timer >= to_wait)
              {
                step++;
                std::cout << "Should be landing" << std::endl;
                landing_timer = ros::Time::now().toSec();
              }
              
            }
            else
            {
              timer_start = true;
              waiting_timer = ros::Time::now().toSec();
            }
          }
          else
          {
            timer_start = false;
          }
          
          
        }
        control_pub.publish(to_send);

      }
      break;
    
    case 4: //landing
    {
      geometry_msgs::Twist to_send;
      double final_speed = -(marker_nav_y-0.1) * 0.1;
      if (final_speed > 0.15)
      {
        final_speed = 0.15;
      }
      else if (final_speed < -0.15)
      {
        final_speed = -0.15;
      }
      to_send.linear.x = final_speed;
      control_pub.publish(to_send);

      {
        if (ros::Time::now().toSec() - landing_timer > 1)
        {
          std_msgs::Empty t_land;
        step++;
        pub_land.publish(t_land);
        }
        
      }
    }
      break;

    default:
      std::cout << "Done" << std::endl;
      break;
    }
    
    sleepRate.sleep();
    if (step > 5)
    {
      break;
    }
    std::cout << "Step: " << step << std::endl;
    //ros::spinOnce();

  }
  return 0;
}


void callBack_marker(visualization_msgs::Marker msg)
{
  if (msg.id == 4)
  {
    last_message = ros::Time::now().toSec();
    marker_nav_y = msg.pose.position.y;
    marker_nav_x = msg.pose.position.x;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    marker_nav_orientation = yaw;
    std::cout << "Yaw orientation of the top marker is: " << marker_nav_orientation <<std::endl;
    cnt = 0;
    
  }
  else if (msg.id == 5)
  {
    last_message = ros::Time::now().toSec();
    marker_find_z = msg.pose.position.z;
    marker_find_x = msg.pose.position.x;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    //std::cout << "Orientation of the marker is: " << marker_nav_orientation <<std::endl;
    std::cout << "Distance to the side marker is :" << marker_find_z << std::endl;
    cnt = 0;
    tag_found = true;
  }
  
}

void drone_prep()
{
  std_msgs::Empty to_send;
  takeoff_pub.publish(to_send);
}



//find the tag
//go to the tag
//follow the tag x
//land
