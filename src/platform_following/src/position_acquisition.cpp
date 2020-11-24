#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class position_acquisition
{
private:
    geometry_msgs::Twist PID_c (geometry_msgs::Pose tag_position)
    {
        geometry_msgs::Twist speed;
        return speed;
    }
public:
    position_acquisition(/* args */);
    ~position_acquisition();
};

position_acquisition::position_acquisition(/* args */)
{
}

position_acquisition::~position_acquisition()
{
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_finding");
    ros::NodeHandle _nh;

    return 0;
}
