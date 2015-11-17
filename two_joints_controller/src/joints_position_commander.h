#ifndef JOINTS_POSITION_COMMANDER_CLASS_H_
#define JOINTS_POSITION_COMMANDER_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

double PI = 3.1415926;

class JointsPositionCommander
{
public:
    JointsPositionCommander(ros::NodeHandle* nodeHandle, const std::string joint_number, 
                            const double amplitude, const double frequency);

    ~JointsPositionCommander();
    void posProfileGen();
private:
    ros::NodeHandle nh_;

    ros::Publisher joints_pos_pub_;

    std::string joint_num_;

    std_msgs::Float64 joints_position_;

    double amplitude_;
    double frequency_;
    double dt_;
    double time_;

    void initializePublishers();

};
#endif