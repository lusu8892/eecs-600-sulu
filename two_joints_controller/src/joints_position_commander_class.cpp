#include "joints_position_commander.h"

JointsPositionCommander::JointsPositionCommander(ros::NodeHandle* nodeHandle, 
					const std::string joint_number, const double amplitude, const double frequency)
					:nh_(*nodeHandle), joint_num_(joint_number), amplitude_(amplitude), frequency_(frequency)
{
	ROS_INFO("in class constructot of JointsPositionCommander");

	initializePublishers();

	dt_ = 0.01;
	time_ = 0.0;
}

JointsPositionCommander::~JointsPositionCommander()
{

}

void JointsPositionCommander::posProfileGen()
{
	joints_position_.data = amplitude_ * sin(2 * PI * frequency_ * time_);
	ROS_INFO_STREAM(joint_num_ << "_position" << "="<< joints_position_.data);
	time_ += dt_;
	joints_pos_pub_.publish(joints_position_);
}

void JointsPositionCommander::initializePublishers()
{
	joints_pos_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_pos_cmd",1,true);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joints_position_commander");
    ros::NodeHandle nh;

    ros::Rate naptime(10.0);

    JointsPositionCommander jointsPositionCommander_1(&nh, "joint1", 0.7, 8.0);
    JointsPositionCommander jointsPositionCommander_2(&nh, "joint2", 0.5, 5.0);


    while (ros::ok()) 
    {
        jointsPositionCommander_1.posProfileGen();
        jointsPositionCommander_2.posProfileGen();

        naptime.sleep();
    }
}