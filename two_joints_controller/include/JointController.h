#ifndef JOINT_CONTROLLER_CLASS_H_
#define JOINT_CONTROLLER_CLASS_H_

#include <ros/ros.h> //ALWAYS need to include this
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>

class JointController
{
public:
	JointController(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    ~JointController();
    controller();
private:
	ros::NodeHandle nh_;
	ros::Publisher trq_pub_;
	ros::Publisher vel_pub_;
	ros::Publisher pos_pub_;

	ros::Subscriber pos_cmd_sub_;

	ros::ServiceClient set_trq_client_;
	ros::ServiceClient get_jnt_state_client_;

	gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg_;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg_;

    bool service_ready_;
    bool result_;
    ros::Duration* half_sec_;
    ros::Duration* duration;
    ros::Rate* rate_timer_;

    std_msgs::Float64 trq_msg_;
    std_msgs::Float64 q1_msg,q1dot_msg_;
    sensor_msgs::JointState joint_state_msg_;

    double q1_;
    double q1dot_;
    double dt_;
    double q1_err_;
    double Kp_;
    double Kv_;
    double trq_cmd_;

    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();

    void posCmdCB(const std_msgs::Float64& pos_cmd_msg);
    double sat(double val, double sat_val);

};
#endif