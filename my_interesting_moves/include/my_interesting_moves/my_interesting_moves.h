// my_interesting_moves header file //
/// sulu; Oct, 2015.
/// Include this file in "interesting_moves.cpp", and in any main that uses this library.
/// This class provides several interesting movements (functions), in terms of joints trajectories

#ifndef MY_INTERESTING_MOVES_H_
#define MY_INTERESTING_MOVES_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <iostream>
// #include <Eigen/Geometry>
// #include <Eigen/Eigenvalues>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <baxter_core_msgs/JointCommand.h>
// #include "trajectory_msgs/JointTrajectory.h"
// #include "trajectory_msgs/JointTrajectoryPoint.h"


typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
const double q0dotmax = 0.5;
const double q1dotmax = 0.5;
const double q2dotmax = 0.5;
const double q3dotmax = 0.5;
const double q4dotmax = 1;
const double q5dotmax = 1;
const double q6dotmax = 1;
const double dt_traj = 0.01; // time step for trajectory interpolation
const double SPEED_SCALE_FACTOR= 0.5; // go this fraction of speed from above maxes

class MyInsterestingMoves
{
public:
    MyInsterestingMoves(ros::NodeHandle* nodehandle);

    void rightArmSinMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory);
    void rightArmSaluteMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory);
    void rightArmZigzagMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory);
    void rightArmComeOnMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory);
    Vectorq7x1 getQvecRigthArm();
    sensor_msgs::JointState get_joint_states();

private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber joint_state_sub_; //these will be set up within the class constructor, hiding these ugly details
    Vectorq7x1 q_vec_right_arm_; //,q_in,q_soln,q_snapshot; 
    Vectorq7x1 qdot_max_vec; // velocity constraint on each joint for interpolation
    sensor_msgs::JointState joint_states_; // copy from robot/joint_states subscription
    baxter_core_msgs::JointCommand right_cmd_;  // define instances of these message types, to control arms


    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    // void initializePublishers();
    // void initializeServices();
    void jointStatesCb(const sensor_msgs::JointState& js_msg); //prototype for callback of joint-state messages
    // void map_right_arm_joint_indices(vector<string> joint_names);

    double transition_time(Vectorq7x1 dqvec);
    double transition_time(Eigen::VectorXd dqvec);
    //prototype for callback for example service
    //bool serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
};

#endif
