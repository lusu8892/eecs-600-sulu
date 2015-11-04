// Created by Su Lu on 28/10/2015.
// Copyright @ 2015 Su Lu. All rights reserved.
#include <my_interesting_moves/my_interesting_moves.h>
#include <string>
#include <vector>

using std::cout;
using std::endl;

MyInterestingMoves::MyInterestingMoves(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    initializeSubscribers();  // package up the messy work of creating subscribers; do this overhead in constructor
    qdot_max_vec_ << q0dotmax, q1dotmax, q2dotmax, q3dotmax, q4dotmax, q5dotmax, q6dotmax;
    qdot_max_vec_ *= SPEED_SCALE_FACTOR;
}

void MyInterestingMoves::rightArmZeroConfig(std::vector<Eigen::VectorXd> qvecs,
        trajectory_msgs::JointTrajectory &new_trajectory, double &final_time)
{
    Vectorq7x1 q_zero_pose;
    q_zero_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    qvecs.push_back(q_zero_pose);
    stuffTrajectory(qvecs, new_trajectory, final_time);
}

void MyInterestingMoves::rightArmSaluteMove(std::vector<Eigen::VectorXd> qvecs,
        trajectory_msgs::JointTrajectory &new_trajectory, double &final_time)
{
    Vectorq7x1 q_zero_pose;
    q_zero_pose << PI/5, PI/4, (170 * PI) / 180, (90 * PI) / 180, (170 * PI) / 180, 0, 0;  // s0 s1 e0 e1 w0 w1 w2
    qvecs.push_back(q_zero_pose);

    Vectorq7x1 q_des_pose;
    q_des_pose << PI/5, PI/4, PI / 2, (90 * PI) / 180, (170 * PI) / 180, 0, 0;  // s0 s1 e0 e1 w0 w1 w2

    for (int cycle = 0; cycle < 3; cycle ++)
    {
        qvecs.push_back(q_des_pose);
        qvecs.push_back(q_zero_pose);
    }
    stuffTrajectory(qvecs, new_trajectory, final_time);
}


void MyInterestingMoves::rightArmZigzagMove(std::vector<Eigen::VectorXd> qvecs,
        trajectory_msgs::JointTrajectory &new_trajectory, double &final_time)
{
    Vectorq7x1 q_zero_pose;
    q_zero_pose << -PI/4, 0, (170 * PI) / 180, 0, 0, 0, 0;  // s0 s1 e0 e1 w0 w1 w2

    Vectorq7x1 q_des_pose;
    q_des_pose << -PI/4, PI/4, (170 * PI) / 180, (120 * PI) / 180, 0, -PI/2, 0;  // s0 s1 e0 e1 w0 w1 w2

    qvecs.push_back(q_zero_pose);

    Eigen::VectorXd freeze_joint_val = qvecs.back();  // remember the current joints value
    Vectorq7x1 q_vec_des;

    for (int cycle; cycle < 3; cycle ++)
    {
        qvecs.push_back(q_des_pose);
        qvecs.push_back(q_zero_pose);
    }

    stuffTrajectory(qvecs, new_trajectory, final_time);
}


void MyInterestingMoves::rightArmComeOnMove(std::vector<Eigen::VectorXd> qvecs,
        trajectory_msgs::JointTrajectory &new_trajectory, double &final_time)
{
    Vectorq7x1 q_zero_pose;
    q_zero_pose << -PI/4, -PI/3, (170 * PI) / 180, PI/6, 0, 0, 0;  // s0 s1 e0 e1 w0 w1 w2

    Vectorq7x1 q_des_pose;
    q_des_pose << -PI/4, -PI/3, (170 * PI) / 180, PI/3, 0, 0, 0;  // s0 s1 e0 e1 w0 w1 w2

    for (int cycle = 0; cycle < 3; cycle ++)
    {
        qvecs.push_back(q_des_pose);
        qvecs.push_back(q_zero_pose);
    }

    stuffTrajectory(qvecs, new_trajectory, final_time);
}

Vectorq7x1 MyInterestingMoves::getQvecRigthArm()
{
    return q_vec_right_arm_;
}

sensor_msgs::JointState MyInterestingMoves::getJointStates()
{
    return joint_states_;
}

void MyInterestingMoves::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    joint_state_sub_ = nh_.subscribe("robot/joint_states", 1, &MyInterestingMoves::jointStatesCb, this);
    ROS_INFO("subscriber has been initialized");
    // add more subscribers here, as needed
}

// NOTE: this is not separately threaded.  this callback only responds with the parent node allows a ros spin.
void MyInterestingMoves::jointStatesCb(const sensor_msgs::JointState& js_msg)
{
    joint_states_ = js_msg;  // copy this to member var
    if (right_arm_joint_indices_.size() < 1)
    {
       map_right_arm_joint_indices(js_msg.name);
    }
    // copy right-arm angles to global vec
    for (int i = 0; i < 7; i++)
    {
        // should do this better; manually remap from joint_states indices to right-arm joint angles
        q_vec_right_arm_[i] = js_msg.position[right_arm_joint_indices_[i]];
        // q_vec_right_arm_[i] = js_msg.position[i];  //w2
    }
    cout << "CB: q_vec_right_arm: " << q_vec_right_arm_.transpose() << endl;
}

double MyInterestingMoves::transitionTime(Eigen::VectorXd dqvec)
{
    double t_max = fabs(dqvec[0]) / qdot_max_vec_[0];
    cout << "qdot max: " << qdot_max_vec_.transpose() << endl;
    double ti;
    for (int i=1; i < dqvec.size(); i++)
    {
        ti = fabs(dqvec[i]) / qdot_max_vec_[i];
        if (ti > t_max)
        {
            t_max = ti;
        }
    }
    return t_max;
}
void MyInterestingMoves::map_right_arm_joint_indices(std::vector<std::string> joint_names)
{
    std::vector<std::string> rt_limb_jnt_names;

    right_arm_joint_indices_.clear();
    int index;
    int n_jnts = joint_names.size();
    cout << "num jnt names = " << n_jnts << endl;
    std::string j_name;
    std::string j_s0_name("right_s0");
    rt_limb_jnt_names.push_back(j_s0_name);
    std::string j_s1_name("right_s1");
    rt_limb_jnt_names.push_back(j_s1_name);
    std::string j_e0_name("right_e0");
    rt_limb_jnt_names.push_back(j_e0_name);
    std::string j_e1_name("right_e1");
    rt_limb_jnt_names.push_back(j_e1_name);
    std::string j_w0_name("right_w0");
    rt_limb_jnt_names.push_back(j_w0_name);
    std::string j_w1_name("right_w1");
    rt_limb_jnt_names.push_back(j_w1_name);
    std::string j_w2_name("right_w2");
    rt_limb_jnt_names.push_back(j_w2_name);

    for (int j = 0; j < 7; j++)
    {
        j_name = rt_limb_jnt_names[j];
        for (int i = 0; i < n_jnts; i++)
        {
             if (j_name.compare(joint_names[i]) == 0)
             {
                 index = i;
                 right_arm_joint_indices_.push_back(index);
                 break;
             }
         }
    }
    cout << "indices of right-arm joints: " << endl;
    for (int i = 0; i < 7; i++)
    {
        cout << right_arm_joint_indices_[i] << ", ";
    }
    cout << endl;
}

void MyInterestingMoves::stuffTrajectory(std::vector<Eigen::VectorXd> qvecs,
        trajectory_msgs::JointTrajectory &new_trajectory, double &final_time)
{
    new_trajectory.points.clear();  // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");
    new_trajectory.header.stamp = ros::Time::now();

    int joints_num = new_trajectory.joint_names.size();

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.clear();

    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = 0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout << "stuff_traj: start pt = " << q_start.transpose() << endl;

    trajectory_point.time_from_start = ros::Duration(net_time);

    for (int i = 0; i < joints_num; i++) {  // pre-sizes positions vector, so can access w/ indices later
        trajectory_point.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point);  // first point of the trajectory
    // add the rest of the points from qvecs

    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout << "dqvec: " << dqvec.transpose() << endl;
        del_time = transitionTime(dqvec);
        if (del_time < dt_traj)
        {
            del_time = dt_traj;
        }
        cout << "stuff_traj: next pt = " << q_end.transpose() << endl;
        net_time += del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f", iq, del_time, net_time);
        for (int i = 0 ; i < joints_num; i++)
        {
            trajectory_point.positions[i] = q_end[i];
        }
        trajectory_point.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point);
    }
    final_time = net_time;
}
