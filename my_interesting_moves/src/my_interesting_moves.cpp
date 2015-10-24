// copyright

#include <my_interesting_moves/my_interesting_moves.h>

MyInterestingMoves::MyInsterestingMoves(ros::NodeHandle* nodehandle): nh_(&nodehandle)
{
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    // define the joint angles 0-6 to be right arm, from shoulder out to wrist;
    // right_cmd_.names.push_back("right_s0");
    // right_cmd_.names.push_back("right_s1");
    // right_cmd_.names.push_back("right_e0");
    // right_cmd_.names.push_back("right_e1");
    // right_cmd_.names.push_back("right_w0");
    // right_cmd_.names.push_back("right_w1");
    // right_cmd_.names.push_back("right_w2");

    qdot_max_vec << q0dotmax, q1dotmax, q2dotmax, q3dotmax, q4dotmax, q5dotmax, q6dotmax;
    qdot_max_vec *= SPEED_SCALE_FACTOR;
}

void MyInterestingMoves::rightArmSinMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory)
{
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    trajectory_point.positions.clear(); 

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];   
     cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 

    //trajectory_point1.positions = qvecs[0];
 
    trajectory_point1.time_from_start =    ros::Duration(net_time); 
    for (int i=0;i<7;i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    } 
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs
   

    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //copy over the joint-command values
            trajectory_point1.positions[i]=q_end[i];
        }   
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start =    ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }
}

void MyInterestingMoves::rightArmSaluteMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory)
{

}

void MyInterestingMoves::rightArmSaluteMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory)
{

}

void MyInterestingMoves::rightArmComeOnMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory)
{

}

// Vectorq7x1 MyInterestingMoves::getQvecRigthArm()
// {

// }

sensor_msgs::JointState MyInterestingMoves::get_joint_states()
{
    return joint_states_;
}

vodi MyInterestingMoves::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    joint_state_sub_ = nh_.subscribe("robot/joint_states", 1, &MyInterestingMoves::jointStatesCb, this);
    // add more subscribers here, as needed
}

//NOTE: this is not separately threaded.  this callback only responds with the parent node allows a ros spin.
void MyInterestingMoves::jointStatesCb(const sensor_msgs::JointState& js_msg) {
    joint_states_ = js_msg; // copy this to member var
    // if (right_arm_joint_indices.size()<1) {
    //    //g_all_jnt_names = js_msg.name;
    //    map_right_arm_joint_indices(js_msg.name);
    // }
    // // copy right-arm angles to global vec
    // for (int i=0;i<7;i++)
    // {
    //     // should do this better; manually remap from joint_states indices to right-arm joint angles
    //     q_vec_right_arm_[i] = js_msg.position[right_arm_joint_indices[i]]; //w2         
    // }
    //cout<<"CB: q_vec_right_arm: "<<q_vec_right_arm_.transpose()<<endl;
}

double MyInterestingMoves::transitionTime(Vectorq7x1 dqvec)
{
    double t_max = fabs(dqvec[0])/qdot_max_vec[0];
    cout<<"qdot max: "<<qdot_max_vec.transpose()<<endl;
    double ti;
    for (int i=1; i < dqvec.size(); i++)
    {
        ti = fabs(dqvec[i]) / qdot_max_vec[i];
        if (ti > t_max)
        {
            t_max= ti;
        }
    }
    return t_max;
}