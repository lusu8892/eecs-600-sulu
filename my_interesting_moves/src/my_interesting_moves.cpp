// scopyright

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

    qdot_max_vec_ << q0dotmax, q1dotmax, q2dotmax, q3dotmax, q4dotmax, q5dotmax, q6dotmax;
    qdot_max_vec_ *= SPEED_SCALE_FACTOR;
}

void stuffTrajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory);
{
    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    // int joints_num = new_trajectory.joint_names.size();

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.clear();

    new_trajectory.header.stamp = ros::Time::now();


}

void MyInterestingMoves::rightArmZeroConfig(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory, double &final_time)
{
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    stuffTrajectory(new_trajectory, trajectory_point);
    int joints_num = new_trajectory.joint_names.size();

    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = 0.0;
    double final_time;
    q_start = qvecs[0];
    q_end = qvecs[0];   
    cout<<"stuff_traj: start pt = "<< q_start.transpose() <<endl; 

    //trajectory_point.positions = qvecs[0];
    trajectory_point.time_from_start = ros::Duration(net_time); 
    for (int i = 0; i < 7; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point); // first point of the trajectory
    //add the rest of the points from qvecs

    for (int iq = 1; iq < qvecs.size(); iq++) {
        // q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transitionTime(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);
        for (int i = 0 ; i < 7; i++)
        {//copy over the joint-command values
            trajectory_point.positions[i]=q_end[i];
        }
        //trajectory_point.positions = q_end;
        trajectory_point.time_from_start = ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point);
    }
    final_time = net_time;
}

void MyInterestingMoves::rightArmSinMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory, double &final_time)
{
    double omega = 1.0; //rad/sec
    double amp = 0.5; //radians
    double start_angle = amp;
    double final_phase = 4 * PI; // radians--two periods
    //dt: break up trajectory into incremental commands this far apart in time
    // below, we will randomize this dt, just to illustrate that trajectories do not have to have fixed time steps
    double dt = 0.1;
    double phase = 0.0; //radians
    double time_from_start = 0.0; // seconds
    double q_des,qdot_des; //radians, radians/sec
    Eigen::VectorXd freeze_joint_val = qvecs[0]; // remember the joint which will be hold from moving
    Vectorq7x1 q_vec_des;

    // "right_s0", "right_e0", "right_w0", "right_w2" joints will will keep the same angle as last movement
    for (int 0; i < 7; i += 2)
        {
            q_vec_des[i] = freeze_joint_val[i]; // 
        }
    //"phase" is a convenient variable = omega*time
    for (phase=0.0; phase < final_phase; phase += omega * dt) {
        q_des = start_angle + amp*sin(phase); //here we make up a desired trajectory shape: q_des(t)

        // "right_s1", "right_e1", "right_w1" joints will move
        for (int i = 1; i < 7 - 1; i += 2)
        {
            q_vec_des[i] = q_des;
        }
        qvecs.push_back(q_des);
    }

    stuffTrajectory(qvecs, new_trajectory, trajectory_point);

}

void MyInterestingMoves::rightArmSaluteMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory, double &final_time)
{

}

void MyInterestingMoves::rightArmSaluteMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory, double &final_time)
{

}

void MyInterestingMoves::rightArmComeOnMove(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &trajectory, double &final_time)
{

}

Vectorq7x1 MyInterestingMoves::getQvecRigthArm()
{
    return q_vec_right_arm_;
}

sensor_msgs::JointState MyInterestingMoves::getJointStates()
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
    for (int i = 0; i < 7; i++)
    {
        // should do this better; manually remap from joint_states indices to right-arm joint angles
        // q_vec_right_arm_[i] = js_msg.position[right_arm_joint_indices[i]]; //w2
        q_vec_right_arm_[i] = js_msg.position[i]; //w2
    }
    cout<<"CB: q_vec_right_arm: "<<q_vec_right_arm_.transpose()<<endl;
}

double MyInterestingMoves::transitionTime(Eigen::VectorXd dqvec)
{
    double t_max = fabs(dqvec[0]) / qdot_max_vec_[0];
    cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i=1; i < dqvec.size(); i++)
    {
        ti = fabs(dqvec[i]) / qdot_max_vec_[i];
        if (ti > t_max)
        {
            t_max= ti;
        }
    }
    return t_max;
}