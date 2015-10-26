// baxte_traj_action_client: 
// sulu, Oct, 2015
// uses traj_interpolator_as to commander baxter to execute four interesting movement

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_interesting_moves/my_interesting_moves.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_trajectory_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_trajectory_streamer in your package.xml,
// and include the header file below
#include <baxter_trajectory_streamer/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector
#define PI = 3.141592653
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "baxter_trajectory_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        
    int g_count = 0;
    int ans;
    double final_time;
    string answer;
    Vectorq7x1 q_pre_pose;
    //q_in << 0, 0, 0, 0, 0, 0, 0;  
    q_pre_pose<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179;
    // q_pre_pose << 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    //cin>>ans;

    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory
    cout<<"instantiating a traj streamer"<<endl; // enter 1:";
    //cin>>ans;
    MyInsterestingMoves myInsterestingMoves(&nh); //instantiate a myInsterestingMoves object and pass in pointer to nodehandle for constructor to use
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout << "do you want baxter to go back to zero configuration first? (Y/N)"
    cin >> answer;
    if (answer.compare("Y") == 0 || answer.compare("y") == 0)
    {
        cout << endl;
        cout<<"getting current right-arm pose:"<<endl;
        q_vec_right_arm =  myInsterestingMoves.getQvecRigthArm();
        cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
        q_in_vecxd = q_vec_right_arm; // start from here;
        des_path.push_back(q_in_vecxd); //put all zeros here
        cout << "stuffing right arm to its zero configuration: " << endl;

        myInsterestingMoves.rightArmZeroConfig(des_path, des_trajectory, final_time);

        // q_in_vecxd = q_pre_pose; // conversion; not sure why I needed to do this...but des_path.push_back(q_in_vecxd) likes it
        // des_path.push_back(q_in_vecxd); //twice, to define a trajectory

        // // here is a "goal" object compatible with the server, as defined in example_action_server/action
        baxter_trajectory_streamer::trajGoal goal; 
        // does this work?  copy traj to goal:
        goal.trajectory = des_trajectory;
        //cout<<"ready to connect to action server; enter 1: ";
        //cin>>ans;
        // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
        actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> action_client("trajActionServer", true);

        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running

        if (!server_exists) {
            ROS_WARN("could not connect to server; will wait forever");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        server_exists = action_client.waitForServer(); //wait forever 

        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        //while(true) {
        // stuff a goal message:
        g_count++;
        goal.traj_id = g_count; // this merely sequentially numbers the goals sent
        ROS_INFO("sending traj_id %d",g_count);
        // action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        // action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }
    }
    else
    {
        cout << "baxter will do the first interesting move at its current position" << endl;
    }

    // baxter doing right arm sine movement
    des_path.clear();
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  myInsterestingMoves.getQvecRigthArm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here
    // q_in_vecxd = q_pre_pose; // conversion; not sure why I needed to do this...but des_path.push_back(q_in_vecxd) likes it
    // des_path.push_back(q_in_vecxd); //twice, to define a trajectory

    cout << "stuffing right arm to execute sine movement: " << endl;
    myInsterestingMoves.rightArmSinMove(des_path, des_trajectory, final_time); //convert from vector of 7dof poses to trajectory message        
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    baxter_trajectory_streamer::trajGoal goal; 
    // does this work?  copy traj to goal:
    goal.trajectory = des_trajectory;
    //cout<<"ready to connect to action server; enter 1: ";
    //cin>>ans;
    // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> action_client("trajActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 
    
   
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    //while(true) {
    // stuff a goal message:
    g_count++;
    goal.traj_id = g_count; // this merely sequentially numbers the goals sent
    ROS_INFO("sending traj_id %d",g_count);
    // action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
    // action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result for goal number %d",g_count);
        return 0;
    }
    else {
        ROS_INFO("finished before timeout");
    }

    // baxter doing right arm salute movement
    des_path.clear();
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  myInsterestingMoves.get_qvec_right_arm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    cout << "stuffing right arm to execute salute movement: " << endl;
    myInsterestingMoves.rightArmSaluteMove(des_path, des_trajectory, final_time)
    goal.trajectory = des_trajectory;
    ROS_INFO("waiting for server: ");
    server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running

    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 
    ROS_INFO("connected to action server");  // if here, then we connected to the server;
    g_count++;
    goal.traj_id = g_count; // this merely sequentially numbers the goals sent
    ROS_INFO("sending traj_id %d",g_count);
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
    finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result for goal number %d",g_count);
        return 0;
    }
    else {
        ROS_INFO("finished before timeout");
    }

    // baxter doing right arm salute movement
    des_path.clear();
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  myInsterestingMoves.get_qvec_right_arm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    cout << "stuffing right arm to execute zigzag movement: " << endl;
    myInsterestingMoves.rightArmSaluteMove(des_path, des_trajectory, final_time)
    goal.trajectory = des_trajectory;
    ROS_INFO("waiting for server: ");
    server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running

    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 
    ROS_INFO("connected to action server");  // if here, then we connected to the server;
    g_count++;
    goal.traj_id = g_count; // this merely sequentially numbers the goals sent
    ROS_INFO("sending traj_id %d",g_count);
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
    finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result for goal number %d",g_count);
        return 0;
    }
    else {
        ROS_INFO("finished before timeout");
    }

    // baxter doing right arm come movement
    des_path.clear();
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  myInsterestingMoves.get_qvec_right_arm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    cout << "stuffing right arm to execute come on movement: " << endl;
    myInsterestingMoves.rightArmSaluteMove(des_path, des_trajectory, final_time)
    goal.trajectory = des_trajectory;
    ROS_INFO("waiting for server: ");
    server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running

    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 
    ROS_INFO("connected to action server");  // if here, then we connected to the server;
    g_count++;
    goal.traj_id = g_count; // this merely sequentially numbers the goals sent
    ROS_INFO("sending traj_id %d",g_count);
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
    finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result for goal number %d",g_count);
        return 0;
    }
    else {
        ROS_INFO("finished before timeout");
    }

    return 0;
}

