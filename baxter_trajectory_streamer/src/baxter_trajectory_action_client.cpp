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
    MyInterestingMoves myInterestingMoves(&nh); //instantiate a myInterestingMoves object and pass in pointer to nodehandle for constructor to use
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }

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
    bool finished_before_timeout;
    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 

    ROS_INFO("connected to action server");  // if here, then we connected to the server;
    cout << "do you want baxter to go back to zero configuration first? (Y/N)";
    cin >> answer;
    if (answer.compare("Y") == 0 || answer.compare("y") == 0)
    {
        cout << endl;
        cout<<"getting current right-arm pose:"<<endl;
        q_vec_right_arm =  myInterestingMoves.getQvecRigthArm();
        cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
        q_in_vecxd = q_vec_right_arm; // start from here;
        des_path.push_back(q_in_vecxd); //put all zeros here
        cout << "stuffing right arm to its zero configuration: " << endl;
        myInterestingMoves.rightArmZeroConfig(des_path, des_trajectory, final_time);

        goal.trajectory = des_trajectory;
        g_count++;
        goal.traj_id = g_count; // this merely sequentially numbers the goals sent
        ROS_INFO("sending traj_id %d",g_count);
        // action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        // action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
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

    // baxter doing right arm salute movement
    des_path.clear();
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  myInterestingMoves.getQvecRigthArm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    cout << "stuffing right arm to execute salute movement: " << endl;
    myInterestingMoves.rightArmSaluteMove(des_path, des_trajectory, final_time);
    goal.trajectory = des_trajectory;

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
    q_vec_right_arm =  myInterestingMoves.getQvecRigthArm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    cout << "stuffing right arm to execute zigzag movement: " << endl;
    myInterestingMoves.rightArmZigzagMove(des_path, des_trajectory, final_time);
    goal.trajectory = des_trajectory;

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
    q_vec_right_arm =  myInterestingMoves.getQvecRigthArm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    cout << "stuffing right arm to execute come on movement: " << endl;
    myInterestingMoves.rightArmComeOnMove(des_path, des_trajectory, final_time);
    goal.trajectory = des_trajectory;

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

// // baxte_traj_action_client: 
// // sulu, Oct, 2015
// // uses traj_interpolator_as to commander baxter to execute four interesting movement

// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include <my_interesting_moves/my_interesting_moves.h>
// //this #include refers to the new "action" message defined for this package
// // the action message can be found in: .../baxter_trajectory_streamer/action/traj.action
// // automated header generation creates multiple headers for message I/O
// // these are referred to by the root name (traj) and appended name (Action)
// // If you write a new client of the server in this package, you will need to include baxter_trajectory_streamer in your package.xml,
// // and include the header file below
// #include <baxter_trajectory_streamer/trajAction.h>

// using std::cout;
// using std::endl;
// using std::string;

// #define VECTOR_DIM 7 // e.g., a 7-dof vector
// #define PI = 3.141592653

// class BaxterTrajActionClient
// {
// public:
//     BaxterTrajActionClient(ros::NodeHandle* nodehandle);
//     ~BaxterTrajActionClient();
//     void callInterestingMove(void (MyInterestingMoves::*func_pointer)(std::vector<Eigen::VectorXd>, trajectory_msgs::JointTrajectory&, double&));
// private:
//     ros::NodeHandle nh_; //standard ros node handle
//     int g_count_;
//     // int ans_;
//     double final_time_;
//     string answer_;
//     Vectorq7x1 q_pre_pose_;
//     Eigen::VectorXd q_in_vecxd_; // variable used to convert Vectorq7x1 data type to VectorXd

//     Vectorq7x1 q_vec_right_arm_;

//     std::vector<Eigen::VectorXd> des_path_;

//     trajectory_msgs::JointTrajectory des_trajectory_; // an empty trajectory

//     MyInterestingMoves myInterestingMoves;

//     // stuff used to make action service(client)
//     actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> action_client_;
//     // here is a "goal" object compatible with the server, as defined in example_action_server/action
//     baxter_trajectory_streamer::trajGoal goal_;

//     bool server_exists_;
//     bool finished_before_timeout_;
//     void doneCb(const actionlib::SimpleClientGoalState& state,
//         const baxter_trajectory_streamer::trajResultConstPtr& result);
// };

// BaxterTrajActionClient::BaxterTrajActionClient(ros::NodeHandle* nodehandle)
//         :myInterestingMoves(nodehandle), action_client_("minimal_action", true)
// {
//     // attempt to connect to the server:
//     ROS_INFO("waiting for server: ");
//     bool server_exists = action_client_.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
//     // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running

//     if (!server_exists)
//     {
//         ROS_WARN("could not connect to server; will wait forever");
//         return; // bail out; optionally, could print a warning message and retry
//     }
//     server_exists = action_client_.waitForServer(); //wait forever 

//     ROS_INFO("connected to action server");  // if here, then we connected to the server;
// }

// void BaxterTrajActionClient::callInterestingMove(void (MyInterestingMoves::*func_pointer)(std::vector<Eigen::VectorXd>, trajectory_msgs::JointTrajectory&, double&))
// {
//         des_path_.clear();
//         cout<<"getting current right-arm pose:"<<endl;
//         q_vec_right_arm_ =  myInterestingMoves.getQvecRigthArm();
//         cout<<"r_arm state:"<<q_vec_right_arm_.transpose()<<endl;
//         q_in_vecxd_ = q_vec_right_arm_; // start from here;
//         des_path_.push_back(q_in_vecxd_); //put all zeros here
//         cout << "stuffing right arm" << endl;

//         myInterestingMoves.rightArmSinMove(des_path_, des_trajectory_, final_time_);
//         // myInterestingMoves.(*func_pointer)(des_path_, des_trajectory_, final_time_);
//         MyInterestingMoves::->*func_pointer(des_path_, des_trajectory_, final_time_);
//         // void* (std::vector<Eigen::VectorXd>, trajectory_msgs::JointTrajectory&, double&)
//         goal_.trajectory = des_trajectory_;
//         g_count_++;
//         goal_.traj_id = g_count_; // this merely sequentially numbers the goals sent
//         ROS_INFO("sending traj_id %d",g_count_);
//         // action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
//         action_client_.sendGoal(goal_,boost::bind(&BaxterTrajActionClient::doneCb, this, _1, _2)); // we could also name additional callback functions here, if desired
//         // action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
//         finished_before_timeout_ = action_client_.waitForResult(ros::Duration(final_time_ + 2.0));
//         //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
//         if (!finished_before_timeout_)
//         {
//             ROS_WARN("giving up waiting on result for goal number %d",g_count_);
//             return;
//         }
//         else
//         {
//             ROS_INFO("finished before timeout");
//         }
// }

// void BaxterTrajActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
//         const baxter_trajectory_streamer::trajResultConstPtr& result)
// {
//     ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
//     ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "baxter_trajectory_action_client_node"); // name this node
//     ros::NodeHandle nh; //standard ros node handle
//     // warm up the joint-state callbacks;
//     cout<<"warming up callbacks..."<<endl;
//     for (int i=0;i<100;i++)
//     {
//         ros::spinOnce();
//         //cout<<"spin "<<i<<endl;
//         ros::Duration(0.01).sleep();
//     }
//     // instantiate a BaxterTrajActionClient object baxterTrajActionClien
//     BaxterTrajActionClient baxterTrajActionClient(&nh);


//     baxterTrajActionClient.callInterestingMove(MyInterestingMoves::rightArmSinMove);

//     return 0;
// }

