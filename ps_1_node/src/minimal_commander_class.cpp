// This is a class based version of minimal_commander_class.cpp
// and action service is incoporated in this source file, a substitude
// service.
#include <ros/ros.h> 
#include <std_msgs/Float64.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <minimal_action_service/minimal_action_msgAction.h>
#include <iostream>
#include <string>

#define PI 3.14159265

class MinimalCommanderClass
{
public:
    MinimalCommanderClass(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    ~MinimalCommanderClass();
    // void executeActionCB(const actionlib::SimpleActionServer<minimal_action_service/minimal_action_msgAction>::GoalConstPtr& goal);
    void velProfileGen(); // this is a member function which is used to generate velocity profile and then publish it to the topic "vel_cmd"
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Publisher  minimal_commander_pub_;
    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in header s<minimal_action_service/minimal_action_msgAction.h>
    // the type "minimal_action_msgAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<minimal_action_service::minimal_action_msgAction> as_;
    
    // here are some message types to communicate with our client(s)
    minimal_action_service::minimal_action_msgGoal goal_; // goal message, received from client
    minimal_action_service::minimal_action_msgResult result_; // put results here, to be sent back to the client when done w/ goal
    minimal_action_service::minimal_action_msgFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    
    std_msgs::Float64 vel_cmd_;
    std_msgs::Float64 amplitute_;
    std_msgs::Float64 frequency_; // example member variable: better than using globals; 
    // convenient way to pass data from a subscriber to other member functions

    double dt_; //10ms integration time step 
    double sample_rate_; // compute the corresponding update frequency 
    double time_;

    ros::Rate* naptime_pointer_;
    // member methods as well:
    void initializePublishers();
    //prototype for action service callback function, the agrument is a pointer to a goal message
    void executeActionCB(const actionlib::SimpleActionServer<minimal_action_service/minimal_action_msgAction>::GoalConstPtr& goal);
    
};

MinimalCommanderClass::MinimalCommanderClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
                    ,as_(nh_, "minimal_action", boost::bind(&MinimalCommanderClass::executeActionCB, this, _1),false)  //, naptime_(rate)
                    // we specify that the new action server should utilize a function of our own design within its behavior. 
                    // This function we wish for the action-server to use is called executeActionCB (our own name choice).
{ // constructor
    ROS_INFO("in class constructor of MinimalCommanderClass");
    // package up the messy work of creating subscribers; do this overhead in constructor
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running

    initializePublishers();
    //initialize variables here, as needed
    vel_cmd_.data = 0.0; 
    amplitute_.data = 0.0;
    frequency_.data = 0.0; // example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    dt_ = 0.01; //10ms integration time step
    sample_rate_ = 10.0 ;// compute the corresponding update frequency
    time_ = 0.0;
    num_count_ = 0;
    naptime_pointer_ = new ros::Rate (sample_rate_);

    // can also do tests/waits to make sure all required services, topics, etc are alive
}

MinimalCommanderClass::~MinimalCommanderClass() {
    delete naptime_pointer_;
}

void MinimalCommanderClass::velProfileGen() {
    while(ros::ok()) {
        vel_cmd_.data = amplitute_.data * sin(2 * PI * frequency_.data * time_);
        time_+= dt_;
        ROS_INFO("Time = %f", time_);
        minimal_commander_pub_.publish(vel_cmd_); // publish velocity command 
        ROS_INFO("velocity command = %f", vel_cmd_.data);
        ROS_INFO("I am running");
        ros::spinOnce(); //allow data update from callback; 
        naptime_pointer_ -> sleep(); // wait for remainder of specified period;
    }   
}


//member helper function to set up publishers;
void MinimalCommanderClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_commander_pub_ = nh_.advertise<std_msgs::Float64>("vel_cmd", 1, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

//member function implementation for a action service callback function
bool MinimalCommanderClass::executeActionCB(const actionlib::SimpleActionServer<minimal_action_service/minimal_action_msgAction>::GoalConstPtr& goal) {
    
    amplitude_date = goal.amplitute;
    

    if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander_class"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_commander" 
    ros::NodeHandle nh; // node handle 
    
    //double ros_rate = 10.0;
    MinimalCommanderClass minimalCommanderClass(&nh);

    minimalCommanderClass.velProfileGen();
    return 0; // should never get here, unless roscore dies 
}
