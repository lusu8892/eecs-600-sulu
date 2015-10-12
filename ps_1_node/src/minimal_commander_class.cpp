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
    //prototype for action service callback function, the agrument is a pointer to a goal message
    void executeActionCB(const actionlib::SimpleActionServer<minimal_action_service::minimal_action_msgAction>::GoalConstPtr& goal);
    void velProfileGen(); // this is a member function which is used to generate velocity profile and then publish it to the topic "vel_cmd"
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Publisher  minimal_commander_pub_;
    // this class will own a "SimpleActionServer" called "action_server_".
    // it will communicate using messages defined in header s<minimal_action_service/minimal_action_msgAction.h>
    // the type "minimal_action_msgAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<minimal_action_service::minimal_action_msgAction> action_server_;
    
    // here are some message types to communicate with our client(s)
    minimal_action_service::minimal_action_msgGoal goal_; // goal message, received from client
    minimal_action_service::minimal_action_msgResult result_; // put results here, to be sent back to the client when done w/ goal
    minimal_action_service::minimal_action_msgFeedback feedback_; // not used in this example; 
    // would need to use: action_server_.publishFeedback(feedback_); to send incremental feedback to the client
    
    std_msgs::Float64 vel_cmd_;
    std_msgs::Float64 amplitude_;
    std_msgs::Float64 frequency_; 
    std_msgs::Float64 cycles_; // example member variable: better than using globals;
    // convenient way to pass data from a subscriber to other member functions

    int num_count_;
    double dt_; //10ms integration time step
    double p_phi_;
    double c_phi_;
    double sample_rate_; // compute the corresponding update frequency 
    double time_;
    double time_now_;
    ros::Rate* naptime_pointer_;
    // member methods as well:
    void initializePublishers();
    // prototype for action service callback function, the agrument is a pointer to a goal message
    // void velProfileGen(); // this is a member function which is used to generate velocity profile and then publish it to the topic "vel_cmd"
};

MinimalCommanderClass::MinimalCommanderClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
                    ,action_server_(nh_, "minimal_action", boost::bind(&MinimalCommanderClass::executeActionCB, this, _1), false)  //, naptime_(rate)
                    // we specify that the new action server should utilize a function of our own design within its behavior. 
                    // This function we wish for the action-server to use is called executeActionCB (our own name choice).
{ // constructor
    ROS_INFO("in class constructor of MinimalCommanderClass");
    // package up the messy work of creating subscribers; do this overhead in constructor
    // do any other desired initializations here...specific to your implementation

    action_server_.start(); //start the server running

    initializePublishers();
    //initialize variables here, as needed
    vel_cmd_.data = 0.0; 
    amplitude_.data = 0.0;
    frequency_.data = 0.0; // example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    dt_ = 0.01; //10ms integration time step
    p_phi_ = 0.0;
    c_phi_ = 0.0;
    sample_rate_ = 10.0 ;// compute the corresponding update frequency
    time_ = 0.0;
    num_count_ = 0;
    naptime_pointer_ = new ros::Rate (sample_rate_);
    // time_begin_ = ros::Time::now().toSec();
    // time_now_ = ros::Time::now().toSec();
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

MinimalCommanderClass::~MinimalCommanderClass() {
    delete naptime_pointer_;
}

//member function implementation for a action service callback function
void MinimalCommanderClass::executeActionCB(const actionlib::SimpleActionServer<minimal_action_service::minimal_action_msgAction>::GoalConstPtr& goal) {
    // getting specified goal from action client to member variable of Class MinimalCommanderClass
    amplitude_.data = goal -> amplitude_input;
    frequency_.data = goal -> frequency_input;
    // cycles_.data = goal -> cycles_input;

    // feeding the specifiec goal back to result_ that will be sent back to action service client
    result_.amplitude_output = amplitude_.data;
    result_.frequency_output = frequency_.data;
    result_.cycles_output = goal -> cycles_input;
    result_.outcome = "goal is in process";

    if (cycles_.data < goal -> cycles_input) {
        velProfileGen();
        ROS_INFO("Current phase = %f;", c_phi_);
        // action_server_.setSucceeded(result_); // tell the client we have given up on this goal; send the result message as well
        if (c_phi_ - p_phi_ >= 2*PI){
            p_phi_ = c_phi_;
            cycles_.data += 1.0;
        }
    }
    else {
        result_.amplitude_output = amplitude_.data;
        result_.frequency_output = frequency_.data;
        result_.cycles_output = cycles_.data;
        // amplitude_.data = 0.0;
        // frequency_.data = 0.0;
        // cycles_.data = 0.0;
        result_.outcome = "goal is completed, the amplitude, frequency, and cycles are set to zero";
        velProfileGen();
        // time_ = 0.0; // time_ is set to zero
        action_server_.setSucceeded(result_);
    }
    
    result_.cycles_output = cycles_.data;

    if (amplitude_.data == 0.0 && frequency_.data == 0.0) {
        cycles_.data = -1.0;
    }

    ROS_WARN("informing client of aborted goal");
    action_server_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
}


//member helper function to set up publishers;
void MinimalCommanderClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_commander_pub_ = nh_.advertise<std_msgs::Float64>("vel_cmd", 1, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

void MinimalCommanderClass::velProfileGen() {
    // while(ros::ok()) {
    vel_cmd_.data = amplitude_.data * sin(2 * PI * frequency_.data * time_);
    time_+= dt_;
    c_phi_ = 2 * PI * frequency_.data * time_;
    ROS_INFO("Time = %f", time_);
    minimal_commander_pub_.publish(vel_cmd_); // publish velocity command 
    ROS_INFO("velocity command = %f", vel_cmd_.data);
    ROS_INFO("I am running");
    // ros::spinOnce(); //allow data update from callback; 
    naptime_pointer_ -> sleep(); // wait for remainder of specified period;
    // }   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander_class"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_commander" 
    ros::NodeHandle nh; // node handle 
    
    //double ros_rate = 10.0;
    MinimalCommanderClass minimalCommanderClass(&nh);

    // minimalCommanderClass.velProfileGen();

    while(ros::ok()){
        minimalCommanderClass.velProfileGen();
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }
    return 0; // should never get here, unless roscore dies 
}
