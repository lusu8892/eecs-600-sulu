// This is a class based version of minimal_commander.cpp
#include <ros/ros.h> 
#include <std_msgs/Float64.h>
#include <math.h>
#include <minimal_service/minimal_server_msg.h>
#include <iostream>
#include <string>

#define PI 3.14159265

class MinimalCommanderClass
{
public:
    MinimalCommanderClass(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    ~MinimalCommanderClass();
    void velProfileGen(); // this is a member function which is used to generate velocity profile and then publish it to the topic "vel_cmd"
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::ServiceServer minimal_service_;
    ros::Publisher  minimal_commander_pub_;
    
    std_msgs::Float64 vel_cmd_;
    std_msgs::Float64 amplitute_;
    std_msgs::Float64 frequency_; // example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions

    double dt_; //10ms integration time step 
    double sample_rate_; // compute the corresponding update frequency 
    double time_;

    int num_count_;
    // ros::Rate naptime_;
    ros::Rate* naptime_pointer_;
    // member methods as well:
    void initializePublishers();
    void initializeServices();
    
    //prototype for callback for example service
    bool serviceCallback(minimal_service::minimal_server_msgRequest& request, minimal_service::minimal_server_msgResponse& response);
    
};

MinimalCommanderClass::MinimalCommanderClass(ros::NodeHandle* nodehandle):nh_(*nodehandle) //, naptime_(rate)
{ // constructor
    ROS_INFO("in class constructor of MinimalCommanderClass");
    // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
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

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void MinimalCommanderClass::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh_.advertiseService("specify_value", &MinimalCommanderClass::serviceCallback, this);  
    // add more services here, as needed
}

//member helper function to set up publishers;
void MinimalCommanderClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_commander_pub_ = nh_.advertise<std_msgs::Float64>("vel_cmd", 1, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

//member function implementation for a service callback function
bool MinimalCommanderClass::serviceCallback(minimal_service::minimal_server_msgRequest& request, minimal_service::minimal_server_msgResponse& response) {
    ROS_INFO("callback activated");
    amplitute_.data = request.amplitude;
    frequency_.data = request.frequency;

    // fill in the response so that we can check if response got by in client node
    response.amplitude = request.amplitude;
    response.frequency = request.frequency;
    ++num_count_;
    ROS_INFO("times callback invoked = %d", num_count_);
    return true;
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
