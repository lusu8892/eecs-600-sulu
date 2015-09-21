// minimal_commander node: 
// This is node is used to PUBLISH the resulting output to the topic 
// "vel_cmd" with data type std_msgs::Float64. The velocity profile should be SINE function
// with the specified amplitude and frequency. 
// In this minimal_commander node, a service that responds to a client by setting the amplitude and 
// frequency to the requested values has been included.
#include<ros/ros.h> 
#include<std_msgs/Float64.h>
#include<math.h>
#include<minimal_service/minimal_server_msg.h>
#include <iostream>
#include <string>

#define PI 3.14159265

std_msgs::Float64 g_vel_cmd;
std_msgs::Float64 g_amp;
std_msgs::Float64 g_fre;

bool callback(minimal_service::minimal_server_msgRequest& request, minimal_service::minimal_server_msgResponse& response)
{
    ROS_INFO("callback activated");
    g_amp.data = request.amplitude;
    g_fre.data = request.frequency;

    // fill in the response so that we can check if response got by in client node
    response.amplitude = request.amplitude;
    response.frequency = request.frequency;
  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_commander" 
    ros::NodeHandle nh; // node handle 
    
    ros::ServiceServer server = nh.advertiseService("specify_value",callback);

    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);
    double dt = 0.01; //10ms integration time step 
    double sample_rate = 10.0; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 
    g_vel_cmd.data = 0.0; // init velocity command to zero 
    g_amp.data = 0.0;
    g_fre.data = 0.0;
    double time = 0.0;

    while (ros::ok()) {
    	// the velocity profile is a sine function: v = Asin(2pift)
        g_vel_cmd.data = g_amp.data * sin(2 * PI * g_fre.data * time);
        time += dt;
        ROS_INFO("Time = %f", time);
        my_publisher_object.publish(g_vel_cmd); // publish velocity command 
        ROS_INFO("velocity command = %f", g_vel_cmd.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
} 
