// minimal_commander node: 
// This is node is used to PUBLISH the resulting output to the topic 
// "vel_cmd" with data type std_msgs::Float64. The velocity profile should be SINE function
// with the specified amplitude and frequency. The
// Alternatively, you can have your commander subscribe to additional topics for 
// amplitude and frequency then use: rostopic pub to change these values without having to edit and recompile).
#include<ros/ros.h> 
#include<std_msgs/Float64.h>
#include<math.h>

#define PI 3.14159265

std_msgs::Float64 g_vel_cmd;
std_msgs::Float64 g_amp;
std_msgs::Float64 g_fre;

void myCallbackAmplitude(const std_msgs::Float64& message_holder) {
    // check for data on topic "amplitude" 
    ROS_INFO("received amplitude value is: %f", message_holder.data);
    g_amp.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

void myCallbackFrequency(const std_msgs::Float64& message_holder) {
    // check for data on topic "frequency" 
    ROS_INFO("received frequency command value is: %f", message_holder.data);
    g_fre.data = message_holder.data; // post the received data in a global var for access by 
    // main prog. 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_commander" 
    ros::NodeHandle nh; // node handle 
    ros::Subscriber my_subscriber_object1 = nh.subscribe("amplitude", 1, myCallbackAmplitude);
    ros::Subscriber my_subscriber_object2 = nh.subscribe("frequency", 1, myCallbackFrequency);
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);
    double dt = 0.1; //100ms integration time step 
    double sample_rate = 1 / dt; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 
    g_vel_cmd.data = 0.0; // init velocity command to zero 
    g_amp.data = 0.0;
    g_fre.data = 0.0;
 
 	int i = 0;
    while (ros::ok()) {
    	i = i + 1;
    	dt = i * dt;
        ROS_INFO("iteration time = %d", i);
    	// the velocity profile is a sine function: v = Asin(2pift)
        g_vel_cmd.data = g_amp.data * sin(2 * PI * g_fre.data * dt);
        my_publisher_object.publish(g_vel_cmd); // publish velocity command 
        ROS_INFO("velocity command = %f", g_vel_cmd.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
} 