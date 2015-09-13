// minimal_commander node: 
// This is node is used to PUBLISH the resulting output to the topic 
// "vel_cmd" with data type std_msgs::Float64. The velocity profile should be SINE function
// with the specified amplitude and frequency. The
// Alternatively, you can have your commander subscribe to additional topics for 
// amplitude and frequency then use: rostopic pub to change these values without having to edit and recompile).
#include<ros/ros.h> 
#include<std_msgs/Float64.h>

std_msgs::Float64 g_vel_cmd;
std_msgs::Float64 g_amp;
std_msgs::Float64 g_fre;

void myCallbackAmplitude(const std_msgs::Float64& message_holder) {
    // check for data on topic "velocity" 
    ROS_INFO("received amplitude value is: %f", message_holder.data);
    g_amp.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

void myCallbackFrequency(const std_msgs::Float64& message_holder) {
    // check for data on topic "vel_cmd" 
    ROS_INFO("received frequency command value is: %f", message_holder.data);
    g_fre.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_controller" 
    ros::NodeHandle nh; // node handle 
    //create 2 subscribers: one for state sensing (velocity) and one for velocity commands 
    ros::Subscriber my_subscriber_object1 = nh.subscribe("amplitude", 1, myCallbackAmplitude);
    ros::Subscriber my_subscriber_object2 = nh.subscribe("frequency", 1, myCallbackFrequency);
    //publish a velocity command computed by this controller; 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("g_vel_cmd", 1);
    double dt = 0.1; //100ms integration time step 
    double sample_rate = 1.0 / dt; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 
    g_vel_cmd.data = 0.0; // init velocity command to zero 

    while (ros::ok()) {
        vel_err = g_vel_cmd.data - g_velocity.data; // compute error btwn desired and actual 
        //velocities 
        g_force.data = Kv*vel_err; //proportional-only velocity-error feedback defines commanded 
        //force 
        my_publisher_object.publish(g_force); // publish the control effort computed by this 
        //controller 
        ROS_INFO("force command = %f", g_force.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
} 