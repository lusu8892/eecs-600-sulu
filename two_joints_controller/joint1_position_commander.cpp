#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

double PI = 3.1415926;

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint1_pos_commander");
    ros::NodeHandle nh;
    // instaniating a publisher
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("joint1_pos_cmd", 1);

    std_msgs::Float64 jnt1_position;

    double amplitude = 10.0;
    double frequency = 5.0;
    double dt = 0.1;
    double time = 0.0;
   
    ros::Rate naptime(10.0);

    while (ros::ok()) 
    {
        jnt1_position.data = amplitude * sin( frequency* PI / 180* time);
        dt++;
        ROS_INFO("joint1_position = %f", jnt1_position.data);
        pub.publish(jnt1_position);
	    naptime.sleep();
    }
}

