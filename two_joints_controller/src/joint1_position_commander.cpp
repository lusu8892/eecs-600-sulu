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

    double omega = 1.0; //rad/sec
    double amp = 0.5; //radians
    double start_angle= amp;
    double phase = 0; // radians--two periods
    double q_des =0;
        
        //dt: break up trajectory into incremental commands this far apart in time
        // below, we will randomize this dt, just to illustrate that trajectories do not have to have fixed time steps
    double dt = 0.1; 
   
    ros::Rate naptime(10.0);

    while (ros::ok()) 
    {
        jnt1_position.data = start_angle + amp*sin(phase); //here we make up a desired trajectory shape: q_des(t)
        
        // ROS_INFO("phase = %f, t = %f",phase,time_from_start);               
        //specify arrival time for this point--in ROS "duration" format
        // trajectory_point.time_from_start = ros::Duration(time_from_start); //this converts from seconds to ros::Duration data type
        // //append this trajectory point to the vector of points in trajectory:
        // trajectory.points.push_back(trajectory_point);  
                // merely for illustration purposes, introduce a random time step; 
                // this shows that trajectory messages do not need a fixed time step
                // also, the dt values can be quite coarse in this example, for the purpose of illustrating the interpolation capability of the server
                dt = (rand() % 100 + 1)*0.01 ;     // rand() % 100 + 1 in the range 1 to 100, so dt is in the range from 0.01 to 1.0 sec
                phase+=omega*dt;
                pub.publish(jnt1_position);
                naptime.sleep();

    }
}


