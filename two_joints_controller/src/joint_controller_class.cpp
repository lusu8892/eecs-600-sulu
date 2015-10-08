#include "joint_controller.h"
// #include <two_joints_controller/src/joint_controller.h>

JointController::JointController(ros::NodeHandle* nodehandle, std::string joint_number):nh_(*nodehandle), joint_num_(joint_number)
{
    ROS_INFO("in class constructor of JointController");

    service_ready_ = false;
    half_sec_ = new ros::Duration(0.5);
    
 	checkService();
 	initializeServices();
	initializePublishers();
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor

	q1_ = 0.0;
	q1dot_ = 0.0;
    dt_ = 0.01;
    q1_err_ = 0.0;
    Kp_ = 0.0;
    Kv_ = 0.0;
    trq_cmd_ = 0.0;
    duration_ = new ros::Duration(dt_);
    rate_timer_ = new ros::Rate(1/dt_);

    effort_cmd_srv_msg_.request.joint_name = joint_num_;
    effort_cmd_srv_msg_.request.effort = 0.0;
    effort_cmd_srv_msg_.request.duration = *duration_;

    get_joint_state_srv_msg_.request.joint_name = joint_num_;

    joint_state_msg_.header.stamp = ros::Time::now();
	joint_state_msg_.name.push_back(joint_num_);
    joint_state_msg_.position.push_back(0.0);
    joint_state_msg_.velocity.push_back(0.0);

    controller();

}
JointController::~JointController()
{
	delete half_sec_;
	delete duration_;
}

void JointController::checkService()
{
	while (!service_ready_) {
      service_ready_ = ros::service::exists("/gazebo/apply_joint_effort",true);
      ROS_INFO("waiting for apply_joint_effort service");
      half_sec_ -> sleep();
    }
    ROS_INFO("apply_joint_effort service exists");

    service_ready_ = false;
    while (!service_ready_) {
      service_ready_ = ros::service::exists("/gazebo/get_joint_properties",true);
      ROS_INFO("waiting for /gazebo/get_joint_properties service");
      half_sec_ -> sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");

}

void JointController::initializeServices()
{
	ros::ServiceClient set_trq_client_ = 
       nh_.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    ros::ServiceClient get_jnt_state_client_ = 
       nh_.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
}

void JointController::initializePublishers()
{
	ros::Publisher torque_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_trq", 1, true); 
    ros::Publisher velocity_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_vel", 1, true);     
    ros::Publisher position_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_pos", 1, true);  
    ros::Publisher joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_num_+"_states", 1); 
}

void JointController::initializeSubscribers()
{
    ros::Subscriber pos_cmd_sub_ = nh_.subscribe(joint_num_+"_pos_cmd",1,&JointController::posCmdCB,this); 
}

void JointController::controller()
{
	while(ros::ok()) {    
        get_jnt_state_client_.call(get_joint_state_srv_msg_);
        q1_ = get_joint_state_srv_msg_.response.position[0];
        q1_msg_.data = q1_;
        position_pub_.publish(q1_msg_);
        
        q1dot_ = get_joint_state_srv_msg_.response.rate[0];
        q1dot_msg_.data = q1dot_;
        velocity_pub_.publish(q1dot_msg_);

		joint_state_msg_.header.stamp = ros::Time::now();
        joint_state_msg_.position[0] = q1_; 
        joint_state_msg_.velocity[0] = q1dot_;

		joint_state_pub_.publish(joint_state_msg_);
        
        //ROS_INFO("q1 = %f;  q1dot = %f",q1,q1dot);
        //watch for periodicity
        q1_err_= pos_cmd_-q1_;
        if (q1_err_>M_PI) {
            q1_err_ -= 2*M_PI;
        }
        if (q1_err_< -M_PI) {
            q1_err_ += 2*M_PI;
        }        
            
        trq_cmd_ = Kp_*(q1_err_)-Kv_*q1dot_;
        //trq_cmd = sat(trq_cmd, 10.0); //saturate at 1 N-m
        trq_msg_.data = trq_cmd_;
        torque_pub_.publish(trq_msg_);
        // send torque command to Gazebo
        effort_cmd_srv_msg_.request.effort = trq_cmd_;
        set_trq_client_.call(effort_cmd_srv_msg_);
        //make sure service call was successful
        result_ = effort_cmd_srv_msg_.response.success;
        if (!result_)
            ROS_WARN("service call to apply_joint_effort failed!");
        ros::spinOnce();
		rate_timer_->sleep();
    }
}


void JointController::posCmdCB(const std_msgs::Float64& pos_cmd_msg)
{
	ROS_INFO("received value of pos_cmd is: %f",pos_cmd_msg.data); 
  	pos_cmd_ = pos_cmd_msg.data;
}
// double JointController::sat(double val, double sat_val)
// {

// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_controller");
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;

    // user can instantiate any number of joints
    JointController jointController1(&nh1, "joint1");
    JointController jointController2(&nh2, "joint2");

    return 0;
}