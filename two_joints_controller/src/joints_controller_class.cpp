#include <two_joints_controller/joints_controller.h>
#include <string>
// #include <two_joints_controller/src/joint_controller.h>

JointsController::JointsController(ros::NodeHandle* nodehandle, std::string joint_number, double Kp, double Kv)
                :nh_(*nodehandle), joint_num_(joint_number), Kp_(Kp), Kv_(Kv)
{
    ROS_INFO("in class constructor of JointsController");

    checkService();

    initializePublishers();
    initializeSubscribers();
    initializeServices();
    q1_ = 0.0;
    q1dot_ = 0.0;
    dt_ = 0.01;
    q1_err_ = 0.0;
    trq_cmd_ = 0.0;
    duration_ = new ros::Duration(dt_);
    // rate_timer_ = new ros::Rate(1/dt_);

    effort_cmd_srv_msg_.request.joint_name = joint_num_;
    ROS_INFO("joint number to request: %s", effort_cmd_srv_msg_.request.joint_name.c_str());

    effort_cmd_srv_msg_.request.effort = 0.0;
    effort_cmd_srv_msg_.request.duration = *duration_;

    ROS_INFO_STREAM(effort_cmd_srv_msg_.request.duration);

    get_joint_state_srv_msg_.request.joint_name = joint_num_;

    joint_state_msg_.header.stamp = ros::Time::now();
    joint_state_msg_.name.push_back(joint_num_);
    joint_state_msg_.position.push_back(0.0);
    joint_state_msg_.velocity.push_back(0.0);
    vec_size_ = joint_state_msg_.name.size();
    ROS_INFO("I am here");
    ROS_INFO("the joint_state_msg_.name vector size: %d", vec_size_);
    ROS_INFO("joint number given by user: %s", joint_state_msg_.name[0].c_str());
    // controller();
}

JointsController::~JointsController()
{
    // delete half_sec_;
    delete duration_;
    // delete rate_timer_;
}

void JointsController::checkService()
{
    bool service_ready_ = false;
    ros::Duration half_sec_(0.5);

    while (!service_ready_)
    {
       service_ready_ = ros::service::exists("/gazebo/apply_joint_effort", true);
       ROS_INFO("waiting for apply_joint_effort service");
       half_sec_.sleep();
    }
    ROS_INFO("apply_joint_effort service exists");

    service_ready_ = false;
    while (!service_ready_)
    {
      service_ready_ = ros::service::exists("/gazebo/get_joint_properties", true);
      ROS_INFO("waiting for /gazebo/get_joint_properties service");
      half_sec_.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");
}

void JointsController::initializeServices()
{
    ROS_INFO("initializing service clients");
    set_trq_client_ =
       nh_.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    get_jnt_state_client_ =
       nh_.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
}

void JointsController::initializePublishers()
{
    ROS_INFO("initializing publishers");
    torque_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_trq", 1, true);
    velocity_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_vel", 1, true);
    position_pub_ = nh_.advertise<std_msgs::Float64>(joint_num_+"_pos", 1, true);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, true);
}

void JointsController::initializeSubscribers()
{
    ROS_INFO("initializing subscribers");
    pos_cmd_sub_ = nh_.subscribe(joint_num_+"_pos_cmd", 1, &JointsController::posCmdCB, this);
}


void JointsController::posCmdCB(const std_msgs::Float64& pos_cmd_msg)
{
    ROS_INFO("received value of pos_cmd is: %f", pos_cmd_msg.data);
    pos_cmd_ = pos_cmd_msg.data;
}
// double JointsController::sat(double val, double sat_val)
// {

// }

void JointsController::controller()
{
    // while(ros::ok()) {
    // If the service call succeeded, call() will return true and the value in srv.response
    // will be valid. If the call did not succeed,  call() will return false
    // and the value in srv.response will be invalid.
    while (!get_jnt_state_client_.call(get_joint_state_srv_msg_))
        {
            get_jnt_state_client_.call(get_joint_state_srv_msg_);
            ROS_INFO("The result for calling get_joint_state_srv_msg_: %d",
                                get_jnt_state_client_.call(get_joint_state_srv_msg_));
            ros::Duration(2.0).sleep();
        }

        // get_jnt_state_client_.call(get_joint_state_srv_msg_);
        q1_ = get_joint_state_srv_msg_.response.position[0];
        ROS_INFO("q1_ = %f", get_joint_state_srv_msg_.response.position[0]);
        q1_msg_.data = q1_;
        position_pub_.publish(q1_msg_);

        q1dot_ = get_joint_state_srv_msg_.response.rate[0];
        q1dot_msg_.data = q1dot_;
        velocity_pub_.publish(q1dot_msg_);

        joint_state_msg_.header.stamp = ros::Time::now();
        joint_state_msg_.position[0] = q1_;
        joint_state_msg_.velocity[0] = q1dot_;

        joint_state_pub_.publish(joint_state_msg_);

        // ROS_INFO("q1 = %f;  q1dot = %f",q1,q1dot);
        // watch for periodicity
        q1_err_ = pos_cmd_-q1_;
        if (q1_err_ > M_PI)
        {
            q1_err_ -= 2 * M_PI;
        }
        if (q1_err_< -M_PI)
        {
            q1_err_ += 2 * M_PI;
        }

        trq_cmd_ = Kp_*(q1_err_)-Kv_*q1dot_;
        // trq_cmd = sat(trq_cmd, 10.0); //saturate at 1 N-m
        trq_msg_.data = trq_cmd_;
        torque_pub_.publish(trq_msg_);
        // send torque command to Gazebo
        effort_cmd_srv_msg_.request.effort = trq_cmd_;
        set_trq_client_.call(effort_cmd_srv_msg_);
        // make sure service call was successful
        result_ = effort_cmd_srv_msg_.response.success;
        if (!result_)
            ROS_WARN("service call to apply_joint_effort failed!");
        // ros::spinOnce();
        // rate_timer_->sleep();
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_controller");
    ros::NodeHandle nh;
    // ros::NodeHandle nh1;
    // ros::NodeHandle nh2;
    ros::Rate rate_timer(1/0.01);
    JointsController jointsController1(&nh, "joint1", 50.0, 3.0);
    ROS_INFO("one JointsController object instantiated");

    JointsController jointsController2(&nh, "joint2", 5.0, 3.0);
    ROS_INFO("two JointsController objects instantiated");
    // JointsController jointsController1(&nh1, "joint1");
    // JointsController jointsController2(&nh2, "joint2");
    while (ros::ok())
    {
        ROS_INFO("starting to roll");

        jointsController1.controller();
        jointsController2.controller();

        ros::spinOnce();
        rate_timer.sleep();
    }

    return 0;
}
