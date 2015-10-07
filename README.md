# eecs-600-sulu
This is the branch PS_4, which is the repo to contain ps4 assignment.

The sequence about how to start the minimal_robot.
1. For gazebo

	rosrun gazebo_ros gazebo

	" navigate to the folder "minimal_robot_description", then enter the following in terminal "

	roscd minimal_robot_description

	rosrun gazebo_ros spawn_model -file minimal_robot_description.urdf -urdf -model one_DOF_robot

	rosrun minimal_joint_controller minimal_joint_controller

	rostopic pub pos_cmd std_msgs/Float64 1.0

2. For Rviz
	
	rosrun rviz rviz

	roslaunch minimal_robot_description minimal_robot_description.launch

	rosrun robot_state_publisher robot_state_publisher

