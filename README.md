# eecs-600-sulu
This is the branch PS_5, which is the repo to contain ps5 assignment.

In src folder, there is a joint_controller_class.cpp whcih is the class version of joints controller,
also with a class version of joints position commander.

The sequence about how to start the two joints robot.

"roslaunch two_joints_controller two_joints_robot.launch"

"rosrun joints_trajectory joints_trajectory_action_client"

"rosrun joints_trajectory joints_trajectory_action_server"