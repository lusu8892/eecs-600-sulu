# eecs-600-sulu
This is the branch PS_6, which is the repo to contain ps6 assignment.

create a library (in its own package) called "my_interesting_moves"

This library should contain mulitple member functions named mnemonically, e.g.

   set_goal_salute(trajectory_msgs::JointTrajectory &des_trajectory);

Create at least 3 interesting functions that populate trajectory messages for Baxter's right arm.

Document your API with Doxygen comments in your header file.

In a separate package, create a node that uses your new library to move Baxter.
It should demonstrate each of your interesting moves by populating trajectory messages
via your library's functions, and executing these using the baxter trajectory streamer action server. (see package: cwru_baxter/baxter_traj_streamer/src/traj_action_clien_pre_pose.cpp for an example action client compatible with this action server).


run:
roslaunch cwru_baxter_sim baxter_world.launch
wait for the message: "Gravity compensation was tuned off"
in another window, enable the robot with the command:
  rosrun baxter_tools enable_robot.py -e
This command will run to completion.

Start the trajectory interpolator action server:
  rosrun baxter_traj_streamer traj_interpolator_as
Leave this node running.

In another terminal, start your node.  For test purposes, run the example client node:
  rosrun baxter_traj_streamer traj_action_client_pre_pose

make a movie of your resulting motions.

Include in your submission:
*the github URL's for your library and for your node
*(your github must include your html files for Doxygen-generated documentation)
*a movie of Baxter executing your designed moves

additionally, please run the ros linter on your code and correct (the majority of) all of the errors it complains to you about.

rosrun roslint cpplint file.cpp