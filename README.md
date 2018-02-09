# hanoi_iiwa
Solving the Hanoi towers with the iiwa.

To start the moveIt move group with the gripper attached, run 'roslaunch iiwa14_s_model_moveit run_move_group'

The gripper controller has to be started with root privileges, so run 'sudo -s' (or in our case the alias 'sudoros') and (after setting up ROS environment variables for root) then run 'roslaunch robotiq_s_model_control s_model_ethercat.launch'

Now you can start this node by running 'roslaunch hanoi_iiwa hanoi.launch'. Note that we rerouted the topics from the robotiq controller node, so we can publish and subscribe to them from our node without changing the code. Change the launch file accordingly if your gripper controller uses a different namespace!