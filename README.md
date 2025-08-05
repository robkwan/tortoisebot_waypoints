# TortoiseBot Waypoints

## Description
This package contains an action server that allows a TortoiseBot to follow waypoints.

0. Set up from git repo:
   ```bash
   cd ~/simulation_ws/src/tortoisebot_waypoints
   git checkout main

1. Launch the Gazebo simulation:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/simulation_ws/devel/setup.bash
   roslaunch tortoisebot_gazebo tortoisebot_playground.launch

2. Launch the Action Server in another terminal:
   ```bash
   source /opt/ros/noetic/setup.bash
   cd ~/simulation_ws && catkin_make && source devel/setup.bash
   rosrun tortoisebot_waypoints tortoisebot_action_server.py

3. Run ros tests on default good cases in another terminal:
   ```bash
   source /opt/ros/noetic/setup.bash
   cd ~/simulation_ws && catkin_make && source devel/setup.bash
   rostest tortoisebot_waypoints waypoints_test.test --reuse-master

======
4. Before running ros test on bad case, need to modify the waypoints_tests.test with "waypoint_test_node_err.py" replacing the original waypoint_test_node.py" first.
   i.e.
       <test test-name="test_waypoint" pkg="tortoisebot_waypoints" type="waypoint_test_node_err.py" />

   - It may be good to relaunch everything from step 1 to 3 above but it is ok to run step 4 only if both the gazebo and action server are still running correctly.

   Simply
   ```bash
   source /opt/ros/noetic/setup.bash
   cd ~/simulation_ws && catkin_make && source devel/setup.bash
   rostest tortoisebot_waypoints waypoints_test.test --reuse-master

   will show the error result with timeout as goal can't be reached after 15 seconds goal is sent.

   

