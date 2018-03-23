# hector_challenge1
How to run it:
clone hector_hunter hector_target uav_devel
build them
cd catkin_ws
source devel/setup.bash
roslaunch hector_hunter m_hector.launch
wait utill everything is set up
in new terminal
cd catkin_ws
source devel/setup.bash
roslaunch hector_hunter hector_target.launch
wait untill the target in its path then
in new terminal
cd catkin_ws
source devel/setup.bash
roslaunch hector_hunter hector_hunter.launch
