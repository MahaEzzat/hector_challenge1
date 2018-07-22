# hector_challenge1

Description: 
Tracking and allocating quadcopter to pick something from. 
This simulation developed with ROS in CPP. 

How to run it:
1.clone hector_hunter hector_target uav_devel
2.build them
3.cd catkin_ws
4.source devel/setup.bash
5.roslaunch hector_hunter m_hector.launch
6.wait util everything is set up
7.in new terminal cd catkin_ws
8.source devel/setup.bash
9.roslaunch hector_hunter hector_target.launch
10.wait until the target in its path
11.in new terminal cd catkin_ws
12.source devel/setup.bash
13.roslaunch hector_hunter hector_hunter.launch

Video link:
https://www.youtube.com/watch?v=zY0xxevXzww&list=PLsQg1LQa63kDVPgswaw-mNshjz-HAWOF6&index=4
