#include <ros/ros.h>
#include "uav_devel/UavPC.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_devel");
  ros::NodeHandle nodeHandle("~");
  
  uav_pc::UavPC uavpc(nodeHandle);
 
  ros::spin();
  return 0;
}