#include <ros/ros.h>
#include "hector_target/HectorTarget.h"

int  main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_target");
    ros::NodeHandle n("~");

 hector_target::HectorTarget HectorTarget(n);

    ros::spin();
    return 0;
}