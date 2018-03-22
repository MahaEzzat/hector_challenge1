#include <ros/ros.h>
#include "hector_hunter/HectorHunter.h"

int  main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_hunter");
    ros::NodeHandle n("~");

 hector_hunter::HectorHunter HectorHunter(n);

    ros::spin();
    return 0;
}