#ifndef UAV_PC_H_
#define UAV_PC_H_

#include <ros/ros.h>
#include <string.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <math.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
//Computer Vision
#include <cv_bridge/cv_bridge.h>
#include <opencv-3.3.1/opencv2/imgproc/imgproc.hpp>
#include <opencv-3.3.1/opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>




namespace uav_pc
{
	class UavPC
	{	
		public:
			UavPC(ros::NodeHandle& nodeHandle);
		virtual ~UavPC();
		private: 
			ros::NodeHandle nodeHandle_;
			ros::Subscriber pcl_sub_;
			ros::Publisher  pcl_clusters_pub_;
			ros::Publisher  red_object_pub;
			tf::TransformListener tf_listener;
			std::vector< double > centriod;
			std::vector< double > pathx;
            std::vector< double > pathy;
		    
            std::vector< char > rgb;

			void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
			std::vector< double > object_centroid;
			
	};
}


#endif
