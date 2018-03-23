#include "uav_devel/UavPC.h"

namespace uav_pc
{
    UavPC::UavPC(ros::NodeHandle& nodeHandle ):nodeHandle_(nodeHandle)
    {
        centriod.push_back(0.0);
        centriod.push_back(0.0);
        centriod.push_back(0.0);
        pcl_sub_ = nodeHandle_.subscribe("/camera/depth/points",1,&UavPC::pclCallback,this); 
        pcl_clusters_pub_= nodeHandle_.advertise<sensor_msgs::PointCloud2> ("/cloud_clusters", 1);
        red_object_pub= nodeHandle_.advertise<geometry_msgs::Point> ("/red_object", 1);
        
    }

    UavPC::~UavPC()
    {
    }
    void UavPC::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
             centriod[0]=0.0;
             centriod[1]=0.0;
             centriod[2]=0.0;

            pcl::PointCloud< pcl::PointXYZRGB>::Ptr PointCloudXYZ_(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*cloud_msg,*PointCloudXYZ_);
            if(!(PointCloudXYZ_->points.size()==0))
            { 
            // Container for original & filtered data
             pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
             pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
             pcl::PCLPointCloud2 cloud_filtered;

             

             // Convert to PCL data type
             pcl_conversions::toPCL(*cloud_msg, *cloud);

              // Perform the actual filtering
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloudPtr);
            sor.setLeafSize (0.01, 0.01, 0.01);
            sor.filter (cloud_filtered);

          
            sensor_msgs::PointCloud2 trans_cloud1,trans_cloud2,output;
             pcl_conversions::moveFromPCL(cloud_filtered, trans_cloud1);
             pcl_ros::transformPointCloud ("world", trans_cloud1, trans_cloud2, tf_listener);
             pcl::PointCloud< pcl::PointXYZRGB>::Ptr input_seg(new pcl::PointCloud<pcl::PointXYZRGB>);
             pcl::fromROSMsg(trans_cloud2,*input_seg);



             

             int nr_points = (int) input_seg->points.size ();

            while (input_seg->points.size()>0.3*nr_points)
            {
	         pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	         pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	         // Create the segmentation object
	         pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	        // Optional
	        seg.setOptimizeCoefficients (true);
	        // Mandatory
	        seg.setModelType (pcl::SACMODEL_SPHERE);
	        seg.setMethodType (pcl::SAC_RANSAC);
	        seg.setDistanceThreshold (0.1);
	        seg.setRadiusLimits (0, 100);
            seg.setInputCloud (input_seg);
	        seg.segment (*inliers, *coefficients);


            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sphere (new pcl::PointCloud<pcl::PointXYZRGB> ());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sphere_rest (new pcl::PointCloud<pcl::PointXYZRGB> ());
            //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

            if (inliers->indices.size () > 1200)
                {

                ROS_INFO("[%d]",inliers->indices.size ());
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud (input_seg);
                extract.setIndices (inliers);
                extract.setNegative (false);
                // Get the points associated with the planar surface
                extract.filter (*cloud_sphere);
                // Remove the sperical inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_sphere_rest);
                *input_seg = *cloud_sphere_rest;

              
                double count =0;
                double sum_x=0,sum_y=0,sum_z=0;
                int cluster_red=0,cluster_green=0,cluster_blue=0;

                    // ROS_INFO("[%d]",input_seg->indices.size ());
                for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
                {
                        sum_x +=cloud_sphere->points[(int)(count)].x;
                        sum_y +=cloud_sphere->points[(int)(count)].y;
                        sum_z +=cloud_sphere->points[(int)(count)].z;
                        
                        //cluster r and b values swap
                        uint8_t red = cloud_sphere->points[(int)(count)].r;
                        cloud_sphere->points[(int)(count)].r=cloud_sphere->points[(int)(count)].b;
                        cloud_sphere->points[(int)(count)].b=red;
                        //cluster histogram and centriod
                        cluster_red +=cloud_sphere->points[(int)(count)].r;
                        cluster_green +=cloud_sphere->points[(int)(count)].g;
                        cluster_blue +=cloud_sphere->points[(int)(count)].b; 
                        count +=1;
                } 



                //cluster color
                cluster_red /=count;
                cluster_green /=count;
                cluster_blue /=count;
                //color scalling removing instensity factor
                uint8_t max_color=0;
                if(cluster_red>=cluster_green && cluster_red>=cluster_blue) max_color=cluster_red;
                else if (cluster_green>=cluster_red && cluster_green>=cluster_blue) max_color=cluster_green;
                else if (cluster_blue>=cluster_red && cluster_blue>=cluster_green) max_color=cluster_blue;
                cluster_red=(cluster_red/(double)max_color)*255;
                cluster_green=(cluster_green/(double)max_color)*255;
                cluster_blue=(cluster_blue/(double)max_color)*255;
                if(cluster_red>(cluster_green+cluster_blue)/3) rgb.push_back('r'); 
                if(cluster_blue>(cluster_green+cluster_red)/3) rgb.push_back('b');
                if(cluster_green>(cluster_blue+cluster_red)/3) rgb.push_back('g'); 
                
                if(rgb[rgb.size()-1]='r')
                    {
                    centriod[0]=(sum_x)/count;
                    centriod[1]=(sum_y)/count;
                    centriod[2]=(sum_z)/count;
                    //ROS_INFO("RED:: piller_x: [%f]  piller_y: [%f] piller_z: [%f]",centriod[0],centriod[1],centriod[2]);
                    }
                    else if (rgb[rgb.size()-1]=='b')
                    {
                    centriod[3] = (sum_x)/count;
                    centriod[4] = (sum_y)/count;
                    centriod[5] = (sum_z)/count;
                    //ROS_INFO("BLUE:: piller_x: [%f]  piller_y: [%f] piller_z: [%f]",centriod[3],centriod[4],centriod[5]);
                    }
                pcl::toROSMsg(*cloud_sphere,output); 
                pcl_clusters_pub_.publish (output);
                 }
                 else break;

                }

            }
            
            geometry_msgs::Point msg;
            msg.x = centriod[0];
            msg.y = centriod[1];
            msg.z = centriod[2];
            ROS_INFO("RED:: piller_x: [%f]  piller_y: [%f] piller_z: [%f]",centriod[0],centriod[1],centriod[2]);
            red_object_pub.publish(msg);


     }   
           
          
    


}
