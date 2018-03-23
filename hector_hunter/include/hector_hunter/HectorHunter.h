#ifndef HECTORHUNTER_H_
#define HECTORHUNTER_H_

#include <ros/ros.h>
#include <string.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <math.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Int8.h>




namespace hector_hunter
{
	class HectorHunter
	{
		public:
			// constructor
			HectorHunter(ros::NodeHandle& n);
			
			// destructor
			virtual ~HectorHunter();
		private:

		//create Ros nodehandle, sub, and pub 
			ros::NodeHandle& n_;
			ros::Subscriber sub_IMU;
			ros::Subscriber sub_pos;
			ros::Subscriber sub_object;
			ros::Subscriber red_object_sub;
			ros::Publisher  pub_;
			ros::ServiceClient client_motor;
			
			
		
			
		//create methods	
			bool Parameters();
			void twistmeth();
			void mark();
			void motor_enable();
     
	  void imuCallback(const sensor_msgs::Imu &msg3);	
	  void posCallback(const geometry_msgs::PoseStamped &msg4);
	  void redobjectCallback(const geometry_msgs::Point &Red_object);
	  void object_subCallback(const std_msgs::Int8 &msg_obj);
	  void PID(); 
	  void Path();
 

          //create arguments
		  geometry_msgs::Twist msg2;

			double roll,yaw,pitch;
			float roll_dot,yaw_dot,pitch_dot;
			float vx,vy,vz;
			float vxi,vyi,vzi;
			float pos_x,pos_y,pos_z; 
			float pos_prev_x,pos_prev_y,pos_prev_z;
			float error_x,error_y,error_z,error;
			float error_d_x,error_d_y,error_d_z;
			float error_i_x,error_i_y,error_i_z;
			float t,t_prev;
			float pos_targ_x;
			float pos_targ_y;
			float pos_targ_z;
			float kp_x,kp_y,kp_z;
			float ki_x,ki_y,ki_z;
			float kd_x,kd_y,kd_z;
			double x,y,r;
			double dt;
			int count_path=0;
			int object_number=0;
			int count=0;
			double theta=0.0;
            std::vector< double > pathx;
            std::vector< double > pathy;
			std::vector< double > pathz;
			std::vector< double > targetx;
			std::vector< double > targety;
			std::vector< double > targetz;

			
			 
			 
	};
	
}




#endif