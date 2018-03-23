#include <ros/ros.h>
#include "hector_target/HectorTarget.h"

namespace hector_target
{
    HectorTarget::HectorTarget(ros::NodeHandle& n) :
    n_(n)
    {

if(!Parameters())
        {
            ROS_ERROR("Error in loading parameters");  
            ros::shutdown();
        }
     HectorTarget::Path();
     HectorTarget::motor_enable();
     sub_state = n_.subscribe("/gazebo/model_states", 1,&HectorTarget::ModelStatecallback,this); 
     sub_pos = n_.subscribe("/q1/ground_truth_to_tf/pose",1,&HectorTarget::posCallback,this);   
     sub_IMU = n_.subscribe("/q1/raw_imu",1,&HectorTarget::imuCallback,this);
     pub_=  n_.advertise<geometry_msgs::Twist>("/q1/cmd_vel",1);   
     
    }

    HectorTarget::~HectorTarget()
    {

    }



//Methods  
 void HectorTarget::imuCallback(const sensor_msgs::Imu &msg3)
    {
 tf::Quaternion bg(msg3.orientation.x,msg3.orientation.y,msg3.orientation.z,msg3.orientation.w);
 tf::Matrix3x3(bg).getRPY(roll,pitch,yaw);
 HectorTarget::PID();
 HectorTarget::twistmeth();
 pub_.publish(msg2);
    }


void HectorTarget::twistmeth()
    {
     msg2.linear.x = vxi;
     msg2.linear.y = vyi;
     msg2.linear.z = vzi;
     msg2.angular.x = 0; 
     msg2.angular.y = 0; 
     msg2.angular.z = 0; 
    }



void HectorTarget::posCallback(const geometry_msgs::PoseStamped &msg4)
 {
  pos_x =  msg4.pose.position.x;
  pos_y =  msg4.pose.position.y;
  pos_z =  msg4.pose.position.z;
  }

void HectorTarget::PID()
{

pos_targ_x = pathx[count_path];
pos_targ_y = pathy[count_path];
pos_targ_z = pathz[count_path];


if(count==0)
{
t_prev = ros::Time::now().toSec();
  error_d_x =0;
  error_d_y =0;
  error_d_z =0;
  error_i_x = 0;
  error_i_y = 0;
  error_i_z = 0;
  error=0.6;
  count++;
}

else
{ 
t = ros::Time::now().toSec() ;
dt = t - t_prev;
t_prev = t;

error_d_x = (pos_prev_x-pos_x)/dt;
error_d_y = (pos_prev_y-pos_y)/dt;
error_d_z = (pos_prev_z-pos_z)/dt;

error_i_x += error_x*dt;
error_i_y += error_y*dt;
error_i_z += error_z*dt;
}

error_x = pos_targ_x-pos_x;
error_y = pos_targ_y-pos_y;
error_z = pos_targ_z-pos_z;
error = sqrt(error_x*error_x+error_y*error_y+error_z*error_z);

vx = kp_x*error_x + ki_x * error_i_x + kd_x *error_d_x;
vy = kp_y*error_y + ki_y * error_i_y + kd_y *error_d_y;
vz = kp_z*error_z + ki_z * error_i_z + kd_z *error_d_z;

if( error < 0.09) {
    count_path++;
    if(count_path>(pathx.size()-1)) count_path=2;
}


pos_prev_x = pos_x;
pos_prev_y = pos_y;
pos_prev_z = pos_z;



vxi = (cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch))*vx - cos(roll)*sin(yaw)*vy + (cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw))*vz;
vyi = (sin(yaw)*cos(pitch)+sin(roll)*cos(yaw)*sin(pitch))*vx + cos(roll)*cos(yaw)*vy + (sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw))*vz;
vzi = -cos(roll)*sin(pitch)*vx  +  sin(roll)*vy   + cos(pitch)*cos(roll)*vz;


}


void HectorTarget::Path()
{


 pathx.push_back(0.0);
 pathy.push_back(0.0);
 pathz.push_back(2.0);

 pathx.push_back(0.0);
 pathy.push_back(0.0);
 pathz.push_back(2.0);


  while(theta<44/7){
    theta += 0.5;
    r=1.5;
    x = r*cos(theta);
    y = r*sin(theta);
    pathx.push_back(x);
    pathy.push_back(y); 
    pathz.push_back(2.0);
} 



}


  bool HectorTarget::Parameters()
    {
        if(!n_.getParam("kp_x_",kp_x)) return false;
        if(!n_.getParam("kp_y_",kp_y)) return false;
        if(!n_.getParam("kp_z_",kp_z)) return false;
        if(!n_.getParam("ki_x_",ki_x)) return false;
        if(!n_.getParam("ki_y_",ki_y)) return false;
        if(!n_.getParam("ki_z_",ki_z)) return false;
        if(!n_.getParam("kd_x_",kd_x)) return false;
        if(!n_.getParam("kd_y_",kd_y)) return false;
        if(!n_.getParam("kd_z_",kd_z)) return false;


        return true;
    }

    void HectorTarget::motor_enable()
{
    //motor enable 
    client_motor = n_.serviceClient<hector_uav_msgs::EnableMotors>("/q1/enable_motors");
    hector_uav_msgs::EnableMotors srv2;
    srv2.request.enable = true;
    if(client_motor.call(srv2))
    {
        ROS_INFO("motor enable success!!");
    }
    else
    {
        ROS_ERROR("Failed to enable motor!");
    }
}


void HectorTarget::ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg_pos)
{ 
  object_name = msg_pos->name[count_object];
  object_x = msg_pos->pose[count_object].position.x;
  object_y = msg_pos->pose[count_object].position.y;
  object_z = msg_pos->pose[count_object].position.z+0.3;
  pathx[0] = object_x;
  pathy[0] = object_y;
  pathx[1] = object_x;
  pathy[1] = object_y;
  pathz[1] = object_z;

}



}
