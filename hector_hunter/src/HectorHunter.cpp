#include <ros/ros.h>
#include "hector_hunter/HectorHunter.h"

namespace hector_hunter
{
    HectorHunter::HectorHunter(ros::NodeHandle& n) :
    n_(n)
    {

if(!Parameters())
        {
            ROS_ERROR("Error in loading parameters");  
            ros::shutdown();
        }
     HectorHunter::motor_enable();
     HectorHunter::Path();
      sub_pos = n_.subscribe("/ground_truth_to_tf/pose",1,&HectorHunter::posCallback,this);
      sub_IMU = n_.subscribe("/raw_imu",1,&HectorHunter::imuCallback,this);    
      sub_object = n_.subscribe("/object_number",1,&HectorHunter::object_subCallback,this); 
      red_object_sub = n_.subscribe("/red_object",1,&HectorHunter::redobjectCallback,this);  
      pub_=  n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);   
    }

    HectorHunter::~HectorHunter()
    {

    }



//Methods  
 void HectorHunter::imuCallback(const sensor_msgs::Imu &msg3)
    {

 tf::Quaternion bg(msg3.orientation.x,msg3.orientation.y,msg3.orientation.z,msg3.orientation.w);
 tf::Matrix3x3(bg).getRPY(roll,pitch,yaw);
 HectorHunter::PID();
 HectorHunter::twistmeth();
    }


void HectorHunter::twistmeth()
    {
    msg2.linear.x = vxi;
    msg2.linear.y = vyi;
    msg2.linear.z = vzi;
    msg2.angular.x = 0; 
    msg2.angular.y = 0; 
    msg2.angular.z = 0; 
    }



void HectorHunter::posCallback(const geometry_msgs::PoseStamped &msg4)
{
  pos_x =  msg4.pose.position.x;
  pos_y =  msg4.pose.position.y;
  pos_z =  msg4.pose.position.z;
  }

void HectorHunter::object_subCallback(const std_msgs::Int8 &msg_obj)
{
    object_number = msg_obj.data;
}

void HectorHunter::redobjectCallback(const geometry_msgs::Point &Red_object)
  {

if(object_number==0)
{

 if(targetx.size()>0)
      {
    if((Red_object.z<0.5) && count_path != 1)
    {
        pathx[1]=targetx[targetx.size()-1];
        pathy[1]=targety[targety.size()-1];
        pathz[1]=targetz[targetz.size()-1];
    }
       if((Red_object.z<0.5) && count_path != 0)
    {
        pathx[0]=targetx[targetx.size()-1];
        pathy[0]=targety[targety.size()-1];
        pathz[0]=targetz[targetz.size()-1];

    }
      }

        if((error < 0.05) && (Red_object.z<0.5))
        {
            msg2.linear.x = 0;
            msg2.linear.y = 0;
            msg2.linear.z = 0;
            msg2.angular.x = 0; 
            msg2.angular.y = 0;
            msg2.angular.z = cos(pitch)*cos(roll)*5.0;
        }
        
      

    if(Red_object.z>0.5)
    {
        if((Red_object.x-pos_x)>0)  targetx.push_back(Red_object.x-0.45);
        else targetx.push_back(Red_object.x+0.45);
        if((Red_object.y-pos_y)>0)  targety.push_back(Red_object.y-0.45);
        else targety.push_back(Red_object.y+0.45);
        targetz.push_back(Red_object.z);


        pathx[1]=pathx[0]=targetx[targetx.size()-1];
        pathy[1]=pathy[0]=targety[targety.size()-1];
        pathz[1]=pathz[0]=targetz[targetz.size()-1];
    }
 } 

 else 
 {
     pathx[1]=pathx[0]=-2.0;
     pathy[1]=pathy[0]=-2.0;
     pathz[1]=pathz[0]=1.0; 
 }

   pub_.publish(msg2);
}

void HectorHunter::PID()
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

if( error < 0.05) {
    count_path++;
    if(count_path>(pathx.size()-1)) count_path=0;
}


pos_prev_x = pos_x;
pos_prev_y = pos_y;
pos_prev_z = pos_z;



vxi = (cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch))*vx - cos(roll)*sin(yaw)*vy + (cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw))*vz;
vyi = (sin(yaw)*cos(pitch)+sin(roll)*cos(yaw)*sin(pitch))*vx + cos(roll)*cos(yaw)*vy + (sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw))*vz;
vzi = -cos(roll)*sin(pitch)*vx  +  sin(roll)*vy   + cos(pitch)*cos(roll)*vz;


}


void HectorHunter::Path()
{
 pathx.push_back(0.0);
 pathy.push_back(0.0);
 pathz.push_back(2.0);
 pathx.push_back(0.0);
 pathy.push_back(0.0);
 pathz.push_back(2.0);
}


  bool HectorHunter::Parameters()
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

    void HectorHunter::motor_enable()
{
    //motor enable 
    client_motor = n_.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
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

}
