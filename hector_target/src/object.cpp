#include <ros/ros.h>
#include <string.h>
#include "gazebo_msgs/SetModelState.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Int8.h>


int count_object=1,count=0,count2=0;
double object_x,object_y,object_z,pos_x,pos_y,pos_z;
std::string object_name;
ros::Publisher pub_object1;

void ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg_pos);
void posCallback(const geometry_msgs::PoseStamped &msg4);
void pos_hunterCallback(const geometry_msgs::PoseStamped &msg5);
void objectmotion();


ros::ServiceClient client;


int  main(int argc, char **argv)
{
    ros::init(argc, argv, "object");
    ros::NodeHandle n;

pub_object1 = n.advertise<std_msgs::Int8>("/object_number",1);
ros::Subscriber sub_state = n.subscribe("/gazebo/model_states", 1,ModelStatecallback); 
client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
ros::Subscriber sub_object1 = n.subscribe("/q1/ground_truth_to_tf/pose",1,posCallback);
ros::Subscriber sub_object2 = n.subscribe("/ground_truth_to_tf/pose",1,pos_hunterCallback); 



  
    ros::spin();
    return 0;
}


void ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg_pos)
{ 
  object_name = msg_pos->name[count_object];
  object_x = msg_pos->pose[count_object].position.x;
  object_y = msg_pos->pose[count_object].position.y;
  object_z = msg_pos->pose[count_object].position.z;
}

void posCallback(const geometry_msgs::PoseStamped &msg4)
{
  pos_x =  msg4.pose.position.x;
  pos_y =  msg4.pose.position.y;
  pos_z =  msg4.pose.position.z-0.3;
  double error = sqrt((pos_x-object_x)*(pos_x-object_x)+(pos_y-object_y)*(pos_y-object_y)+(pos_z-object_z)*(pos_z-object_z));
  
   if((error<0.1)) count++;
   std_msgs::Int8 object_index_;
   object_index_.data = count2;
   pub_object1.publish(object_index_);
   if(count>0 && count2<1) objectmotion();
}

void pos_hunterCallback(const geometry_msgs::PoseStamped &msg5)
{
  pos_x =  msg5.pose.position.x;
  pos_y =  msg5.pose.position.y;
  pos_z =  msg5.pose.position.z-0.3;
  double error = sqrt((pos_x-object_x)*(pos_x-object_x)+(pos_y-object_y)*(pos_y-object_y)+(pos_z-object_z)*(pos_z-object_z));
  
  if((error<0.65)) count2++;
   std_msgs::Int8 object_index_;
   object_index_.data = count2;
   pub_object1.publish(object_index_);
   if(count2>0) objectmotion();
   
}

void objectmotion()

{
    // Position
    geometry_msgs::Point pr2_position;
    pr2_position.x = pos_x;
    pr2_position.y = pos_y;
    pr2_position.z = pos_z;

    // orientation
    geometry_msgs::Quaternion pr2_orientation;
    pr2_orientation.x = 0.0;
    pr2_orientation.y = 0.0;
    pr2_orientation.z = 0.0;
    pr2_orientation.w = 1.0;

    // pose (Pose + Orientation)
    geometry_msgs::Pose pr2_pose;
    pr2_pose.position = pr2_position;
    pr2_pose.orientation = pr2_orientation;

    //ModelState
    gazebo_msgs::ModelState pr2_modelstate;
    pr2_modelstate.model_name = object_name;
    pr2_modelstate.pose = pr2_pose;



    //gazebo-->pose
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = pr2_modelstate;

    //Server
    if(client.call(srv))
    {
       // ROS_INFO("PR2's magic moving success!!");
    }
    else
    {
       // ROS_ERROR("Failed to magic move PR2! Error msg:%s",srv.response.status_message.c_str());
    }

  
}


