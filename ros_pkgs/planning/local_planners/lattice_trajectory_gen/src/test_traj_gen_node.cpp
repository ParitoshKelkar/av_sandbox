#include <lattice_trajectory_gen/libtraj_motion_model.h>
#include <lattice_trajectory_gen/libtraj_gen_common.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <av_planning_msgs/VehicleStateCubicSpline.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <ros/ros.h>

namespace common =  libtraj_gen_common;
namespace libmm =  libtraj_motion_model;

// set params 
common::VehicleState start(0,0,0.0,0,0.2,3); //x,y,theta,kappa,vel 
common::VehicleState goal(0,0,0.0,0,0.2,3); //x,y,theta,kappa,vel - will be defined in the Cb
bool new_goal_rx = false;


void simpleGoalCb(const geometry_msgs::PoseWithCovarianceStamped&);
bool computeTraj();

void simpleGoalCb(const geometry_msgs::PoseWithCovarianceStamped& rviz_goal)
{
  goal.sx = rviz_goal.pose.pose.position.x;
  goal.sy = rviz_goal.pose.pose.position.y;
  tf::Quaternion q(rviz_goal.pose.pose.orientation.x,rviz_goal.pose.pose.orientation.y,rviz_goal.pose.pose.orientation.z,rviz_goal.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  goal.theta = yaw;
  new_goal_rx = true;
  
}

bool computeTrajectory() // use the start and goal from global scope
{

}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"test_traj_gen_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);


  ros::Subscriber sub_goal_pose  = nh.subscribe("/simple_goal",10,simpleGoalCb);



  while (ros::ok())
  {
    if (!new_goal_rx)
    {
      ROS_WARN_THROTTLE("NEW GOAL NOT RECIEVED");
      loop_rate.sleep();
      continue;
    }

    bool result = computeTrajectory();

    loop_rate.sleep();
  }
  
  


  return 0;
}
