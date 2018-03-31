#include <lattice_trajectory_gen/libtraj_motion_model.h>
#include <lattice_trajectory_gen/libtraj_gen_common.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <ros/ros.h>

namespace common =  libtraj_gen_common;
namespace libmm =  libtraj_motion_model;

void amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped&);

void amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& current_pose)
{

}

int main(int argc, char** argv)
{
  double dt = 0.1;
  ros::init(argc,argv,"lattice_traj_gen");
  ros::NodeHandle nh;

  common::VehicleState start(25,70,0,0,0.1);  // x,y,theta,kappa,vel
  common::VehicleState goal(25+6,70+4,0.0,0.2,0.1); 


  ros::Subscriber amcl_pose_sub = nh.subscribe("/amcl_pose",10,amclPoseCb);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);



  common::CubicSpline curvature = common::initCurvature(start,goal);


  ROS_INFO_STREAM(" -- BEGINNING FIRST INTEGRATION WITH SPLINE "<<curvature<<"--\n");
  common::VehicleState integrated_state = libmm::motionModel(start,goal,dt,curvature);
  ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
  ROS_INFO(" - BEGINNING CORRECTIONS - ");
  bool converged = false;
  int converge_epochs = 0;

  while( !converged && converge_epochs < 5)
  {
    common::CubicSpline delta_param = libmm::generateCorrection(start,goal,integrated_state,dt,curvature);
    curvature.p1 = curvature.p1 - delta_param.p1;
    curvature.p2 = curvature.p2 - delta_param.p2;
    curvature.s = curvature.s - delta_param.s;

    ROS_INFO_STREAM(" - new spline "<<curvature<<"-");
    ROS_INFO("lattice_traj_gen::node running MM with adjusted curvature");
    integrated_state = libmm::motionModel(start,goal,dt,curvature);
    ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
    ROS_INFO(" - END ITER_EPOCH %d -",converge_epochs);
    ROS_INFO("\n");
    converged = common::hasConverged(goal,integrated_state);
    converge_epochs+=1;
  }
  if (converged)
  {

    // publish to rviz 
    // publish corresponding twist messages 


  }


  return 0;
}

