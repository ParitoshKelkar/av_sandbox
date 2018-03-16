#include <lattice_trajectory_gen/libtraj_motion_model.h>
#include <lattice_trajectory_gen/libtraj_gen_common.h>

#include <iostream>
#include <ros/ros.h>

namespace common =  libtraj_gen_common;
namespace libmm =  libtraj_motion_model;

int main(int argc, char** argv)
{
  double dt = 0.1;
  ros::init(argc,argv,"lattice_traj_gen");
  ros::NodeHandle nh;

  common::VehicleState start(0,0,0,0,0.1);  // x,y,theta,kappa,vel
  common::VehicleState goal(6,4,0.0,0.2,0.1); 



  common::CubicSpline curvature = common::initCurvature(start,goal);


  common::VehicleState integrated_state = libmm::motionModel(start,goal,dt,curvature);
  ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
  common::CubicSpline delta_param = libmm::generateCorrection(start,goal,integrated_state,dt,curvature);


  return 0;
}

