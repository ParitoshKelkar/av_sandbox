#include <iostream>
#include <prius_msgs/Control.h>
#include <av_planning_msgs/VehicleStateCubicSpline.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <lattice_trajectory_gen/libtraj_motion_model.h>
#include <lattice_trajectory_gen/libtraj_gen_common.h>

namespace common =  libtraj_gen_common;
namespace libmm =  libtraj_motion_model;

void curvatureCb(const std_msgs::Float64MultiArray&);
void startStateCb(const av_planning_msgs::VehicleStateCubicSpline&);

double start_time = 0.0;
bool new_state = false;
double goal_vel = 0;

common::VehicleState veh_current_state;
common::CubicSpline current_curvature;

void curvatureCb(const std_msgs::Float64MultiArray& msg)
{
  current_curvature.p0 = msg.data[0];
  current_curvature.p1 = msg.data[1];
  current_curvature.p2 = msg.data[2];
  current_curvature.p3 = msg.data[3];
  current_curvature.s = msg.data[4];
}

void startStateCb(const av_planning_msgs::VehicleStateCubicSpline& msg)
{
  veh_current_state.x = msg.x;
  veh_current_state.y = msg.y;
  veh_current_state.theta = msg.theta;
  veh_current_state.vel = msg.vel;
  veh_current_state.kappa = msg.kappa;
  goal_vel = msg.goal_vel;
  new_state = true;
  start_time = ros::Time::now().toSec();

}


 

int main(int argc, char** argv)
{
  ros::init(argc,argv,"throttle_gen");
  ros::NodeHandle nh;
  ros::Publisher pub_cmd_vel  = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  ros::Subscriber sub_start_state = nh.subscribe("/start_state_curvature",10,startStateCb);
  ros::Subscriber sub_curvature = nh.subscribe("/curvature",10,curvatureCb);
  float rate = 15;
  geometry_msgs::Twist twist_cmd;
  ros::Rate loop_rate(rate);
  common::VehicleState temp_veh_state;

  while(ros::ok())
  {
    ros::spinOnce();

    if (new_state)
    {
      double elapsed_time = ros::Time::now().toSec() - start_time;
      temp_veh_state = libmm::getNextState(veh_current_state,goal_vel, 1/rate, current_curvature,elapsed_time);
      new_state = false;

    }
    twist_cmd.linear.x = temp_veh_state.vel;
    ROS_INFO_STREAM("VEL NEEDED "<<twist_cmd.linear.x);
    twist_cmd.angular.z = temp_veh_state.vel*temp_veh_state.kappa;
    pub_cmd_vel.publish(twist_cmd);
    loop_rate.sleep();
  }

  return 0;
}
