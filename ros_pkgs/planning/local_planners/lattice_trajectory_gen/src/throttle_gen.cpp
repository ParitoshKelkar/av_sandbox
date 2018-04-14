#include <iostream>
#include <prius_msgs/Control.h>
#include <lattice_trajectory_gen/VehicleStateCubicSpline.h>
#include <geometry_msgs/Twist.h>
#include <lattice_trajectory_gen/libtraj_motion_model.h>
#include <lattice_trajectory_gen/libtraj_gen_common.h>

namespace common =  libtraj_gen_common;
namespace libmm =  libtraj_motion_model;

void curvatureCb(const common::CubicSpline&);
void startStateCb(const lattice_trajectory_gen::VehicleStateCubicSpline&);

double start_time = 0.0;
bool new_state = false;
double goal_vel = 0;

common::VehicleState veh_current_state;
common::CubicSpline current_curvature;

void curvatureCb(const common::CubicSpline& msg)
{
  curvature.p0 = msg.data[0];
  curvature.p1 = msg.data[1];
  curvature.p2 = msg.data[2];
  curvature.p3 = msg.data[3];
  curvature.length = msg.data[4];
}

void startStateCb(const lattice_trajectory_gen::VehicleStateCubicSpline& msg)
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
  ros::rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();
    if (new_state)
    {
      double elapsed_time = ros::Time::now().toSec() - start_time;
      common::VehicleState temp_veh_state = libmm::getNextState(current_state,goal_vel, 1/rate, curvature,elapsed_time);
      new_state = false;

    }
    twist_cmd.twist.linear.x = temp_veh_state.vel;
    twist_cmd.twist.angular.z = temp_veh_state.vel*temp_veh_state.kappa;
    pub_cmd_vel.advertise(twist_cmd);
    loop_rate.sleep();
  }

  return 0;
}
