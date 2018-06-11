#include <lattice_trajectory_gen/libtraj_motion_model.h>
#include <lattice_trajectory_gen/libtraj_gen_common.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <av_planning_msgs/VehicleStateCubicSpline.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <ros/ros.h>

namespace common =  libtraj_gen_common;
namespace libmm =  libtraj_motion_model;

common::VehicleState global_waypoint(16,5,1.57,0.2,3); 
//common::VehicleState start_global(0,0,0,0,3); 

bool pose_reached = false;
double current_x, current_y, current_theta, current_vel, current_omega;

void amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped&);
void odomCb(const nav_msgs::Odometry&);
common::VehicleState updateStartPose();

void amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& current_pose)
{
  // take in the position and check for convergence only using that right now 
  if (fabs(current_pose.pose.pose.position.x - global_waypoint.x) < 0.2 && fabs(current_pose.pose.pose.position.y - global_waypoint.y) < 0.2 )
    pose_reached = true;
  current_x = current_pose.pose.pose.position.x;
  current_y = current_pose.pose.pose.position.y;
  tf::Quaternion q(current_pose.pose.pose.orientation.x,current_pose.pose.pose.orientation.y,current_pose.pose.pose.orientation.z,current_pose.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  current_theta = yaw;

}


void odomCb(const nav_msgs::Odometry& msg)
{
  current_vel = msg.twist.twist.linear.x;
  current_omega = msg.twist.twist.angular.z;
}


common::VehicleState updateStartPose()
{
  common::VehicleState temp_state;
  temp_state.x = current_x;
  temp_state.y = current_y;
  temp_state.theta = current_theta;

  return temp_state;
}



int main(int argc, char** argv)
{
  double dt = 0.1;
  ros::init(argc,argv,"lattice_traj_gen");
  ros::NodeHandle nh;
  double rate = 15; // Hz
  ros::Rate loop_rate(rate);

  //common::VehicleState start(0,0,0,0,0.1);  // x,y,theta,kappa,vel
  //common::VehicleState goal(6,4,0.1,0.2,0.1);


  ros::Subscriber sub_amcl_pose = nh.subscribe("/amcl_pose",10,amclPoseCb);
  ros::Subscriber sub_odom = nh.subscribe("/base_pose_ground_truth",10,odomCb);

  ros::Publisher pub_curvature = nh.advertise<std_msgs::Float64MultiArray>("/curvature",10);
  ros::Publisher pub_start_state = nh.advertise<av_planning_msgs::VehicleStateCubicSpline>("/start_state_curvature",10);
  av_planning_msgs::VehicleStateCubicSpline temp_veh_state;
  std_msgs::Float64MultiArray curv_pub_array;
  curv_pub_array.data.resize(5);

/*
 *  while (ros::ok())
 *  {
 *    ros::spinOnce(); // update pose if needed 
 *    //if (!poseHasChanged(start)); // will have to implement this in a while 
 *    if (current_x == 0)
 *    {
 *      loop_rate.sleep();
 *      continue;
 *    }
 *
 *    common::VehicleState v_state_temp = updateStartPose();
 *    goal.x = global_waypoint.x - v_state_temp.x;
 *    goal.y = global_waypoint.y - v_state_temp.y;
 *
 *    if (fabs(current_vel) < 0.001)
 *      current_vel = 0;
 *
 *    start.vel = current_vel;
 *    if (start.vel == 0)
 *      start.vel = 0.1;
 *    
 *    start.kappa = current_omega/start.vel;
 *    if (current_omega < 0.1)
 *      start.kappa = 0.0;
 *    if (fabs(current_theta) < 0.01)
 *      current_theta = 0;
 *
 *    start.theta = current_theta;
 *    start.theta = 0;
 *    start.vel = 0.1;
 *
 *
 *    ROS_INFO_STREAM(" -- BEGINNNING WITH START POSE --- "<<start);
 *    common::CubicSpline curvature = common::initCurvature(start,goal);
 *    ROS_INFO_STREAM(" -- BEGINNING FIRST INTEGRATION WITH SPLINE "<<curvature<<"--\n");
 *    common::VehicleState integrated_state = libmm::motionModel(start,goal,dt,curvature);
 *    ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
 *    ROS_INFO(" - BEGINNING CORRECTIONS - ");
 *    bool converged = false;
 *    int converge_epochs = 0;
 *    while( !converged && converge_epochs < 5)
 *    {
 *      common::CubicSpline delta_param = libmm::generateCorrection(start,goal,integrated_state,dt,curvature);
 *      curvature.p1 = curvature.p1 - delta_param.p1;
 *      curvature.p2 = curvature.p2 - delta_param.p2;
 *      curvature.s = curvature.s - delta_param.s;
 *
 *      ROS_INFO_STREAM(" - new spline "<<curvature<<"-");
 *      ROS_INFO("lattice_traj_gen::node running MM with adjusted curvature");
 *      integrated_state = libmm::motionModel(start,goal,dt,curvature);
 *      ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
 *      ROS_INFO(" - END ITER_EPOCH %d -",converge_epochs);
 *      ROS_INFO("\n");
 *      converged = common::hasConverged(goal,integrated_state);
 *      converge_epochs+=1;
 *    }
 *    if (converged)
 *    {
 *      ROS_INFO(" -- CONVERGED --");
 *      // publish relevant topics 
 *      temp_veh_state.x = start.x; temp_veh_state.y = start.y; temp_veh_state.theta = start.theta; temp_veh_state.vel = start.vel; temp_veh_state.kappa = start.kappa;temp_veh_state.goal_vel = goal.vel;
 *      pub_start_state.publish(temp_veh_state);
 *      curv_pub_array.data[0] = curvature.p0; curv_pub_array.data[1] = curvature.p1; curv_pub_array.data[2] = curvature.p2; curv_pub_array.data[3] = curvature.p3; curv_pub_array.data[4] = curvature.s;
 *      pub_curvature.publish(curv_pub_array);
 *    }
 *
 *    loop_rate.sleep();
 *
 *  }
 */





  common::VehicleState start(0,0,0,0,3);  // x,y,theta,kappa,vel
  common::VehicleState goal(16,5,1.57,0.2,3); 



  //common::CubicSpline curvature = common::initCurvature(start,goal);
  common::CubicSpline curvature(start.kappa,0,0,goal.kappa,20); // use guess generated from matlab code 


  ROS_INFO_STREAM(" -- BEGINNING FIRST INTEGRATION WITH SPLINE "<<curvature<<"--\n");
  common::VehicleState integrated_state = libmm::motionModel(start,goal,dt,curvature);
  ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
  ROS_INFO(" - BEGINNING CCRRECTIONS - ");
  bool converged = false;
  int converge_epochs = 0;

  while( !converged && converge_epochs < 5)
  {
    common::CubicSpline delta_param = libmm::generateCorrection(start,goal,integrated_state,dt,curvature);
    double udpated_p1 = curvature.p1 + delta_param.p1;
    double updated_p2 = curvature.p2 + delta_param.p2;
    double updated_s = curvature.s + delta_param.s;

    ROS_INFO_STREAM(" - new spline "<<curvature<<"-");
    ROS_INFO("lattice_traj_gen::node running MM with adjusted curvature");
    // remake curvature 
    curvature.paramReset(start.kappa,udpated_p1, updated_p2, goal.kappa,updated_s);
    integrated_state = libmm::motionModel(start,goal,dt,curvature);
    ROS_INFO_STREAM("lattice_traj_gen::node state after motion model "<<integrated_state);
    ROS_INFO(" - END ITER_EPOCH %d -",converge_epochs);
    ROS_INFO("\n");
    converged = common::hasConverged(goal,integrated_state);
    converge_epochs+=1;

  }
  if (converged)
  {
    ROS_INFO(" -- CONVERGED -- ");
    ROS_INFO_STREAM("INTEGRATED STATE "<<integrated_state);
  }

  return 0;
}

