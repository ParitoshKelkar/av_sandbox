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

common::VehicleState goal(26+6,70+4,0.0,0.2,0.1); 
bool pose_reached = false;
double current_x, current_y, current_theta, current_vel, current_omega;

void amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped&);
void odomCb(const nav_msgs::Odometry&);
common::VehicleState updateStartPose();

void amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& current_pose)
{
  // take in the position and check for convergence only using that right now 
  if (fabs(current_pose.pose.pose.position.x - goal.x) < 0.2 && fabs(current_pose.pose.pose.position.y - goal.y) < 0.2 )
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

  common::VehicleState start(26,70,0,0,0.1);  // x,y,theta,kappa,vel


  ros::Subscriber sub_amcl_pose = nh.subscribe("/amcl_pose",10,amclPoseCb);
  ros::Subscriber sub_odom = nh.subscribe("/base_pose_ground_truth",10,odomCb);

  ros::Publisher pub_curvature = nh.advertise<std_msgs::Float64MultiArray>("/curvature",10);
  ros::Publisher pub_start_state = nh.advertise<av_planning_msgs::VehicleStateCubicSpline>("/start_state_curvature",10);
  av_planning_msgs::VehicleStateCubicSpline temp_veh_state;
  std_msgs::Float64MultiArray curv_pub_array;
  curv_pub_array.data.resize(5);

  while (ros::ok())
  {
    ros::spinOnce(); // update pose if needed 
    //if (!poseHasChanged(start)); // will have to implement this in a while 

    start = updateStartPose();
    start.vel = current_vel;
    start.kappa = current_omega/start.vel;

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
      // publish relevant topics 
      temp_veh_state.x = start.x; temp_veh_state.y = start.y; temp_veh_state.theta = start.theta; temp_veh_state.vel = start.vel; temp_veh_state.kappa = start.kappa;temp_veh_state.goal_vel = goal.vel;
      pub_start_state.publish(temp_veh_state);
      curv_pub_array.data[0] = curvature.p0; curv_pub_array.data[1] = curvature.p1; curv_pub_array.data[2] = curvature.p2; curv_pub_array.data[3] = curvature.p3; curv_pub_array.data[4] = curvature.s;
      pub_curvature.publish(curv_pub_array);
    }

    loop_rate.sleep();

  }





  //if (converged)
  //{

    //// publish to rviz 
    //// publish corresponding twist messages 
    //double elapsed_time = 0;
    //double horizon = curvature.s/goal.vel;
    //common::VehicleState current_state;
    //double prev_time;
    //while(!pose_reached)
    //{
      //ros::spinOnce();

      //current_state = libmm::getNextState(current_state,goal,dt,curvature,elapsed_time);
      //elapsed_time+= dt;
      //current_state.x = current_x; //Cb
      //current_state.y = current_y; //Cb

    //}


  //}


  return 0;
}

