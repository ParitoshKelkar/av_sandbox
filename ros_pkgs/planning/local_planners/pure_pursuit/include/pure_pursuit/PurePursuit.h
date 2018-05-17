/*
* @author ParitoshKelkar 
*
* @version 
*
* @brief 
*
*/
#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <wp_creator/WaypointVector.h>
#include <wp_creator/WaypointType.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class PurePursuit
{
private:

  wp_creator::WaypointVector waypoints_;
  geometry_msgs::PoseWithCovarianceStamped current_pose_;
  double current_yaw_;
  int current_target_wp_;
  float lookahead_dist_th_;
  ros::NodeHandle nh_;

  ros::Publisher pub_viz_circle_pp_;
  



  /**
  * @brief  uses known vehicle pose in global frame to make conversion 
  *
  * @param  double& - x_result
  *         double& - y_result
  *         double  - x(veh_frame)
  *         double - y(veh_frame)
  *
  * @return void
  * 
  * @throws Exception
  */
  void vehicleToGlobalFrame(double&, double&, double, double);


  /**
  * @brief  calculate and publish circle marker to viz
  *
  * @param  int - the target waypoint 
  *
  * @return void
  * 
  * @throws Exception
  */
  void visualizeCircleTraj(int);
  

  /**
  * @brief  ** self explanatory **
  *
  * @param   
  *
  * @return geometry_msgs::Point
  * 
  * @throws Exception
  */
  geometry_msgs::Point getClosestPathPoint();


  /**
  * @brief  might be an approximation or could be geometrically 
  *         calculated
  *
  * @param  geometry_msgs::Point - the closest path Point
  *
  * @return int - the waypoint index
  * 
  * @throws 
  */
  int getTargetWaypoint(geometry_msgs::Point);


  /**
  * @brief  uses target position to determine 
  *         the appropriate curvature needed
  *
  * @param  const int - the idx of the next wp
  *
  * @return float
  * 
  * @throws Exception
  */
  float getCurvature(const int);


  /**
  * @brief  ** self explanatory **
  *
  * @param  std::pair<float,float>
  *         std::pair<float,float>
  *
  * @return float
  * 
  * @throws Exception
  */
  float calculateEucDist(std::pair<float,float>,std::pair<float,float>);



  

public:
  PurePursuit();
  PurePursuit(ros::NodeHandle n);
  virtual ~PurePursuit();

  /**
  * @brief  serves as the main func that
  *         processess all the ips
  *
  * @param  
  *
  * @return   geometry_msgs::Twist - twist that is fed to the controller
  * 
  * @throws Exception
  */
  geometry_msgs::Twist getTwistCommand();

  // subscribers to made public 
  void currentPoseCb(const nav_msgs::Odometry&);
  void currentWayPtCb(const wp_creator::WaypointVector&);
};

#endif /* PUREPURSUIT_H */
