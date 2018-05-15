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

class PurePursuit
{
private:

  wp_creator::WaypointVector waypoints_;
  geometry_msgs::PoseWithCovarianceStamped current_pose_;
  int current_target_wp_;
  float lookahead_dist_th_;
  

  void currentPoseCb(const nav_msgs::Odometry&)
  void currentWayPtCb(const wp_creator::WaypointVector&);
  

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
};

#endif /* PUREPURSUIT_H */
