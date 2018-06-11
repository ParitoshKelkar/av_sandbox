#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <wp_creator/WaypointVector.h>
#include <wp_creator/WaypointType.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>

float current_x, current_y, current_theta;
bool recieved_pose = false;
void gtPoseCb(const nav_msgs::Odometry&);
wp_creator::WaypointVector generateWaypointsInWorldFrame(float,float);
visualization_msgs::Marker generateVizMarker(wp_creator::WaypointVector);


void gtPoseCb(const nav_msgs::Odometry& msg)
{
  // take in the position and check for convergence only using that right now 
  current_x = msg.pose.pose.position.x;
  current_y = msg.pose.pose.position.y;
  tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  current_theta = yaw;
  recieved_pose = true;
}

visualization_msgs::Marker generateVizMarker(const wp_creator::WaypointVector wp_vec)
{
  visualization_msgs::Marker marker; 

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(); 
  marker.id = 0;

  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD; 

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5; 

  marker.color.g = 1.0;
  marker.color.a = 1.0;

  for (wp_creator::WaypointType iter_wp : wp_vec.data)
  {

    geometry_msgs::Point p;
    p.x = iter_wp.x; 
    p.y = iter_wp.y; 

    marker.points.push_back(p);

  }

  return marker;

}


wp_creator::WaypointVector generateWaypointsInWorldFrame(float resolution, float num_wp)
{
  // first generate points in local frame and then convert into global 
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,3); 
  H<<cos(current_theta), -sin(current_theta),current_x,
    sin(current_theta), cos(current_theta), current_y,
    0,0,1; 
  int wp_id = 0; 
  wp_creator::WaypointVector wp_vec; 
  wp_vec.header.frame_id = "map";
  Eigen::VectorXd p(3); // not initialized
  Eigen::VectorXd p_global_frame(3);
  p(2) = 1; p_global_frame(2) = 1;
  wp_creator::WaypointType temp_wp;
  temp_wp.header.frame_id = "map";
  for (int iter = 1; iter <=num_wp; iter++)
  {
    float x_offset = iter*resolution; // not handling negative X etc 
    p(0) = x_offset; p(1) = 0;
    p_global_frame = H*p;

    temp_wp.ID = wp_id; wp_id++; 
    temp_wp.x = p_global_frame(0); 
    temp_wp.y = p_global_frame(1); 
    wp_vec.data.push_back(temp_wp);
  }

  return wp_vec;

}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"default_wp_loader");
  ros::NodeHandle nh;

  // get current pose of vehicle and publish 30 wps in front of it 
  ros::Subscriber sub_pose = nh.subscribe("/base_pose_ground_truth",10,gtPoseCb);
  ros::Publisher pub_def_wps = nh.advertise<wp_creator::WaypointVector>("/waypoints",10);
  ros::Publisher pub_viz_def_wps = nh.advertise<visualization_msgs::Marker>("/viz_waypoints",10);
  ros::Rate loop_rate(1);


  while(!recieved_pose && ros::ok())
  {
    ros::spinOnce();
    ROS_WARN_THROTTLE(2,"default_wp_loader::WAITING FOR AMCL POSE");
    loop_rate.sleep();

  }

  ROS_INFO("default_wp_loader::Recieved Pose, generating points");

  float spacing = 5;
  float num_wp = 30;
  wp_creator::WaypointVector wp_vec = generateWaypointsInWorldFrame(spacing,num_wp);
  visualization_msgs::Marker viz_marker = generateVizMarker(wp_vec);
  ROS_INFO("default_wp_loader:: Calculated Points");
  ROS_INFO("default_wp_loader::Publishing to Rviz");

  while(ros::ok())
  {
    pub_def_wps.publish(wp_vec);
    pub_viz_def_wps.publish(viz_marker);
    loop_rate.sleep();
  }


  return 0;
}
