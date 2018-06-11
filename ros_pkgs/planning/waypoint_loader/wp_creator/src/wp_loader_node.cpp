#include <wp_creator/waypoint_loader.h>
#include <wp_creator/WaypointVector.h>
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
  ROS_INFO("INITIALISING WAYPOINT LOADER NODE");
  ros::init(argc, argv, "waypoint_loader_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  ros::Publisher pub_wps_ = nh.advertise<wp_creator::WaypointVector>("waypoints",10,true); // latching is set to true 
  ros::Publisher pub_viz_wps_ = nh.advertise<visualization_msgs::Marker>("viz_waypoints",10,true); // latching is set to true 
  std::string temp_str = ros::package::getPath("wp_creator");
  std::string csv_file_path = temp_str.append("/data/mcity_cone_wps.csv");
  std::string backup_path = "/home/paritoshkelkar/.autoware/data/mcity_cone_wps.csv";
  WaypointLoader wp_loader(nh);
  wp_creator::WaypointVector wp_vec = wp_loader.readFileAndExtractWaypoints(csv_file_path); // no error checks right now 
  visualization_msgs::Marker wp_viz_marker = wp_loader.getVizMarker(wp_vec);

  while(ros::ok())
  {
    pub_wps_.publish(wp_vec);
    pub_viz_wps_.publish(wp_viz_marker); 
    loop_rate.sleep();
  }

  return 0;
}
