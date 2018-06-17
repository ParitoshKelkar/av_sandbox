#ifndef WAYPOINT_LOADER_H
#define WAYPOINT_LOADER_H
#include <iostream>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <wp_creator/WaypointVector.h>
#include <wp_creator/WaypointType.h>

class WaypointLoader
{
private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_simple_goal_;
        bool goal_pose_set_;
        
        double decelerate_;

        /**
        * @brief Initializes the private members(publisher included)
        *
        * @param: 
        *
        * @return: bool
        */
        bool initPrivateMembers();


public:
        WaypointLoader();
        WaypointLoader(ros::NodeHandle);
        ~WaypointLoader(); // If this were declared virtual, then it would have to be defined

        /**
        * @brief This function uses file_path_ private member to locate file. Reads and Publishes waypoints
        *
        * @param 
        *
        * @return bool
        */
        wp_creator::WaypointVector readFileAndExtractWaypoints(std::string);


        /**
        * @brief  constructs viz marker from point in waypoint list 
        *
        * @param wp_creator::WaypointVector
        *
        * @return visualization_msgs::Marker
        */
        visualization_msgs::Marker getVizMarker(const wp_creator::WaypointVector);

};

#endif /* WAYPOINT_LOADER_H */
