#include <wp_creator/waypoint_loader.h>

WaypointLoader::WaypointLoader(ros::NodeHandle n):nh_(n)
{
}
WaypointLoader::~WaypointLoader()
{
}


visualization_msgs::Marker WaypointLoader::getVizMarker(const wp_creator::WaypointVector wp_vec)
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


wp_creator::WaypointVector WaypointLoader::readFileAndExtractWaypoints(std::string file_path)
{
  std::ifstream fs(file_path.c_str());
  if (fs.is_open())
          ROS_INFO("File opened successfully\n");
  else
  {
          ROS_ERROR("waypoint_loader:File could not be opened. File path is %s \n",file_path.c_str());
          exit(-1);
  }

  std::string el;
  std::string row;
  std::vector<std::string> rowElements;
  wp_creator::WaypointType temp_wp;
  temp_wp.header.frame_id = "map";
  wp_creator::WaypointVector wp_vec;
  int row_num = 0;
  while (std::getline(fs,row))
  { 
    
    std::istringstream ss(row);
    std::string column;
    
    while (std::getline(ss, el, ','))
        rowElements.push_back(el);

    
    temp_wp.ID = row_num;
    temp_wp.x = std::stof(rowElements[0]);
    temp_wp.y = std::stof(rowElements[1]);

    row_num +=  1; 

    wp_vec.data.push_back(temp_wp);

    rowElements.clear();
  }


  fs.close();
  ROS_INFO("File Read and closed");

  return wp_vec;
}

