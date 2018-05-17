#include <pure_pursuit/PurePursuit.h>

PurePursuit::PurePursuit()
{
  current_target_wp_ = -1;
  lookahead_dist_th_ = 15; // mtrs- will be variable on speed 
}

PurePursuit::PurePursuit(ros::NodeHandle n)
{

  current_target_wp_ = -1;
  lookahead_dist_th_ = 15; // mtrs- will be variable on speed 
  nh_ = n;
  pub_viz_circle_pp_ = nh_.advertise<visualization_msgs::Marker>("/pp_circle",10);
  current_yaw_ = 0 ;

}

void PurePursuit::currentPoseCb(const nav_msgs::Odometry& msg)
{
  current_pose_.pose.pose.position = msg.pose.pose.position;
  current_pose_.pose.pose.orientation = msg.pose.pose.orientation;
  
  tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,theta;
  m.getRPY(roll,pitch,theta);
  current_yaw_ = theta;
}

void PurePursuit::currentWayPtCb(const wp_creator::WaypointVector& msg)
{
  waypoints_ = msg;
}


geometry_msgs::Point PurePursuit::getClosestPathPoint()
{
  // for now, just approximate to the next waypoint 
  // global coordinates 
  if (current_target_wp_ < 0) 
    current_target_wp_= 0;
  geometry_msgs::Point p;
  p.x = waypoints_.data[current_target_wp_].x;
  p.y = waypoints_.data[current_target_wp_].y;
  p.z = 0;
   
  return p;
}


float PurePursuit::getCurvature(const int target_wp)
{
  // convert from global frame wp to local frame 
  //Eigen::MatrixXd = Eigen::MatrixXd::Zeros(2,2);
  double current_yaw = current_yaw_; // TODO 
  double xg_local = (waypoints_.data[target_wp].x - current_pose_.pose.pose.position.x)*cos(current_yaw) + (waypoints_.data[target_wp].y - current_pose_.pose.pose.position.y)*sin(current_yaw);

  double yg_local = -(waypoints_.data[target_wp].x - current_pose_.pose.pose.position.x)*sin(current_yaw) + (waypoints_.data[target_wp].y - current_pose_.pose.pose.position.y)*cos(current_yaw);

  float D = calculateEucDist(std::make_pair(0,0), std::make_pair(xg_local, yg_local) );

  float kappa = 0;

  kappa = 2*(xg_local)/(D*D);


  return kappa;

}


geometry_msgs::Twist PurePursuit::getTwistCommand()
{
  geometry_msgs::Point closest_path_pt = getClosestPathPoint();
  int target_wp = getTargetWaypoint(closest_path_pt);
  float kappa = getCurvature(target_wp);

  // assume const vel for now 
  geometry_msgs::Twist twist_cmd;
  twist_cmd.linear.x  = 5; 
  twist_cmd.angular.z = kappa*5;

  // visulize Circle traj 
  visualizeCircleTraj(target_wp);

  return twist_cmd;
}

void PurePursuit::vehicleToGlobalFrame(double &x_result, double &y_result, double x_vf, double y_vf)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,3); 
  H<<cos(current_yaw_), -sin(current_yaw_),current_pose_.pose.pose.position.x,
    sin(current_yaw_), cos(current_yaw_), current_pose_.pose.pose.position.y,
    0,0,1; 

  Eigen::VectorXd p(3); // not initialized
  Eigen::VectorXd p_global_frame(3);
  p(2) = 1; p_global_frame(2) = 1;
  p(0) = x_vf; p(1) = y_vf;
  p_global_frame = H*p; 

  x_result = p_global_frame(0); 
  y_result = p_global_frame(1);
}

void PurePursuit::visualizeCircleTraj(int target_wp)
{

  // there could be two circles that fit the bill 
  double xc1_vf, yc1_vf;
  //double xc1, yc2;
  xc1_vf = 0; yc1_vf = lookahead_dist_th_; // this is in vehicle frame 
  double  xc1_gf, yc1_gf; vehicleToGlobalFrame(xc1_gf, yc1_gf, xc1_vf, yc1_vf);

  visualization_msgs::Marker marker; 

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(); 
  marker.id = 0;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD; 

  marker.pose.position.x = xc1_gf;
  marker.pose.position.y = yc1_gf;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.b = 1.0;
  marker.color.a = 1.0;

  pub_viz_circle_pp_.publish(marker);
  ROS_INFO("PurePursuit::published marker to viz");


}

int PurePursuit::getTargetWaypoint(geometry_msgs::Point p)
{

  // this ignores tight curves in the road and related issues 
  // considers only the euc dist 
  //
  // start from the current target wp 
  if (current_target_wp_ < 0) // this will technically never happen 
    return 0;

  float net_euc_dist = 0;
  for (wp_creator::WaypointType wp : waypoints_.data)
  {
    if (wp.ID < waypoints_.data[current_target_wp_].ID)
      continue;

    net_euc_dist += calculateEucDist(std::make_pair(p.x,p.y),std::make_pair(wp.x,wp.y));

    if (net_euc_dist > lookahead_dist_th_)
      return wp.ID;
  }

  return -1;

}


float PurePursuit::calculateEucDist(std::pair<float,float> p1, std::pair<float,float> p2)
{
  float d = sqrt(pow(p1.first - p2.first,2) + pow(p1.second - p2.second,2));
  return d;
}


PurePursuit::~PurePursuit()
{
}
