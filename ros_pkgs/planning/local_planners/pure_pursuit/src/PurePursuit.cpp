#include <pure_pursuit/pure_pursuit.h>

PurePursuit::PurePursuit()
{
  current_target_wp_ = -1;
  lookahead_dist_ = 15; // mtrs- will be variable on speed 
}

void PurePursuit::currentPoseCb(const nav_msgs::Odometry& msg)
{
  current_pose_ = msg.pose;
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
  // use current pose 
  double current_yaw;
  tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  current_yaw = yaw;

  //Eigen::MatrixXd = Eigen::MatrixXd::Zeros(2,2);
  double xg_local = (waypoints_.data[target_wp].x - current_pose_.pose.position.x)*cos(current_yaw) + (waypoints_.data[target_wp].y - current_pose_.pose.position.y)*sin(current_yaw);

  double yg_local = -(waypoints_.data[target_wp].x - current_pose_.pose.position.x)*sin(current_yaw) + (waypoints_.data[target_wp].y - current_pose_.pose.position.y)*cos(current_yaw);

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

  return twist_cmd;

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
  for (auto wp : waypoints_.data)
  {
    if (wp.id < waypoints_.data[current_target_wp_].id)
      continue;

    net_euc_dist + = calculateEucDist(std::make_pair(p.x,p.y),std::make_pair(wp.x,wp.y));

    if (net_euc_dist > lookahead_dist_th_)
      return wp.id;
  }

  return -1;

}


float PurePursuit::calculateEucDist(std::pair<float,float> p1, std::pair<float,float> p2)
{
  float d = sqrt(pow(p1.first - p2.first,2) + pow(p1.second - p2.second));
  return d;
}

