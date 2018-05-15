#include <pure_pursuit/pure_pursuit.h>

PurePursuit::PurePursuit()
{
  current_target_wp_ = -1;
  lookahead_dist_ = 15; // mtrs- will be variable on speed 
}

void PurePursuit::currentPoseCb(const nav_msgs::Odometry& msg)
{
}

void PurePursuit::currentWayPtCb(const wp_creator::WaypointVector& msg)
{
}


geometry_msgs::Point PurePursuit::getClosestPathPoint()
{
  // for now, just approximate to the next waypoint 
  // global coordinates 
  geometry_msgs::Point p;
  p.x = waypoints_.data[current_target_wp_].x;
  p.y = waypoints_.data[current_target_wp_].y;
  p.z = 0;
   
  return p;
}

int PurePursuit::getTargetWaypoint(geometry_msgs::Point)
{

  // this ignores tight curves in the road and related issues 
  // considers only the euc dist 
  //
  // start from the current target wp 
  if (current_target_wp_ < 0)
    return 0;

  float net_euc_dist = 0;
  for (auto wp : waypoints_.data)
  {
    if (wp.id < waypoints_.data[current_target_wp_].id)
      continue;

    net_euc_dist + = calculateEucDist(std::make_pair<float, float>(p.x,p.y),std::make_pair<float,float>(wp.x,wp.y));

    if (net_euc_dist > lookahead_dist_th_)
      return wp.id;
  }

  return -1;

}

