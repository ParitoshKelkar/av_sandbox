#include <ros/ros.h>
#include <iostream>

#include <pure_pursuit/PurePursuit.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"PurePursuit");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  PurePursuit pp(private_nh);

  ros::Subscriber sub_wp = n.subscribe("/default_wp",10,&PurePursuit::currentWayPtCb, &pp);
  ros::Subscriber sub_gt_pose_ = n.subscribe("/base_pose_ground_truth",10,&PurePursuit::currentPoseCb, &pp);


  ros::Rate loop_rate(25); // 25Hz 
  ROS_INFO("pure_pursuit_node::INITIALIZING");
  geometry_msgs::Twist pp_twist;

  while(ros::ok())
  {
    ros::spinOnce();
    pp_twist = pp.getTwistCommand();
  }

  return 0;
}
