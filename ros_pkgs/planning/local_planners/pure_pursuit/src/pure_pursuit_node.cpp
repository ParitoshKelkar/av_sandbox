#include <ros/ros.h>
#include <iostream>

#include <pure_pursuit/PurePursuit.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"PurePursuit");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  PurePursuit pp(private_nh);
  

  ros::Subscriber sub_wp = n.subscribe("/default_waypoints",10,&PurePursuit::currentWayPtCb, &pp);
  ros::Subscriber sub_gt_pose_ = n.subscribe("/base_pose_ground_truth",10,&PurePursuit::currentPoseCb, &pp);

  ros::Publisher pub_twist_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);


  ros::Rate loop_rate(50); // 25Hz 
  ROS_INFO("pure_pursuit_node::INITIALIZING");
  geometry_msgs::Twist pp_twist;

  while(ros::ok())
  {
    ros::spinOnce();
    if(!pp.wp_set || !pp.pose_set)
    {
      ROS_INFO_THROTTLE(2,"pure_pursuit_node::Waiting for Topics");
      loop_rate.sleep();
      continue;
    }
    pp_twist = pp.getTwistCommand();
    pub_twist_cmd.publish(pp_twist);
    loop_rate.sleep();
  }

  return 0;
}
