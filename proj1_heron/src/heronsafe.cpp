#include "heronsafe.h"


void cb_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{

  return;
}



void cb_nearest_left( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  
  return;
}

void cb_nearest_right( const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  

  ROS_WARN_STREAM("Vel ="<< msg_vel.linear.x<< " DLeft="<<pose_out_left.pose.position.x<<" "<<pose_out_left.pose.position.y<<"  DRight="<<pose_out_right.pose.position.x<<" "<<pose_out_right.pose.position.y);

  return;
}




/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "heronsafe");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n_public;

  ros::Subscriber sub = n_public.subscribe("/heron/odom", 1, cb_odometry);
  ros::Subscriber sub_nearest_left = n_public.subscribe("/lidar_left/nearest", 1, cb_nearest_left);
  ros::Subscriber sub_nearest_right = n_public.subscribe("/lidar_right/nearest", 1, cb_nearest_right);

  listener =  new tf::TransformListener();
  pub = n_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  try
  {
    listener->waitForTransform("/lidar_right_link", "base_link", ros::Time::now(), ros::Duration(3.0));
    listener->waitForTransform("/lidar_left_link", "base_link", ros::Time::now(), ros::Duration(3.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  ros::spin();

  delete listener;
  return 0;
}
