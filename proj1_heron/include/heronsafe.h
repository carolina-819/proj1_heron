#ifndef GAZEBO2ODOM_H
#define GAZEBO2ODOM_H

/// C++ includes
#include <algorithm>

/// Ros includes
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

///MSGs includes
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

///TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub;
tf::TransformListener *listener;
geometry_msgs::PoseStamped pose_out_left;
geometry_msgs::PoseStamped pose_out_right;

geometry_msgs::Twist msg_vel;

void cb_odometry(const nav_msgs::Odometry::ConstPtr &msg);
void cb_nearest_left( const geometry_msgs::PoseStamped::ConstPtr &msg);
void cb_nearest_right( const geometry_msgs::PoseStamped::ConstPtr &msg);


#endif
