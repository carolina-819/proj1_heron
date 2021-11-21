#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <ros/console.h>
#include "nav_msgs/Odometry.h"
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

bool init = false;  
ros::Publisher flag_pub, vel_pub;
int state = 0;

void poseCB(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::Twist vel;
    ROS_INFO("%f \n", msg->pose.pose.position.x);
    if(msg->pose.pose.position.y<-7){
        vel.linear.x = 2.0;
        vel.angular.z = 0.0;
       
    }else if(msg->pose.pose.position.y<-1 && msg->pose.pose.position.y>=-7){
        vel.linear.x = 1.0;
        vel.angular.z = 0.0;
       
    }else{
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        std_msgs::Bool men;
        men.data = true;
        flag_pub.publish(men);
    }
     //vel.linear.x = 4.0;
    vel_pub.publish(vel);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "heron_controller");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    flag_pub = nh.advertise<std_msgs::Bool>("/camera_request", 1);
    ros::Subscriber pose = nh.subscribe("/heron/odom", 1, poseCB);
    

    
    //ros::Subscriber shape_sub = nh.subscribe("/marker_shape", 1, markerCB);
    


    ros::spin();
   
    return 0;
}