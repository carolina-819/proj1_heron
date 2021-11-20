#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <ros/console.h>
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

bool init = false;  
ros::Publisher flag_pub, vel_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heron_controller");
    ros::NodeHandle nh;
    cv::namedWindow("inRange");

    //ros::Subscriber shape_sub = nh.subscribe("/marker_shape", 1, markerCB);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    flag_pub = nh.advertise<std_msgs::Bool>("/camera_request", 1);


    ros::spin();
    cv::destroyWindow("inRange");
    return 0;
}