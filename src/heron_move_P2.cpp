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
std_msgs::Bool men;
int state = 0;
std::string stateR ("not ready"), stateL ("not ready");


void poseCB(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::Twist vel;
    
    if(msg->pose.pose.position.y<-7){

        vel.linear.x = 2.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        init = false;
        men.data = init;
        flag_pub.publish(men);
       
    }else if(msg->pose.pose.position.y<-1.5 && msg->pose.pose.position.y>=-7){
        vel.linear.x = 1.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        init = false;
        men.data = init;
        flag_pub.publish(men);
       
    }else if(msg->pose.pose.position.y>=-1.5 && msg->pose.pose.position.y<=-0.5){
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
    }else if(msg->pose.pose.position.y>-0.5){
        init = true;
        men.data = init;
        flag_pub.publish(men);
    }
    if(stateR != "not ready"){
        vel.linear.x = 0.0;
        vel.angular.z = 10.0;
        vel_pub.publish(vel);
    }
     //vel.linear.x = 4.0;
    
}

void markerleftCB(const std_msgs::String::ConstPtr& msg){
    stateR = msg->data;
      
}
void markerrightCB(const std_msgs::String::ConstPtr& msg){
    stateL = msg->data;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "heron_controller");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    flag_pub = nh.advertise<std_msgs::Bool>("/camera_request", 1);
    ros::Subscriber shapeleft_sub = nh.subscribe("/marker_shape_left", 1, markerleftCB);
    ros::Subscriber shaperight_sub = nh.subscribe("/marker_shape_right", 1, markerrightCB);
    ros::spinOnce();
    ros::Subscriber pose = nh.subscribe("/heron/odom", 1, poseCB);
    
    

    
    
    


    ros::spin();
   
    return 0;
}