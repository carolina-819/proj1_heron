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
#include "std_msgs/Int8.h"
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

bool init = false;  
ros::Publisher flag_pub, vel_pub;
ros::Subscriber pose, centered;
std_msgs::Bool men;
int state = 0;
int stateR = -1, stateL=-1;

geometry_msgs::Twist vel;
bool centrado=false;
int x=0;


void centeredCB(const std_msgs::Bool::ConstPtr& msg){
    
    if (msg->data)
        centrado = msg->data;
    else centrado = false;    
    ROS_INFO("centered? %s", centrado ? "true" : "false");
    return;
}

void cb_nearest_left();

void moveToDock(int dock){
    ROS_INFO("TAS A ANDAR?");
    if(dock == 0){
        if(!centrado){
            vel.linear.x=0;
            vel.angular.z = 1.0;   
        }
        if(centrado){
            vel.angular.z=0;

          //  vel.linear.x = 2.0;
        }
    }
    else if(dock == 1){
        if(!centrado){
            vel.linear.x=0;
            vel.angular.z = -1.0;
        }            
        if(centrado){
            vel.angular.z=0;
        }
    }
    else{
        vel.angular.z = 0;
    }
    vel_pub.publish(vel);
    return;
}

void poseCB(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::Twist vel;
    
    if(msg->pose.pose.position.y<-7 && state == 0){

        vel.linear.x = 2.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        init = false;
        men.data = init;
        flag_pub.publish(men);
       
    }else if(state == 0 &&msg->pose.pose.position.y<-1.5 && msg->pose.pose.position.y>=-7){
        vel.linear.x = 1.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        init = false;
        men.data = init;
        flag_pub.publish(men);
       
    }else if(state == 0 && msg->pose.pose.position.y>=-1.5 && msg->pose.pose.position.y<=-0.5){
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
    }else if(state == 0 && msg->pose.pose.position.y>-0.5 && stateR == -1){
        //init = true;
        moveToDock(1);
        //men.data = init;
        //flag_pub.publish(men);
    }else if(stateR != - 1 && state == 0){ //sao diferentes
        init = false;
        men.data = init;
        flag_pub.publish(men);
        //start state machine
        state = 1;
        //Alinha_barco()
        //check_availabilty()
        //ROS_INFO("wtf %s", stateR);
    }
    if(state == 1){
        //align boat
        moveToDock(1);
    }
     //vel.linear.x = 4.0;
    
}

void markerleftCB(const std_msgs::Int8::ConstPtr& msg){
    stateL = msg->data;
    ROS_INFO("STATEL: %d", stateL);
      
}
void markerrightCB(const std_msgs::Int8::ConstPtr& msg){
    stateR = msg->data;
    ROS_INFO("STATER: %d", stateR);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "heron_controller");
    ros::NodeHandle nh;
    ros::Subscriber shapeleft_sub, shaperight_sub;

    shapeleft_sub = nh.subscribe("/marker_shape_left", 1, markerleftCB);
    shaperight_sub = nh.subscribe("/marker_shape_right", 1, markerrightCB);
    centered = nh.subscribe("/centered", 1, centeredCB);
    ros::spinOnce();
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    flag_pub = nh.advertise<std_msgs::Bool>("/camera_request", 1);

    ros::Subscriber pose = nh.subscribe("/heron/odom", 1, poseCB);
    
    ros::spin();
   
    return 0;
}