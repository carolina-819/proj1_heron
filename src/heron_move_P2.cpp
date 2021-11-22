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
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "std_msgs/Float32.h"
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

bool init = false;  
ros::Publisher flag_pub, vel_pub;
ros::Subscriber pose, centered;
std_msgs::Bool men;
int state = 0;
int stateR = -1, stateL = -1;
tf::TransformListener *listener;
geometry_msgs::PoseStamped pose_out_left, poseout;
geometry_msgs::Twist vel;
float centrado;
int x=0;

void cb_nearest_left(const geometry_msgs::PoseStamped::ConstPtr &msg){
    listener->transformPose("/base_link", *msg, pose_out_left);
    return;
}

void centeredCB(const std_msgs::Float32::ConstPtr& msg){
    if (msg->data)
        centrado = msg->data;
     
    //ROS_INFO("centered? %s", centrado ? "true" : "false");
    return;
}

bool align_boat(double x, double y, double z, double w, double target){
    //bool aligned = false;
    tf::Quaternion qtf(
        x,
        y,
        z,
        w);
        tf::Matrix3x3 m(qtf);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if((target - yaw) < 0.1 &&  (target - yaw) > -0.1){
            //ROS_INFO("ALINHADO");
            return true;
        }else{
            //ROS_INFO("DIFERENCA %f", target - yaw);
            vel.angular.z = 2 * (target-yaw);
            vel_pub.publish(vel);
            return false;
        }
        //ROS_INFO("%f ", yaw);
}

int decide_nextStep(int sR, int sL){
    if((sR == 13 || sR == 23 || sR == 33) && (sL == 13 || sL == 23 || sL == 33) ){ //gostraight
        return 2;
    }else if(sL == 22){
        return 3;
    }else if(sR == 22){
        return 4;
    }else if((sR == 11 || sR == 21 || sR == 31) || (sL == 11 || sL == 21 || sL == 31)){
        return 5;
    }else{
        return 6; //do nothing
    }
}

void moveToDock(int dock){
    //ROS_INFO("Distancia: %f\n", pose_out_left.pose.position.x);
    if(dock == 0){
        if(centrado<280){
            vel.linear.x=0;
            vel.angular.z = 1.0;   
        }
        else if(centrado>320){
            vel.linear.x = 0;
            vel.angular.z=-1.0;
        }else if(pose_out_left.pose.position.x >= 2 && centrado > 280 && centrado < 320){
            vel.linear.x = 3.0;
            vel.angular.z=0;
            }
        else {
            vel.linear.x=0;
            vel.angular.z = 0;
        }
    }
    else if(dock == 1){
        if(centrado<280){
            vel.linear.x=0;
            vel.angular.z =1.0;   
        }
        else if(centrado>320){
            vel.angular.z=-1.0;
        }else if(pose_out_left.pose.position.x >= 2 && centrado > 280 && centrado < 320){
                vel.linear.x = 3.0;
                vel.angular.z=0;
            }
            else {
                vel.linear.x=0;
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
    bool aligned = false;
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
        init = true;        
        men.data = init;
        flag_pub.publish(men);
    }else if(stateR != - 1 && state == 0){
        init = false;
        men.data = init;
        flag_pub.publish(men);
        //start state machine
        state = 1;
    }if(state == 1){ 
        state = decide_nextStep(stateR, stateL);
    }else if(state == 2){ //segue em frente
        aligned = align_boat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, 1.57);
        if(aligned){
            vel.linear.x = 4.0;
            vel_pub.publish(vel);
        }

    }else if(state == 3){ //docka
        aligned = align_boat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, -3.14);
        if(aligned){
            state = 35;
        }
        
    }else if(state == 35){
        moveToDock(0);
    }else if(state == 4){
        aligned = align_boat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, 0);
        if(aligned){
            state = 45;
        }
    }else if(state == 45){
        moveToDock(1);
    }else if(state == 5){ //volta p tras
        aligned = align_boat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, -1.57);
        if(aligned){
            vel.linear.x = 4.0;
            vel_pub.publish(vel);
        }
    }else if(state == 6){
        vel.linear.x = 0;
        vel.angular.z = 0;
        vel_pub.publish(vel);
    }
    //ROS_INFO("state %d", state);
     //vel.linear.x = 4.0;
    
}

void markerleftCB(const std_msgs::Int8::ConstPtr& msg){
    stateL = msg->data;
    //ROS_INFO("STATEL: %d", stateL);
      
}
void markerrightCB(const std_msgs::Int8::ConstPtr& msg){
    stateR = msg->data;
    //ROS_INFO("STATER: %d", stateR);

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
   
    ros::Subscriber sub_nearest_left = nh.subscribe("lidar_left/nearest", 1, cb_nearest_left);
    listener = new tf::TransformListener();

    try{
        listener->waitForTransform("/lidar_right_link", "base_link", ros::Time::now(), ros::Duration(3.0));
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