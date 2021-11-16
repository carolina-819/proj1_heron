#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "geometry_msgs/Twist.h"
#include <ros/console.h>
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

#define MAX_FEATURES 100


bool init = true;  //flag that is true only at the beginning
ros::Publisher pub;
geometry_msgs::Twist vel;
// Callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat imagem = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::Mat hsv_image;
    cv::Mat mask, image_final, bwfind;
    cv::Mat result_green, green_contour;
    //cv::imshow("view", imagem);

    cv::cvtColor(imagem, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(40, 40, 40), cv::Scalar(70, 255, 255), mask);
    cv::bitwise_and(hsv_image, hsv_image, result_green, mask = mask);
    std::vector<cv::Mat> hsv_planes;
    cv::split(result_green, hsv_planes);
    //hsv_planes[0] // H channel
    //hsv_planes[1] // S channel
    //hsv_planes[2] // V channel

    std::vector<std::vector<cv::Point>> contours;
	  std::vector<cv::Vec4i> hierarchy;
    cv::findContours(hsv_planes[1], contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    cv::drawContours(imagem, contours, -1, cv::Scalar(255, 0, 0), 5);
    cv::Moments m = cv::moments(hsv_planes[1]);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);
    cv::circle(imagem, p, 5, cv::Scalar(125, 10, 125), -1);
    std::string str;
    cv::Size s = imagem.size();
  
    if(p.x > 300){ //camara esta à direita do verde
    vel.linear.x=0.0;
    vel.angular.z=-1.0;//Publish the message. 
    }else{
      vel.linear.x=0.0;
      vel.angular.z=1.0;//Publish the message. 
    }
     pub.publish(vel);  

    cv::putText(imagem,  std::to_string(p.x), cv::Point(p.x, p.y + 0.1), cv::FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0));
    cv::imshow("inRange", imagem);

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_subcriber_feature");
  ros::NodeHandle nh;
  cv::namedWindow("inRange");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/left/image_raw", 1, imageCallback);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  ros::spin();
  cv::destroyWindow("inRange");
  return 0;
}
