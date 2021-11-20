#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "geometry_msgs/Twist.h"
#include <ros/console.h>
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

//Pergunta 1
#define MAX_FEATURES 100


bool init = true;  //flag that is true only at the beginning
ros::Publisher pub;
geometry_msgs::Twist vel;
std::vector<std::vector<cv::Point>> polyCurves;
std::string color, shape, lastcolor = " ", lastshape = " ";
int siz;

std::string detectShape(cv::Mat input){
  std::string type;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(input, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
  //cv::Moments m = cv::moments(input);
  //cv::Point p(m.m10/m.m00, m.m01/m.m00);

  for (int i = 0; i < contours.size(); i++) {
      std::vector<cv::Point> current;
      cv::approxPolyDP(contours[i], current, 1, true);

      if(current.size() > 0){
          polyCurves.push_back(current);
      }
  }
  //cv::drawContours(imagem, polyCurves, -1, cv::Scalar(0, 255, 0), 1);
  for(int i = 0; i<polyCurves.size(); i++){
    siz = polyCurves[i].size();
    if(siz > 4){
      bool k = cv::isContourConvex(polyCurves[i]);
      if(k){
        type = "circle";
      }else{
        type = "cross";
      }
    }else{
      type = "triangle";
    }
  }
  return type;
}



// Callback
void imageLeftCB(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    
    cv::Mat result_green, green_contour, result_blue, blue_contour, result_red, red_contour, result_white, white_contour;
    cv::Mat hsv_image, gray_image;
    cv::Mat maskG, maskR, maskB, maskW;
    cv::Mat image_final, bwfind, mask_platform, platform;
    cv::Mat imagem = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    
    //detect marker platform
    cv::cvtColor(imagem, hsv_image, cv::COLOR_BGR2HSV);
    cv::cvtColor(imagem, gray_image, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_image, mask_platform, 15, 255, cv::THRESH_BINARY_INV);

    cv::bitwise_and(hsv_image, hsv_image, platform, mask_platform = mask_platform);

    //create color masks
    cv::inRange(hsv_image, cv::Scalar(40, 40, 40), cv::Scalar(70, 255, 255), maskG); //detect green colour
    cv::inRange(hsv_image, cv::Scalar(0, 50, 20), cv::Scalar(5, 255, 255), maskR); //detect red colour
    cv::inRange(hsv_image, cv::Scalar(94, 80, 2), cv::Scalar(120, 255, 255), maskB); //detect blue colour
    cv::inRange(hsv_image, cv::Scalar(0,0,0), cv::Scalar(0,0,255), maskW); //detect white
    
    cv::Mat maskWflood = maskW.clone();
    cv::floodFill(maskWflood, cv::Point(0,0), cv::Scalar(255));
    cv::Mat maskWflood_inv;
    cv::bitwise_not(maskWflood, maskWflood_inv);
    cv::Mat im_out = (maskW | maskWflood_inv);
    std::string color;
    
    //detect color
    cv::bitwise_and(hsv_image, hsv_image, platform, im_out = im_out);

    cv::bitwise_and(platform, platform, result_green, maskG = maskG);
    cv::bitwise_and(platform, platform, result_red, maskR = maskR);
    cv::bitwise_and(platform, platform, result_blue, maskB = maskB);

    std::vector<cv::Mat> hsv_planes_g, hsv_planes_b, hsv_planes_r, hsv_planes_w;
    cv::split(result_green, hsv_planes_g);
    cv::split(result_red, hsv_planes_r);
    cv::split(result_blue, hsv_planes_b);
    //hsv_planes[0] // H channel
    //hsv_planes[1] // S channel
    //hsv_planes[2] // V channel

    //detect shape

    //detect green
    if (cv::countNonZero(hsv_planes_g[1]) != 0){
      color = "green";
      lastcolor = color;
      shape = detectShape(hsv_planes_g[1]);
      lastshape = shape;
    }else if(cv::countNonZero(hsv_planes_r[1]) != 0){
      color = "red";
      lastcolor = color;
      shape = detectShape(hsv_planes_r[1]);
      lastshape = shape;
    }else if(cv::countNonZero(hsv_planes_b[1]) != 0){
      color = "blue";
      lastcolor = color;
      shape = detectShape(hsv_planes_b[1]);
      lastshape = shape;
    }else{
      color = lastcolor;
      shape = lastshape;
    }
    cv::putText(imagem, color + " " + shape , cv::Point(20, 20) ,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
  /* if(p.x > 300){ //camara esta à direita do verde
    vel.linear.x=0.0;
    vel.angular.z=-1.0;//Publish the message. 
    }else{*/
      vel.linear.x=0.0;
      vel.angular.z=4.0;//Publish the message. 
  //  }
     pub.publish(vel);  

    
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
  ros::init(argc, argv, "camera_subscriber");
  ros::NodeHandle nh;
  cv::namedWindow("inRange");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_left = it.subscribe("camera/left/image_raw", 1, imageLeftCB);
  
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  ros::spin();
  cv::destroyWindow("inRange");
  return 0;
}