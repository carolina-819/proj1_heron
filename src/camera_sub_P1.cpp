#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <ros/console.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
// cannot be used opencv_contrib::xfeatures not installed in VM
//#include <opencv2/xfeatures2d.hpp>  

//Pergunta 1
#define MAX_FEATURES 100


bool init = false;  //flag that requests camera
ros::Publisher pub, pub_type_left, pub_type_right, pub_centered;
geometry_msgs::Twist vel;
std::vector<std::vector<cv::Point>> polyCurves;
std::string color, colorR, colorL, lastshape = " ", formaL, formaR;;
std_msgs::Int8 mensagemL, mensagemR;
cv::Mat marker_color;
int siz,  lastcolor;
int state = -1;
std_msgs::Float32 aux;

std::string detectShape(cv::Mat input){
  std::string shape;
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
  
  for(int i = 0; i<polyCurves.size(); i++){
    siz = polyCurves[i].size();
    if(siz > 4){
      bool k = cv::isContourConvex(polyCurves[i]);
      if(k){
        shape = "circle";
      }else{
        shape = "cross";
      }
    }else{
      shape = "triangle";
    }
  }
  return shape;
}

std::tuple<bool, float> isShapeCentered(cv::Mat input){
  cv::Moments m = cv::moments(input);
  cv::Point p(m.m10/m.m00, m.m01/m.m00);
  if (p.x < 350 && p.x > 250){
    return std::tuple<bool, float>{true, p.x};
  }else{
    return std::tuple<bool, float>{false, p.x};
  }

}

int detectColor(cv::Mat red, cv::Mat green, cv::Mat blue){
  if(cv::countNonZero(red)){
    color = "red";
    marker_color = red;
    return 1;
  }else if(cv::countNonZero(green)){
    color = "green";
    marker_color = green;
    return 2;
  }else if(cv::countNonZero(blue)){
    color = "blue";
    marker_color = blue;
    return 3;
  }else{
    return 0;
  }
}

int shapeToInt(std::string sh){
  if(sh == "circle"){
    return 1;
  }else if(sh == "triangle"){
    return 2;
  }else if(sh == "cross"){
    return 3;
  }else{
    return -1;
  }
}

// Callback
void imageLeftCB(const sensor_msgs::ImageConstPtr& msg)
{
  
  int corL, corR;
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

    //Found a marker

    int o = detectColor(hsv_planes_r[1], hsv_planes_g[1], hsv_planes_b[1]);
    
    if(std::get<0>(isShapeCentered(marker_color))){
      aux.data=std::get<1>(isShapeCentered(marker_color));
      pub_centered.publish(aux);
    }
    else{
      aux.data=std::get<1>(isShapeCentered(marker_color));
      pub_centered.publish(aux);
    }
    //ROS_INFO("cor %d ", o);
    
   if(init){

    if(state == -1){
      mensagemL.data = -1;
      mensagemR.data = -1;
    }
    if(state == -1 && init == true){ //requested identification
      state = 0;
    }
    else if(state == 0 && o != 0){ //found markers
      state = 1;
      lastcolor = detectColor(hsv_planes_r[1], hsv_planes_g[1], hsv_planes_b[1]);
      colorL = color;
    }else if(state == 1 && std::get<0>(isShapeCentered(marker_color))){ //centered marker
       //identified marker
        corL = lastcolor;
        formaL = detectShape(marker_color);
        mensagemL.data = (corL * 10) + shapeToInt(formaL);
        //cv::drawContours(imagem, polyCurves, -1, cv::Scalar(0, 255, 0), 1);
        state = 2;
    }else if(state == 2 && o == 0){ //screen blank
      state = 3;
    }else if(state == 3 && o != 0){ //found right marker
      lastcolor = detectColor(hsv_planes_r[1], hsv_planes_g[1], hsv_planes_b[1]);
      colorR = color;
      state = 4;
    }else if(state == 4 && std::get<0>(isShapeCentered(marker_color))){ //right marker centered
        corR = lastcolor;
        formaR = detectShape(marker_color);
        mensagemR.data = (corR * 10) + shapeToInt(formaR);
      state = 5;
      init == false;
    }

    switch(state){
      case -1:
        break;
      case 0:
        vel.linear.x=0.0;
        vel.angular.z=4.0;
        pub.publish(vel);
        break;
      case 1:
        vel.linear.x=0.0;
        vel.angular.z=2.0;
        pub.publish(vel);
        break;
      case 2:  
        vel.linear.x=0.0;
        vel.angular.z=-4.0;
        pub.publish(vel);
        cv::putText(imagem, "left marker: " + formaL + " " + colorL, cv::Point(20, 20) ,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,0),0.5,false);
        break;
      case 3:
        vel.linear.x=0.0;
        vel.angular.z=-4.0;
        pub.publish(vel);
        cv::putText(imagem, "left marker: " + formaL + " " + colorL, cv::Point(20, 20) ,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,0),0.5,false);
        break;
      case 4:
        vel.linear.x=0.0;
        vel.angular.z=-2.0;
        pub.publish(vel);
        cv::putText(imagem, "left marker: " + formaL + " " + colorL, cv::Point(20, 20) ,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,0),0.5,false);
        break;
      case 5: 
        vel.linear.x=0.0;
        vel.angular.z=0.0;
        pub.publish(vel);  
        cv::putText(imagem, "left marker: " + formaL + " " + colorL, cv::Point(20, 20) ,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,0),0.5,false);
        //cv::drawContours(imagem, polyCurves, -1, cv::Scalar(255, 0, 0), 1);
        cv::putText(imagem, "right marker: " + formaR + " " + colorR, cv::Point(20, 50) ,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,0),0.5,false);
        break;
     
    }
    pub_type_left.publish(mensagemL);
    pub_type_right.publish(mensagemR);
   
   
    cv::imshow("inRange", imagem);

    cv::waitKey(30);
   }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  
}

void flagCB(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data){
    init = msg->data;
  }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_subscriber");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Subscriber flagReq;
  image_transport::Subscriber sub_left;
  cv::namedWindow("inRange");
  //boost::shared_ptr<std_msgs::Bool const> flagReq;
  std_msgs::Bool flag;
  
  
  flagReq = nh.subscribe("/camera_request", 1, flagCB);
  ros::spinOnce();
  sub_left = it.subscribe("camera/left/image_raw", 1, imageLeftCB);
  pub_centered = nh.advertise<std_msgs::Float32>("/centered", 1);
  pub_type_left = nh.advertise<std_msgs::Int8>("/marker_shape_left",1);
  pub_type_right = nh.advertise<std_msgs::Int8>("/marker_shape_right",1);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  
  
 
  
  
  
  
  
  //flag_pub = nh.advertise<std_msgs::Bool>("/camera_request", 1);

  

  ros::spin();
  cv::destroyWindow("inRange");
  
  return 0;
}