/*******************************************************************
* ROS RELATED
*******************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*******************************************************************
* OPENCV RELATED
*******************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/*******************************************************************
* PCL RELATED
*******************************************************************/
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*******************************************************************
* STD INCLUDES
*******************************************************************/
#include <iostream>
#include <cmath>


/***
 * This file is used to check the depth values, the user needs to draw a line on the given image
 * using the mouse, and the depth values along the line will be printed.
 * */

namespace  {
  bool _line = false;
  bool leftDown = true;
  bool leftup = true;
  cv::Point start_point;
  cv::Point end_point;
  cv::Mat m_img;
}


static void onMouse(int event, int x, int y, int, void*) {

  if(event==cv::EVENT_LBUTTONDOWN)
    {
      leftDown=true;
      start_point.x=x;
      start_point.y=y;
    }
    if(event==cv::EVENT_LBUTTONUP)
    {
      leftup=true;
      end_point.x=x;
      end_point.y=y;
    }

    if(leftDown==true&&leftup==true) //when the selection is done
    {
      std::cout << "Start point : " << start_point << std::endl;
      std::cout << "End point : " << end_point << std::endl;
      leftDown=false;
      leftup=false;
      _line = true;
      return;
    }
}


void compute(sensor_msgs::ImageConstPtr color_msg, sensor_msgs::ImageConstPtr depth_msg) {

  if(!_line){
    try
    {
      m_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imshow("view", m_img);
      cv::setMouseCallback("view", onMouse);
      cv::waitKey(0);
      cv::destroyWindow("view");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgra8'.", color_msg->encoding.c_str());
    }
  }
  else{
    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::LineIterator it(m_img, start_point, end_point);
    cv::LineIterator it2 = it;

    for(int i = 0; i < it2.count; i++, ++it2)
    {
      auto curr_pixel = it2.pos();
      float depth = cv_ptr_depth->image.at<float>(static_cast<int>(curr_pixel.y), static_cast<int>(curr_pixel.x));
      std::cout << depth << std::endl;
      //std::cout << curr_pixel << std::endl;
    }
  }

  std::cout << "---------------------------------" << std::endl;
}


int main(int argc, char **argv){
  cv::namedWindow("view", 0);
  cv::resizeWindow("view", 1080, 1920);

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  message_filters::Subscriber<sensor_msgs::Image> color_sub;

  depth_sub.subscribe(nh, "/depth_to_rgb_image", 1);
  color_sub.subscribe(nh, "/rgb_image", 1);
  sync.reset(new Sync(sync_pol(5),color_sub, depth_sub));
  sync->registerCallback(boost::bind(compute, _1, _2));

  ros::spin();
  return 0;
}
