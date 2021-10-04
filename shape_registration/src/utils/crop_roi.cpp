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

/**
 The user will crop an area given the rgb image from the camera. After the ROI is selected by the user, the point cloud of that are will be generated
 using information from both the RGB and depth image.
 The resulted point cloud will be published to a ROS topic called /points_created.

 To use this file in your launch file, you need to give the topic names to receive the RGB and depth data from the camera, along with the topic names to
 get the camera information for the depth image.
 */

namespace  {
  bool box_selected = false;
  bool leftDown=false,leftup=false;
  cv::Point cor1;
  cv::Point cor2;
  cv::Rect box;
  cv::Mat m_img;
  cv::Mat depth_img;
  sensor_msgs::PointCloud2 msg_n;
  ros::Publisher m_pub;
}

static void onMouse(int event, int x, int y, int, void*) {

  if(event==cv::EVENT_LBUTTONDOWN)
    {
      leftDown=true;
      cor1.x=x;
      cor1.y=y;
    }
    if(event==cv::EVENT_LBUTTONUP)
    {
      leftup=true;
      cor2.x=x;
      cor2.y=y;
    }

    if(leftDown==true&&leftup==true) //when the selection is done
    {
      std::cout << "Cor1 : " << cor1 << std::endl;
      std::cout << "Cor2 : " << cor2 << std::endl;
      box.width=abs(cor1.x-cor2.x);
      box.height=abs(cor1.y-cor2.y);
      box.x=std::min(cor1.x,cor2.x);
      box.y=std::min(cor1.y,cor2.y);
      cv::Mat crop(m_img, box); //Selecting a ROI(region of interest) from the original pic
      //cv::imshow("Cropped Image",crop); //showing the cropped image
      //cv::waitKey(0);
      cv::rectangle(m_img, cor1, cor2, cv::Scalar(0, 255, 0), 5);
      //cv::imshow("view", m_img);
      leftDown=false;
      leftup=false;
      box_selected = true;
      //cv::destroyWindow("Cropped Image");
      return;
    }
}

void crop_rgb_image(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));
  if(!box_selected){
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
  try
  {
    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = cv_ptr_depth->image.clone();

    pcl::PointCloud<pcl::PointXYZ> cloud;

    int cloud_id = 0;
    for (int i = int(cor1.x); i < int(cor2.x); i ++) {
      for (int j = int(cor1.y); j < int(cor2.y); j ++) {
        float depth = cv_ptr_depth->image.at<float>(static_cast<int>(j), static_cast<int>(i));
        if(depth != 0.0f) {
          //depth -= 0.025f;
          pcl::PointXYZ cloud_point;
          cloud_point.x = depth * (i - cx) / fx;
          cloud_point.y = depth * (j - cy) / fy;
          cloud_point.z = depth;
          std::cout << depth << std::endl;
          cloud.push_back(cloud_point);
          cloud_id +=1;
        }
      }
    }
    pcl::toROSMsg(cloud, msg_n);
    msg_n.header.frame_id = "rgb_camera_link";
    msg_n.header.stamp = ros::Time::now();
    m_pub.publish(msg_n);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgra8'.", depth_msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  cv::namedWindow("view", 0);
  cv::resizeWindow("view", 1080, 1920);

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  message_filters::Subscriber<sensor_msgs::Image> color_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;

  depth_sub.subscribe(nh, "/depth_to_rgb_image", 1);
  color_sub.subscribe(nh, "/rgb_image", 1);
  cam_info_sub.subscribe(nh, "/camera_info", 1);
  sync.reset(new Sync(sync_pol(5),color_sub, depth_sub, cam_info_sub));
  sync->registerCallback(boost::bind(crop_rgb_image, _1, _2, _3));

  m_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_created", 1);

  ros::spin();
  return 0;

}
