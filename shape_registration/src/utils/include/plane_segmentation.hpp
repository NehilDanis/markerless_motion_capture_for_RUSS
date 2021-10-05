#ifndef PLANE_SEGMENTATION_HPP
#define PLANE_SEGMENTATION_HPP

#include <iostream>
#include "preprocessing.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>


class PlaneSegmentation
{
public:
  /**
   * @brief PlaneSegmentation
   * @param nh
   */
  PlaneSegmentation(ros::NodeHandle *nh);

private:
  /**
   * @brief compute is a callback function, removes the plane from the pointcloud data
   * @param ros_cloud
   */
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);

  void calculate_rotation();

private:
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  double m_threshold_for_CT_plane_seg;
  double m_threshold_for_RGBD_plane_seg;
  std::string m_CT_arm_input_path;
  std::string m_segmented_CT_arm_output_path;
  std::string m_CT_artery_input_path;
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Matrix4d transformation_to_robot_base;

};

#endif // PLANE_SEGMENTATION_HPP
