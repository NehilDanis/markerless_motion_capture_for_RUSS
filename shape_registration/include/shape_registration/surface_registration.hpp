#ifndef SURFACE_REGISTRATION_HPP
#define SURFACE_REGISTRATION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>


#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"

#include <vector>

using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;


class SurfaceRegistration
{
public:
  /**
   * @brief SurfaceRegistration
   * @param nh
   */
  SurfaceRegistration(ros::NodeHandle *nh);

private:
  /**
   * @brief compute
   * @param ros_cloud
   */
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);

  /**
   * @brief sets the transformation from camera to the robot base using config file
   * @param file_path full path to the config yaml file
   */
  void set_transformation_to_robot_base(const std::string &file_path);

  /**
   * @brief preprocessing of the preoperative data
   * @param limb_surface_file_path full path to te limb surface point cloud
   * @param artery_surface_file_path full path to the scan artery point cloud
   */
  void set_preoperative_pointclouds(const std::string &limb_surface_file_path, const std::string &artery_surface_file_path);

  /**
   * @brief Moving the two point clouds together by moving their centers to each other
   * @param source source point cloud ptr
   * @param target target point cloud ptr
   */
  void move_source_and_target_closer(PointCloudT::Ptr &source, const PointCloudT::Ptr &target, PointCloudT::Ptr &artery);
private:
  float m_voxel_size;
  std::shared_ptr<ICPAlgorithm> m_shape_registration;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;
  ros::Publisher m_pub_transformed_source;
  ros::Publisher m_pub_artery;
  ros::Publisher m_pub_artery_robot_base;
  ros::Publisher m_pub_source;
  ros::Publisher m_pub_source_keypoint;
  ros::Publisher m_pub_target;
  ros::Publisher m_pub_target_keypoint;
  std::string m_data_path;
  std::string m_artery_data_path;
  PointCloudT m_source_cloud;
  PointCloudT m_ct_cloud_normal;
  PointCloudT m_target_cloud;
  PointCloudT m_artery_cloud;
  geometry_msgs::TransformStamped m_transformStamped;
  Eigen::Matrix4d m_transformation_to_robot_base;
  std::string m_catkin_dir_path;
};

#endif // SURFACE_REGISTRATION_HPP
