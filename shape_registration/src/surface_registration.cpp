#include "shape_registration/surface_registration.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <string>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "shape_registration/utils/helper.hpp"



SurfaceRegistration::SurfaceRegistration(ros::NodeHandle *nh)
{
  // required parameters
  int max_num_iter;
  std::string calibration_file_path;
  std::string sub_topic_name;
  nh->getParam("icp_registration/voxel_grid_filter_voxel_size", m_voxel_size);
  nh->getParam("icp_registration/icp_max_num_of_iterations", max_num_iter);
  nh->getParam("icp_registration/ct_arm_data_path", m_data_path);
  nh->getParam("icp_registration/ct_artery_data_path", m_artery_data_path);
  nh->getParam("icp_registration/calibration_file_path", calibration_file_path);
  nh->getParam("icp_registration/subscribe_to", sub_topic_name);
  nh->getParam("icp_registration/catkin_directory_path", m_catkin_dir_path);

  // set robot base transformation
  set_transformation_to_robot_base(calibration_file_path);


  // Create ICP algorithm object

  this->m_shape_registration = std::make_shared<ICPAlgorithm>(max_num_iter);

  // load preoperative data
  // limb and artery surface
  set_preoperative_pointclouds(m_data_path, m_artery_data_path);


  // initialize publishers
  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/final_result", 30);
  this->m_pub_transformed_source = nh->advertise<sensor_msgs::PointCloud2>("/transformed_source", 30);
  this->m_pub_artery = nh->advertise<sensor_msgs::PointCloud2>("/artery", 30);
  this->m_pub_artery_robot_base = nh->advertise<sensor_msgs::PointCloud2>("/artery_robot_base", 30);
  this->m_pub_source = nh->advertise<sensor_msgs::PointCloud2>("/source", 30);
  this->m_pub_source_keypoint = nh->advertise<sensor_msgs::PointCloud2>("/source_keypoint", 30);
  this->m_pub_target = nh->advertise<sensor_msgs::PointCloud2>("/target", 30);
  this->m_pub_target_keypoint = nh->advertise<sensor_msgs::PointCloud2>("/target_keypoint", 30);

  // node is subscribed to plane segmented point cloud data from RGB-D cam.
  this->m_sub = nh->subscribe(sub_topic_name, 30, &SurfaceRegistration::compute, this);

}

void SurfaceRegistration::set_preoperative_pointclouds(const std::string &limb_surface_file_path, const std::string &artery_surface_file_path) {
  // Load both the CT arm and artery surface point clouds
  PointCloudT::Ptr limb_surface_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (limb_surface_file_path, *limb_surface_cloud) == -1) //* load the ct arm cloud file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  PointCloudT::Ptr cloud_artery (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (artery_surface_file_path, *cloud_artery) == -1) //* load ct artery cloud file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  // The scale difference between the Azure kinect camera and the CT is 1000
  // below every point of CT data is divided by 1000 to get the same scale of data with the camera.

  auto num_points = (limb_surface_cloud->size() > cloud_artery->size()) ? limb_surface_cloud->size() : cloud_artery->size();

  for(size_t i = 0; i < num_points; i++) {
    if(!(i < limb_surface_cloud->points.size())) continue;
    else{
      limb_surface_cloud->points[i].x = limb_surface_cloud->points[i].x / 1000;
      limb_surface_cloud->points[i].y = limb_surface_cloud->points[i].y / 1000;
      limb_surface_cloud->points[i].z = limb_surface_cloud->points[i].z / 1000;
    }
    if(!(i < cloud_artery->points.size())) continue;
    else{
      cloud_artery->points[i].x = cloud_artery->points[i].x / 1000;
      cloud_artery->points[i].y = cloud_artery->points[i].y / 1000;
      cloud_artery->points[i].z = cloud_artery->points[i].z / 1000;
    }
  }

  limb_surface_cloud = Preprocessing::voxel_grid_downsampling(limb_surface_cloud, 0.015f);
  limb_surface_cloud = Preprocessing::statistical_filtering(limb_surface_cloud, 1.5);


  this->m_artery_cloud = *cloud_artery;
  this->m_source_cloud = *limb_surface_cloud;

  pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "src/arm_downsampled_ct.pcd", *limb_surface_cloud);
  pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "src/artery.pcd", *cloud_artery);
}

void SurfaceRegistration::move_source_and_target_closer(PointCloudT::Ptr &source, const PointCloudT::Ptr &target, PointCloudT::Ptr &artery) {
  //// This section moves the source center closer to the target center
  PointT centroid_s;
  pcl::computeCentroid(*source, centroid_s);

  PointT centroid_t;
  pcl::computeCentroid(*target, centroid_t);

  PointT diff;
  diff.x = centroid_t.x - centroid_s.x;
  diff.y = centroid_t.y - centroid_s.y;
  diff.z = centroid_t.z - centroid_s.z;

  PointCloudT::Ptr temp_source = source;
  PointCloudT::Ptr temp_target = target;

  for (auto &point : source->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }

  for (auto &point : artery->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }
}


void SurfaceRegistration::set_transformation_to_robot_base(const std::string &calibration_file_path){
  YAML::Node config = YAML::LoadFile(calibration_file_path);
  YAML::Node attributes = config["transformation"];
  Eigen::Quaterniond q;
  Eigen::Vector3d t;

  q.x() = attributes["qx"].as<double>();
  q.y() = attributes["qy"].as<double>();
  q.z() = attributes["qz"].as<double>();
  q.w() = attributes["qw"].as<double>();

  t.x() = attributes["x"].as<double>();
  t.y() = attributes["y"].as<double>();
  t.z() = attributes["z"].as<double>();

  auto rotation_mat = Utils::from_quaternion_to_rot(q);
  this->m_transformation_to_robot_base = Utils::from_rotation_translation_to_transformation(rotation_mat, t);
}

void SurfaceRegistration::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {

  // the transformed artery needs to be saved only once,
  static size_t saved_flag = 0;

  // Plane segmented point cloud from RGB-D cam
  PointCloudT::Ptr target (new PointCloudT);

  PointCloudT::Ptr target_to_save (new PointCloudT);

  // Create the source
  // CT Arm
  PointCloudT::Ptr source (new PointCloudT);
  *source = this->m_source_cloud;

  // Create the artery
  PointCloudT::Ptr artery (new PointCloudT);
  *artery = this->m_artery_cloud;

  pcl::fromROSMsg(*ros_cloud, *target);
  pcl::fromROSMsg(*ros_cloud, *target_to_save);


  // the distance between source and target point clouds is calculated, and the limb and artery point clouds
  // are shifted accordingly.
  move_source_and_target_closer(source, target, artery);

  // find the initial transformation between the source and target point clouds,
  // using the
  this->m_shape_registration->get_initial_transformation(source, target);

  /**
   Transform the source point cloud given the alignment
   */

   pcl::transformPointCloud(*source, *source, this->m_shape_registration->transformation);

   pcl::transformPointCloud(*artery, *artery, this->m_shape_registration->transformation);


   // publish the CT cloud which got closer to the target by using the initial alignment
   sensor_msgs::PointCloud2 msg;
   pcl::toROSMsg(*source, msg);
   msg.fields = ros_cloud->fields;
   msg.header = ros_cloud->header;
   this->m_pub_transformed_source.publish(msg);


   // the ICP algorithm is employed to find the registration between initally aligned source and target
   PointCloudT final_cloud = this->m_shape_registration->compute(source, target, 0.0003);

   // the ICP algorithm is set to return an empty cloud if the registration quality is bad.
   // here the final cloud is checked whether it is empty or not
   if (final_cloud.size() != 0) {

     // if the registration quality is good then the artery will also be transformed to the target
     // point cloud frame
     pcl::transformPointCloud(*artery, *artery, this->m_shape_registration->get_ICP_obj().getFinalTransformation());

     // Save the points to pcd file.

     pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "arm_transformed.pcd", final_cloud);
     pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "arm_camera.pcd", *target);
     pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "src/artery_in_cam.pcd", *artery);

     // publish the transformed source and artery clouds
     sensor_msgs::PointCloud2 msg_final;
     pcl::toROSMsg(final_cloud, msg_final);
     msg_final.fields = ros_cloud->fields;
     msg_final.header = ros_cloud->header;
     this->m_pub.publish(msg_final);

     sensor_msgs::PointCloud2 msg_artery;
     pcl::toROSMsg(*artery, msg_artery);
     msg_artery.fields = ros_cloud->fields;
     msg_artery.header = ros_cloud->header;
     this->m_pub_artery.publish(msg_artery);

     if(saved_flag == 0) {
       // Transform the artery to the robot base coordinates later to use for finding scan trajectory
       auto artery_in_robot_base = std::make_shared<PointCloudT>();
       pcl::transformPointCloud(*artery, *artery_in_robot_base, this->m_transformation_to_robot_base);

       // transform the limb cloud to the robot base and save the point cloud
       target_to_save->width = target_to_save->points.size();
       target_to_save->height = 1;
       auto arm_in_robot_base = std::make_shared<PointCloudT>();
       pcl::transformPointCloud(*target_to_save, *arm_in_robot_base, this->m_transformation_to_robot_base);

       // transform the ct cloud into robot base
       auto ct_in_robot_base = std::make_shared<PointCloudT>();
       pcl::transformPointCloud(final_cloud, *ct_in_robot_base, this->m_transformation_to_robot_base);

       // Save the points to pcd file.
       pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "src/arm_downsampled_robot_base.pcd", *arm_in_robot_base);
       pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "src/artery_downsampled_robot_base.pcd", *artery_in_robot_base);
       pcl::io::savePCDFileASCII (this->m_catkin_dir_path + "src/ct_in_robot_base.pcd", *ct_in_robot_base);


       saved_flag += 1;
     }
   }
}

int main(int argc, char** argv) {
  // Initialize the surface registration node
  ros::init(argc, argv, "surface_registration_node");
  ROS_INFO("Initialized Surface Registration Node");
  ros::NodeHandle n;
  SurfaceRegistration registration_algorithm(&n);
  ros::spin();
}
