#include "plane_segmentation.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

PlaneSegmentation::PlaneSegmentation(ros::NodeHandle *nh)
{
  nh->getParam("plane_segmentation/plane_segmentation_threshold_for_CT", m_threshold_for_CT_plane_seg);
  nh->getParam("plane_segmentation/plane_segmentation_threshold_for_RGBD", m_threshold_for_RGBD_plane_seg);
  nh->getParam("plane_segmentation/input_path_arm_data", m_CT_arm_input_path);
  nh->getParam("plane_segmentation/output_path_segmented_arm_data", m_segmented_CT_arm_output_path);
  nh->getParam("plane_segmentation/input_path_artery_data", m_CT_artery_input_path);
  nh->getParam("plane_segmentation/calibration_file_path", m_calibration_file_path);

  // read the calibration data from yaml file

  YAML::Node config = YAML::LoadFile(m_calibration_file_path);
  YAML::Node attributes = config["transformation"];

  transformStamped.transform.rotation.x = attributes["qx"].as<double>();
  transformStamped.transform.rotation.y = attributes["qy"].as<double>();
  transformStamped.transform.rotation.z = attributes["qz"].as<double>();
  transformStamped.transform.rotation.w = attributes["qw"].as<double>();

  transformStamped.transform.translation.x = attributes["x"].as<double>();
  transformStamped.transform.translation.y = attributes["y"].as<double>();
  transformStamped.transform.translation.z = attributes["z"].as<double>();

  calculate_rotation();


  /*PointCloudT::Ptr cloud (new PointCloudT);
  PointCloudT::Ptr extracted_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_CT_arm_input_path, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }

  extracted_cloud = Preprocessing::extract_plane(cloud, m_threshold_for_CT_plane_seg);
  pcl::io::savePCDFile(m_segmented_CT_arm_output_path, *extracted_cloud);*/

  this->m_sub = nh->subscribe("/points_created", 1, &PlaneSegmentation::compute, this);
  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/plane_segmented_data", 1);

}

void PlaneSegmentation::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
  static bool file_saved = false;
  ROS_INFO("Reached the plane segmentation level !!");
  PointCloudT::Ptr pcl_cloud (new PointCloudT);
  PointCloudT::Ptr segmented_cloud (new PointCloudT);
  pcl::fromROSMsg(*ros_cloud, *pcl_cloud);

  ROS_INFO("Created cloud");
  segmented_cloud = Preprocessing::extract_plane(pcl_cloud, m_threshold_for_RGBD_plane_seg);

  ROS_INFO("extracted plane from cloud");

  segmented_cloud = Preprocessing::voxel_grid_downsampling(segmented_cloud, 0.015f);
  segmented_cloud = Preprocessing::statistical_filtering(segmented_cloud, 1.0);

  //ROS_INFO("applied statistical filter");
  if(segmented_cloud != nullptr && !segmented_cloud->points.empty()) {
    if(!file_saved) {

      segmented_cloud->width = segmented_cloud->points.size();
      segmented_cloud->height = 1;
      pcl::transformPointCloud(*segmented_cloud, *segmented_cloud, this->transformation_to_robot_base);

      // Save the points to pcd file.
      pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/segmented_cloud_in_robot_base.pcd", *segmented_cloud);
      file_saved = true;


    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*segmented_cloud, msg);
    msg.fields = ros_cloud->fields;
    msg.header = ros_cloud->header;
    this->m_pub.publish(msg);
  }
  else {
    ROS_INFO("No plane is found!");
  }
  ROS_INFO("Finished plane segmentation!");
}

void PlaneSegmentation::calculate_rotation(){
  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  double z = transformStamped.transform.translation.z;

  double qw = transformStamped.transform.rotation.w;
  double qx = transformStamped.transform.rotation.x;
  double qy = transformStamped.transform.rotation.y;
  double qz = transformStamped.transform.rotation.z;

  this->transformation_to_robot_base << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy), x,
                                  2 * (qx*qy + qw*qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy*qz - qw*qx), y,
                                  2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
                                  0                                          , 0                   , 0                 , 1;
}


int main(int argc, char** argv) {
  // Initialize the plane segmentation node
  // This node will segment the planes in the point clouds and subtract them
  ros::init(argc, argv, "plane_segmentation_node");
  ROS_INFO("Initialized Plane Sementation Node");
  ros::NodeHandle n;
  PlaneSegmentation plane_segmnentation_obj(&n);
  ros::spin();
}

