/*******************************************************************
* OPENCV RELATED
*******************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>
#include <yaml-cpp/yaml.h>
#include "shape_registration/tracking.h"

namespace  {
cv::Point pt_img_1;
cv::Point pt_img_2;
}

MovementTracker::MovementTracker(ros::NodeHandle *nh) {

  nh->getParam("movement_monitoring/error_check_method", this->m_error_check_method);
  nh->getParam("movement_monitoring/voxel_grid_filter_voxel_size", this->m_voxel_size);
  nh->getParam("movement_monitoring/plane_segmentation_threshold", this->m_plane_seg_threshold);
  nh->getParam("movement_monitoring/board_x", this->m_board_x);
  nh->getParam("movement_monitoring/board_y", this->m_board_y);
  nh->getParam("movement_monitoring/marker_id", this->m_aruco_marker_id);
  nh->getParam("movement_monitoring/calibration_file_path", m_calibration_file_path);
  nh->getParam("movement_monitoring/icp_max_num_of_iterations", m_max_iterations);

  this->m_icp= std::make_shared<ICPAlgorithm>(m_max_iterations);
  YAML::Node config = YAML::LoadFile(m_calibration_file_path);
  YAML::Node attributes = config["transformation"];
  m_transformStamped.transform.rotation.x = attributes["qx"].as<double>();
  m_transformStamped.transform.rotation.y = attributes["qy"].as<double>();
  m_transformStamped.transform.rotation.z = attributes["qz"].as<double>();
  m_transformStamped.transform.rotation.w = attributes["qw"].as<double>();

  m_transformStamped.transform.translation.x = attributes["x"].as<double>();
  m_transformStamped.transform.translation.y = attributes["y"].as<double>();
  m_transformStamped.transform.translation.z = attributes["z"].as<double>();

  this->m_transformation_to_robot_base = calculate_rotation(this->m_transformStamped);

  sync.reset(new Sync(sync_pol(5), m_prev_cloud_sub, m_curr_cloud_sub, m_prev_img_sub, m_curr_img_sub, m_prev_img_depth_sub, m_curr_img_depth_sub, m_cam_info_sub));
  sync->registerCallback(boost::bind(&MovementTracker::calculate_trasformation, this, _1, _2, _3, _4, _5, _6, _7));

  m_pub_transformation = nh->advertise<geometry_msgs::TransformStamped>("/transformation", 1);

  m_pub_cloud_1 = nh->advertise<sensor_msgs::PointCloud2>("/cloud_1", 1);
  m_pub_cloud_2 = nh->advertise<sensor_msgs::PointCloud2>("/cloud_2", 1);
  m_pub_cloud_3 = nh->advertise<sensor_msgs::PointCloud2>("/cloud_result", 1);

  m_movement_start_pub = nh->advertise<sensor_msgs::PointCloud2>("/movement_start_plane_segmented", 1);
  m_movement_end_pub = nh->advertise<sensor_msgs::PointCloud2>("/movement_end_plane_segmented", 1);

  m_prev_cloud_sub.subscribe(*nh, "movement_start", 1);
  m_curr_cloud_sub.subscribe(*nh, "movement_end", 1);

  m_prev_img_sub.subscribe(*nh, "movement_start_img", 1);
  m_curr_img_sub.subscribe(*nh, "movement_end_img", 1);

  m_prev_img_depth_sub.subscribe(*nh, "movement_start_depth", 1);
  m_curr_img_depth_sub.subscribe(*nh, "movement_end_depth", 1);

  m_cam_info_sub.subscribe(*nh, "cam_info_move", 1);

}

Eigen::Matrix4d MovementTracker::calculate_rotation(geometry_msgs::TransformStamped t_stamped) {
  double x = t_stamped.transform.translation.x;
  double y = t_stamped.transform.translation.y;
  double z = t_stamped.transform.translation.z;

  double qw = t_stamped.transform.rotation.w;
  double qx = t_stamped.transform.rotation.x;
  double qy = t_stamped.transform.rotation.y;
  double qz = t_stamped.transform.rotation.z;

  Eigen::Matrix4d result;
  result << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy), x,
      2 * (qx*qy + qw*qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy*qz - qw*qx), y,
      2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
      0                                          , 0                   , 0                 , 1;

  return result;
}

static void onMouse(int event, int x, int y, int, void*) {

  if(event==cv::EVENT_LBUTTONDOWN)
    {
      pt_img_1.x=x;
      pt_img_1.y=y;
    }
    if(event==cv::EVENT_RBUTTONDOWN)
    {
      pt_img_2.x=x;
      pt_img_2.y=y;
    }
}

void MovementTracker::project_detected_corners_to_world_frame(const cv::Mat& image, const std::vector<cv::Point2f> &corners_in_image_frame, std::vector<Eigen::Vector4d> & corners_in_world_frame, const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));


  for(std::size_t i = 0; i < corners_in_image_frame.size(); i++)
  {
      auto depth = image.at<float>(static_cast<int>(corners_in_image_frame[i].y), static_cast<int>(corners_in_image_frame[i].x));
      Eigen::Vector4d tempPoint((corners_in_image_frame[i].x - cx) * depth / fx,
                                (corners_in_image_frame[i].y - cy) * depth / fy,
                                depth,
                                1.0);
      corners_in_world_frame.push_back(tempPoint);
  }
}


std::vector<Eigen::Vector4d> MovementTracker::find_chessboard_corners(cv_bridge::CvImageConstPtr cv_ptr_color, cv_bridge::CvImageConstPtr cv_ptr_depth, const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
  auto board_size = cv::Size(this->m_board_x, this->m_board_y);

  //Detect corners of chessboard
  cv::Mat chessboard = cv_ptr_color->image;
  cv::Mat Extractcorner = chessboard.clone();
  std::vector<cv::Point2f> corners;
  cv::Mat imageGray;
  cv::cvtColor(Extractcorner, imageGray, CV_RGB2GRAY);

  bool patternfound = cv::findChessboardCorners(Extractcorner, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

  if(!patternfound)
  {
      std::cout << "can not find chessboard corners!" << std::endl;
      exit(1);
  }
  else
  {
      ROS_INFO_STREAM("FOUND");
      cv::cornerSubPix(imageGray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));
  }
  std::vector<Eigen::Vector4d> detected_corners;
  this->project_detected_corners_to_world_frame(cv_ptr_depth->image, corners, detected_corners, cam_info_msg);
  return detected_corners;
}

std::vector<Eigen::Vector4d> MovementTracker::find_aruco_corners(cv_bridge::CvImageConstPtr cv_ptr_color, cv_bridge::CvImageConstPtr cv_ptr_depth, const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
  cv::Mat imageGray;
  cv::cvtColor(cv_ptr_color->image, imageGray, CV_RGB2GRAY);
  std::vector<Eigen::Vector4d> detected_corners;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(imageGray, dictionary, corners, ids);
  // if at least one marker detected
  if (ids.size() > 0) {
      for (unsigned int i = 0; i < corners.size(); i++) {
        if(ids[i] == this->m_aruco_marker_id){
          this->project_detected_corners_to_world_frame(cv_ptr_depth->image, corners[i], detected_corners, cam_info_msg);
        }
      }
  }
  return detected_corners;
}

// the function will ask user to select a point and its correspondence on both prev and curr images
// then the error will be calculated according to this point.
void MovementTracker::calculate_error(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform) {
  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));


  auto prev = cv_bridge::toCvCopy(prev_img_msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("view", prev);
  cv::setMouseCallback("view", onMouse);
  cv::waitKey(0);
  cv::destroyWindow("view");

  auto curr = cv_bridge::toCvCopy(curr_img_msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("view2", curr);
  cv::setMouseCallback("view2", onMouse);
  cv::waitKey(0);
  cv::destroyWindow("view2");
  std::cout << pt_img_1 << std::endl;
  std::cout << pt_img_2 << std::endl;

  auto prev_depth = cv_bridge::toCvCopy(prev_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto curr_depth = cv_bridge::toCvCopy(curr_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

  pcl::PointCloud<pcl::PointXYZ> cloud1;

  pcl::PointCloud<pcl::PointXYZ> cloud2;
  pcl::PointCloud<pcl::PointXYZ> cloudresult;

  float depth_img_1 = prev_depth->image.at<float>(static_cast<int>(pt_img_1.y), static_cast<int>(pt_img_1.x));
  float depth_img_2 = curr_depth->image.at<float>(static_cast<int>(pt_img_2.y), static_cast<int>(pt_img_2.x));
  if(depth_img_1 != 0.0f && depth_img_2 != 0.0f) {
    pcl::PointXYZ pt_1;
    pcl::PointXYZ pt_2;
    pcl::PointXYZ pt_result;
    pt_1.x = depth_img_1 * (int(pt_img_1.x) - cx) / fx;
    pt_1.y = depth_img_1 * (int(pt_img_1.y) - cy) / fy;
    pt_1.z = depth_img_1;

    pt_2.x = depth_img_2 * (int(pt_img_2.x) - cx) / fx;
    pt_2.y = depth_img_2 * (int(pt_img_2.y) - cy) / fy;
    pt_2.z = depth_img_2;

    cloud1.push_back(pt_1);
    cloud2.push_back(pt_2);
    cloudresult.push_back(pt_result);

    Eigen::Vector4d u(pt_1.x, pt_1.y, pt_1.z, 1.0f);
    auto result = movement_transform * u;
    pt_result.x = result.x();
    pt_result.y = result.y();
    pt_result.z = result.z();
    std::cout << "Selected point: " <<  pt_2 << std::endl;
    std::cout << "Transformed point: " << pt_result << std::endl;


    // find euclidean distance

    auto distance = std::sqrt(std::pow(pt_2.x - pt_result.x, 2) + std::pow(pt_2.y - pt_result.y, 2) + std::pow(pt_2.z - pt_result.z, 2));

    std::cout << "Error: " << distance *  1000 << " mm" << std::endl;

    sensor_msgs::PointCloud2 msg_1;
    pcl::toROSMsg(cloud1, msg_1);
    msg_1.header.frame_id = "rgb_camera_link";
    msg_1.header.stamp = ros::Time::now();
    m_pub_cloud_1.publish(msg_1);

    sensor_msgs::PointCloud2 msg_2;
    pcl::toROSMsg(cloud2, msg_2);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_2.publish(msg_2);

    sensor_msgs::PointCloud2 msg_3;
    pcl::toROSMsg(cloudresult, msg_3);
    msg_3.header.frame_id = "rgb_camera_link";
    msg_3.header.stamp = ros::Time::now();
    m_pub_cloud_3.publish(msg_2);
  }
}

void MovementTracker::calculate_error_using_marker(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform) {
  auto prev_depth_img = cv_bridge::toCvCopy(prev_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto curr_depth_img = cv_bridge::toCvCopy(curr_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto prev_img = cv_bridge::toCvCopy(prev_img_msg, sensor_msgs::image_encodings::BGR8);
  auto curr_img = cv_bridge::toCvCopy(curr_img_msg, sensor_msgs::image_encodings::BGR8);

  std::vector<Eigen::Vector4d> detected_corners_prev_frame;
  std::vector<Eigen::Vector4d> detected_corners_curr_frame;

  if(this->m_error_check_method == "aruco") {
    detected_corners_prev_frame = find_aruco_corners(prev_img, prev_depth_img, cam_info_msg);
    detected_corners_curr_frame = find_aruco_corners(curr_img, curr_depth_img, cam_info_msg);

  }
  else if(this->m_error_check_method == "chessboard") {
    detected_corners_prev_frame = find_chessboard_corners(prev_img, prev_depth_img, cam_info_msg);
    detected_corners_curr_frame = find_chessboard_corners(curr_img, curr_depth_img, cam_info_msg);
  }

  PointCloudT::Ptr marker_1_cloud (new PointCloudT);
  PointCloudT::Ptr marker_2_cloud (new PointCloudT);
  PointCloudT::Ptr transformed_cloud (new PointCloudT);


  if(detected_corners_curr_frame.size() == detected_corners_prev_frame.size()) {
    auto error = 0.0;

    for (unsigned int i = 0; i < detected_corners_curr_frame.size(); i++) {
      auto prev_corner = detected_corners_prev_frame[i];
      auto curr_corner = detected_corners_curr_frame[i];
      pcl::PointXYZ prev_p;
      prev_p.x  = prev_corner.x();
      prev_p.y  = prev_corner.y();
      prev_p.z  = prev_corner.z();
      pcl::PointXYZ curr_p;
      curr_p.x  = curr_corner.x();
      curr_p.y  = curr_corner.y();
      curr_p.z  = curr_corner.z();
      marker_1_cloud->points.push_back(prev_p);
      marker_2_cloud->points.push_back(curr_p);
    }

    //move checkerboard points to the robot base.
    pcl::transformPointCloud(*marker_1_cloud, *marker_1_cloud, m_transformation_to_robot_base);
    pcl::transformPointCloud(*marker_2_cloud, *marker_2_cloud, m_transformation_to_robot_base);

    pcl::transformPointCloud(*marker_1_cloud, *transformed_cloud, movement_transform);

    marker_1_cloud->width = marker_2_cloud->width = transformed_cloud->width = detected_corners_curr_frame.size();
    marker_1_cloud->height = marker_2_cloud->height = transformed_cloud->height = 1;

    for(unsigned int i = 0; i < marker_1_cloud->points.size(); i ++) {

      // find euclidean distance
      pcl::PointXYZ pt_2 = marker_2_cloud->points[i];
      pcl::PointXYZ pt_result = transformed_cloud->points[i];
      auto distance = std::sqrt(std::pow(pt_2.x - pt_result.x, 2) + std::pow(pt_2.y - pt_result.y, 2) + std::pow(pt_2.z - pt_result.z, 2));
      std::cout << "Point " << i + 1 << std::endl;
      std::cout << "Selected point: " <<  pt_2 << std::endl;
      std::cout << "Transformed point: " << pt_result << std::endl;
      std::cout << "Distance: " << distance * 1000 << " mm" << std::endl;

      error += distance;
    }

    error /= marker_1_cloud->points.size();
    std::cout << "Error: " << error * 1000 << " mm" << std::endl;

    sensor_msgs::PointCloud2 msg_1;
    pcl::toROSMsg(*marker_1_cloud, msg_1);
    msg_1.header.frame_id = "rgb_camera_link";
    msg_1.header.stamp = ros::Time::now();
    m_pub_cloud_1.publish(msg_1);

    sensor_msgs::PointCloud2 msg_2;
    pcl::toROSMsg(*marker_2_cloud, msg_2);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_2.publish(msg_2);

    sensor_msgs::PointCloud2 msg_3;
    pcl::toROSMsg(*transformed_cloud, msg_3);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_3.publish(msg_3);

  }

}


void MovementTracker::calculate_trasformation(const sensor_msgs::PointCloud2ConstPtr& prev_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& curr_cloud_msg, const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
  PointCloudT::Ptr prev_ptr (new PointCloudT);
  PointCloudT::Ptr curr_ptr (new PointCloudT);

  pcl::fromROSMsg(*prev_cloud_msg, *prev_ptr);
  pcl::fromROSMsg(*curr_cloud_msg, *curr_ptr);

  // remove the plane from the point cloud

  prev_ptr = Preprocessing::extract_plane(prev_ptr, this->m_plane_seg_threshold);
  curr_ptr = Preprocessing::extract_plane(curr_ptr, this->m_plane_seg_threshold);

  prev_ptr = Preprocessing::voxel_grid_downsampling(prev_ptr, this->m_voxel_size);
  prev_ptr = Preprocessing::statistical_filtering(prev_ptr, 1.0);

  curr_ptr = Preprocessing::voxel_grid_downsampling(curr_ptr, this->m_voxel_size);
  curr_ptr = Preprocessing::statistical_filtering(curr_ptr, 1.0);

  // move the curr position to the robot base
  pcl::transformPointCloud(*curr_ptr, *curr_ptr, m_transformation_to_robot_base);
  // move the prev position to the robot base
  pcl::transformPointCloud(*prev_ptr, *prev_ptr, m_transformation_to_robot_base);


//  float avg_depth = 0.0f;
//  for (const auto &point : curr_ptr->points) {
//    avg_depth += point.z;
//  }
//  avg_depth = avg_depth / curr_ptr->points.size();

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_curr (new pcl::PointCloud<pcl::PointXYZ>);
//  for (const auto& point : curr_ptr->points){
//    if(point.z < 1.7 * avg_depth)
//      cloud_cluster_curr->push_back (point); //*
//  }

//  avg_depth = 0.0f;
//  for (const auto &point : prev_ptr->points) {
//    avg_depth += point.z;
//  }
//  avg_depth = avg_depth / prev_ptr->points.size();

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_prev (new pcl::PointCloud<pcl::PointXYZ>);
//  for (const auto& point : prev_ptr->points){
//    if(point.z < 1.7 * avg_depth)
//      cloud_cluster_prev->push_back (point); //*
//  }

  // once both curr and prev frames are set then apply icp and find the transformation between the two frames
//  m_icp->compute(cloud_cluster_prev, cloud_cluster_curr);
  m_icp->compute(prev_ptr, curr_ptr);

  auto transformation = m_icp->get_ICP_obj().getFinalTransformation();

  Eigen::Quaternionf q(transformation.block<3,3>(0,0));
  geometry_msgs::TransformStamped movement;
  movement.header = prev_cloud_msg->header;

  movement.transform.translation.x = transformation(0, 3);
  movement.transform.translation.y = transformation(1, 3);
  movement.transform.translation.z = transformation(2, 3);
  movement.transform.rotation.x = q.x();
  movement.transform.rotation.y = q.y();
  movement.transform.rotation.z = q.z();
  movement.transform.rotation.w = q.w();

  Eigen::Matrix4d movement_transform;
  movement_transform = this->calculate_rotation(movement);

  std::cout << "movement: " << movement_transform << std::endl;
  m_pub_transformation.publish(movement);


  if (this->m_error_check_method == "selection") {
    calculate_error(prev_img_msg, curr_img_msg, prev_img_depth_msg, curr_img_depth_msg, cam_info_msg, movement_transform);
  }
  else{
    // using either aruco marker or chessboard checker board
    calculate_error_using_marker(prev_img_msg, curr_img_msg, prev_img_depth_msg, curr_img_depth_msg, cam_info_msg, movement_transform);
  }

  sensor_msgs::PointCloud2 msg_start;
  pcl::toROSMsg(*prev_ptr, msg_start);
  msg_start.header.frame_id = "rgb_camera_link";
  msg_start.header.stamp = ros::Time::now();
  m_movement_start_pub.publish(msg_start);

  sensor_msgs::PointCloud2 msg_stop;
  pcl::toROSMsg(*curr_ptr, msg_stop);
  msg_stop.header.frame_id = "rgb_camera_link";
  msg_stop.header.stamp = ros::Time::now();
  m_movement_end_pub.publish(msg_stop);

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh;
  MovementTracker tracker(&nh);
  ros::spin();
}
