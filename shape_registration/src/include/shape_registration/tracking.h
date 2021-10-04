#ifndef TRACKING_H
#define TRACKING_H
/*******************************************************************
* ROS RELATED
*******************************************************************/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/*******************************************************************
* PCL RELATED
*******************************************************************/

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

/*******************************************************************
* CUSTOM
*******************************************************************/
#include "preprocessing.hpp"
#include "icp_algorithm.hpp"

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class MovementTracker{
public:
  MovementTracker(ros::NodeHandle *nh);

private:
  /**
   * @brief calculate_trasformation callback
   * @param prev_cloud_msg
   * @param curr_cloud_msg
   * @param prev_img_msg
   * @param curr_img_msg
   * @param prev_img_depth_msg
   * @param curr_img_depth_msg
   * @param cam_info_msg
   */
  void calculate_trasformation(const sensor_msgs::PointCloud2ConstPtr& prev_cloud_msg,
                               const sensor_msgs::PointCloud2ConstPtr& curr_cloud_msg,
                               const sensor_msgs::ImageConstPtr& prev_img_msg,
                               const sensor_msgs::ImageConstPtr& curr_img_msg,
                               const sensor_msgs::ImageConstPtr& prev_img_depth_msg,
                               const sensor_msgs::ImageConstPtr& curr_img_depth_msg,
                               const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

  /**
   * @brief calculate_rotation creates 4x4 rotation matrix from geometry message.
   * @param t_stamped geometry msg
   * @return 4x4 transformation matrix
   */
  Eigen::Matrix4d calculate_rotation(geometry_msgs::TransformStamped t_stamped);

  /**
   * @brief find_corners finds the corners on the chessboard
   * @param cv_ptr_color
   * @param cv_ptr_depth
   * @param cam_info_msg
   * @return
   */
  std::vector<Eigen::Vector4d> find_chessboard_corners(cv_bridge::CvImageConstPtr cv_ptr_color, cv_bridge::CvImageConstPtr cv_ptr_depth, const sensor_msgs::CameraInfoConstPtr& cam_info_msg);
  /**
   * @brief project_detected_corners_to_world_frame
   * @param image
   * @param corners_in_image_frame
   * @param corners_in_world_frame
   * @param cam_info_msg
   */
  void project_detected_corners_to_world_frame(const cv::Mat& image, const std::vector<cv::Point2f> &corners_in_image_frame, std::vector<Eigen::Vector4d> & corners_in_world_frame, const sensor_msgs::CameraInfoConstPtr& cam_info_msg);
  /**
   * @brief find_aruco_corners
   * @param cv_ptr_color
   * @param cv_ptr_depth
   * @param cam_info_msg
   * @return
   */
  std::vector<Eigen::Vector4d> find_aruco_corners(cv_bridge::CvImageConstPtr cv_ptr_color, cv_bridge::CvImageConstPtr cv_ptr_depth, const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

  /**
   * @brief calculate_error
   * @param prev_img_msg
   * @param curr_img_msg
   * @param prev_img_depth_msg
   * @param curr_img_depth_msg
   * @param cam_info_msg
   * @param movement_transform
   */
  void calculate_error(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform);

  void calculate_error_using_marker(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform);
private:
  double m_voxel_size;
  double m_plane_seg_threshold;
  int m_aruco_marker_id;
  int m_board_x;
  int m_board_y;
  geometry_msgs::TransformStamped m_transformStamped;
  Eigen::Matrix4d m_transformation_to_robot_base;
  std::shared_ptr<ICPAlgorithm> m_icp;
  ros::Publisher m_pub_transformation;

  ros::Publisher m_pub_cloud_1;
  ros::Publisher m_pub_cloud_2;
  ros::Publisher m_pub_cloud_3;

  ros::Publisher m_movement_start_pub;
  ros::Publisher m_movement_end_pub;
  std::string m_error_check_method;
  message_filters::Subscriber<sensor_msgs::PointCloud2> m_prev_cloud_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> m_curr_cloud_sub;
  message_filters::Subscriber<sensor_msgs::Image> m_prev_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> m_curr_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> m_prev_img_depth_sub;
  message_filters::Subscriber<sensor_msgs::Image> m_curr_img_depth_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> m_cam_info_sub;
  std::string m_calibration_file_path;
  int m_max_iterations;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;

};

#endif // TRACKING_H
