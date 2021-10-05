/**
  STD
 **/

#include <iostream>
#include <string>
#include <vector>
#include <cmath>


/**
  ROS RELATED
 **/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>


/**
  PCL RELATED
 **/
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include "preprocessing.hpp"
#include <pcl/filters/extract_indices.h>

/**
  EIGEN RELATED
 **/

#include <Eigen/Core>

const std::string BASE_LINK = "iiwa_link_0";
const std::string EE_LINK = "iiwa_link_ee";


using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

void optimize_normals(PointCloudT::Ptr &trajectory, pcl::PointCloud<pcl::Normal>::Ptr &trajectory_normals) {
  Eigen::Vector3f gt_normal(0, 0, -1);
  for(unsigned int i = 0; i < trajectory->size(); i++) {
    // go thru the elements in the trajectory and optimize the normals by looking at the previous and current normal
    auto normal = trajectory_normals->points[i].getNormalVector3fMap();
    if(normal[2] > 0) {
      normal[0] *= -1;
      normal[1] *= -1;
      normal[2] *= -1;
    }


    // look at the normals around it
    unsigned int first_point;
    unsigned int second_point;
    if(i == trajectory->size() - 1) {
      first_point = i - 2;
      second_point = i - 1;
    }
    else if (i == 0) {
      first_point = i + 1;
      second_point = i + 2;
    }
    else {
      first_point = i - 1;
      second_point = i + 1;
    }

    auto first_normal = trajectory_normals->points[first_point].getNormalVector3fMap();
    auto second_normal = trajectory_normals->points[second_point].getNormalVector3fMap();
    auto curr_normal = trajectory_normals->points[i].getNormalVector3fMap();
    auto cos_theta = first_normal.dot(curr_normal) /
        (first_normal.norm() * curr_normal.norm());
    auto theta_between_first_curr =  acos(cos_theta) * 180.0 / M_PI;

    cos_theta = first_normal.dot(second_normal) /
        (first_normal.norm() * second_normal.norm());
    auto theta_between_first_second =  acos(cos_theta) * 180.0 / M_PI;

    if(theta_between_first_curr > 10.0) {
      // then first and curr has a large difference
      if(theta_between_first_second < 10.0) {
        std::cout << "heyy" << std::endl;
        // then the curr point has an issue, lets change its normal
        trajectory_normals->points[i]._Normal::normal_x = (first_normal.x() + second_normal.x()) / 2;
        trajectory_normals->points[i]._Normal::normal_y = (first_normal.x() + second_normal.y()) / 2;
        trajectory_normals->points[i]._Normal::normal_z = (first_normal.x() + second_normal.z()) / 2;
      }

    }

  }
}

Eigen::Quaternionf get_rotation(Eigen::Vector3f x, Eigen::Vector3f z){
  if(z[2] > 0) {
    z[2] = - z[2];
    z[1] = - z[1];
    z[0] = - z[0];
  }

  x = x.normalized();
  z = z.normalized();

  Eigen::Vector3f y = (z.cross(x)).normalized();


  Eigen::Matrix3f pose_rotm;
  pose_rotm << x, y, z;
  Eigen::Quaternionf q(pose_rotm);
  return q.normalized();
}

PointCloudT project_trajectory_onto_surface_method2(const PointCloudT::Ptr &trajectory, const PointCloudT::Ptr & arm_cloud, const pcl::PointCloud<pcl::Normal>::Ptr &trajectory_normals, pcl::PointCloud<pcl::Normal>::Ptr &new_normals){
  // if two points in the trajectory are on almost the same direction, then we don't need to add all the points

  PointCloudT new_trajectory;

  Eigen::Vector3f prev_direction;
  Eigen::Vector3f curr_direction;
  bool defined_prev_dir = false;
  for(size_t i = 0; i < trajectory->points.size() - 1; i ++) {
    auto curr_point = trajectory->points[i];
    auto curr_normal = trajectory_normals->points[i];
    auto next_point = trajectory->points[i + 1];
    curr_direction = next_point.getArray3fMap() - curr_point.getArray3fMap();
    if(!defined_prev_dir) {
      // add the first point of the trajectory
      prev_direction = curr_direction;
      new_trajectory.points.push_back(curr_point);
      new_normals->points.push_back(curr_normal);
      defined_prev_dir = true;
    }
    else{
      auto cos_theta = curr_direction.dot(prev_direction) /
          (curr_direction.norm() * prev_direction.norm());
      auto theta =  acos(cos_theta) * 180.0 / M_PI;
      if(theta > 30 && theta <= 40) {
        // add the curr point to the trajectory
        new_trajectory.points.push_back(curr_point);
        new_normals->points.push_back(curr_normal);
      }
    }
  }

  // add the last point of the trajectory
  new_trajectory.points.push_back(trajectory->points[trajectory->points.size()-1]);
  new_normals->points.push_back(trajectory_normals->points[trajectory_normals->points.size() - 1]);
  new_normals->height = 1;
  new_normals->width = new_normals->points.size();
  return new_trajectory;
}


/**
 * @brief find_trajectory_from_p_cloud in this function, the trajectory is created from the point cloud of the artery.
 * @param artery_cloud artery to create the trajectory from
 * @param transformed_cloud generated trajectory will be written here
 * @return
 */
PointCloudT find_trajectory_from_p_cloud(const PointCloudT::Ptr &artery_cloud, PointCloudT::Ptr &transformed_cloud){

  /**
  find the PCA of the artery cloud and then project it to the
  eigen vector space.
  **/

  // find the centroid of the original cloud
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*artery_cloud, pcaCentroid);


  pcl::PCA<PointT> cpca = new pcl::PCA<PointT>;
  cpca.setInputCloud(artery_cloud);
  cpca.project(*artery_cloud, *transformed_cloud); // original cloud is projected to the eigen vector space
  Eigen::Matrix3f eigenVectorsPCA = cpca.getEigenVectors();
  Eigen::Vector3f eigenValuesPCA = cpca.getEigenValues();

  /**
    Project the eigen vectors to the eigen space as well
  **/

  // rotation matrx to go to eigen vector space
//  Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
//  tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
//  tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t

  /**
   X DIRECTION SHOWS THE LARGEST EIGEN VALUE DIRECTION
   START FROM SMALLEST X VALUE AND GO USING SOME INTERVAL
   next_point = curr_point + interval
   **/

   PointT min_p, max_p;
   pcl::getMinMax3D(*transformed_cloud, min_p, max_p);

   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
   kdtree.setInputCloud(transformed_cloud);
   auto result_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

   // every 2 cm we look for the knn to find the point
   for(float start = min_p.x; start < max_p.x; start += 0.02f) {

     pcl::PointXYZ searchPoint;

     searchPoint.x = start;
     searchPoint.y = 0;
     searchPoint.z = 0;

     // K nearest neighbor search

     int K = 5;

     std::vector<int> pointIdxKNNSearch(K);
     std::vector<float> pointKNNSquaredDistance(K);

     /*std::cout << "K nearest neighbor search at (" << searchPoint.x
               << " " << searchPoint.y
               << " " << searchPoint.z
               << ") with K=" << K << std::endl;*/

     if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
     {
       pcl::PointXYZ result_point;
       result_point.x = 0;
       result_point.y = 0;
       result_point.z = 0;
       for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i) {
         result_point.x += (*transformed_cloud)[ pointIdxKNNSearch[i] ].x;
         result_point.y += (*transformed_cloud)[ pointIdxKNNSearch[i] ].y;
         result_point.z += (*transformed_cloud)[ pointIdxKNNSearch[i] ].z;

       }
       result_point.x  = result_point.x / pointIdxKNNSearch.size ();
       result_point.y  = result_point.y / pointIdxKNNSearch.size ();
       result_point.z  = result_point.z / pointIdxKNNSearch.size ();
       result_cloud->points.push_back(result_point);
     }
   }
   // project the resulting trajectory to its old place
   cpca.reconstruct(*result_cloud, *result_cloud);

   // After the projection some points in the trajectory have exactly the same coordinates.
   // Below code is to get rid of them
   PointCloudT trajectory;
   size_t ind = 0;
   PointT prev_point;
   for(const auto &point : result_cloud->points) {
     if(ind == 0 ) {
       trajectory.points.push_back(point);
     }
     else {
       if(std::abs(prev_point.x - point.x) > 1e-4f || std::abs(prev_point.y - point.y) > 1e-4f || std::abs(prev_point.z - point.z) > 1e-4f) {
         // this check is necessary to make sure not to add the same point two times.
          trajectory.points.push_back(point);
       }
     }
     prev_point = point;
     ind  += 1;
   }
   return trajectory;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "find_trajectory");
  ros::NodeHandle nh;
  std::string catkin_directory_path;
  nh.getParam("trajectory_extraction/catkin_directory_path", catkin_directory_path);
  /**
    READ THE ARM AND ARTERY POINT CLOUDS
   **/

  auto artery_cloud = std::make_shared<PointCloudT>();
  auto artery_cloud_cam_base = std::make_shared<PointCloudT>();
  auto arm_cloud = std::make_shared<PointCloudT>();
  auto transformed_cloud = std::make_shared<PointCloudT>();

  if (pcl::io::loadPCDFile<PointT> (catkin_directory_path + "src/artery_downsampled_robot_base.pcd", *artery_cloud) == -1) //* load the artery
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  if (pcl::io::loadPCDFile<PointT> (catkin_directory_path + "src/arm_downsampled_robot_base.pcd", *arm_cloud) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  /**
    FIND THE TRAJECTORY FROM THE ARTERY POINT CLOUD
   **/

   auto trajectory = std::make_shared<PointCloudT>(find_trajectory_from_p_cloud(artery_cloud, transformed_cloud));

   /**
     PROJECT THE TRAJECTORY TO THE ARM SURFACE AND RECOVER THE NORMAL DIRECTIONS TO DECIDE THE ORIENTATION OF THE PROBE
    **/

   // For now lets project it directly up

   pcl::PointXYZ minPt, maxPt;
   pcl::getMinMax3D (*arm_cloud, minPt, maxPt);
   float max_z  = maxPt.z;

   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_n;
   kdtree_n.setInputCloud(arm_cloud);
   auto trajectory_projected = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

   auto indices_to_extract = pcl::make_shared<std::vector<int>>();

   std::vector<geometry_msgs::PoseStamped> poses;
   PointT prev_point;
   PointT first_point;
   first_point.x = trajectory->points[0].x;
   first_point.y = trajectory->points[0].y;
   first_point.z = max_z;


   int K_1 = 2;
   std::vector<int> pointIdxKNNSearch_1(K_1);
   std::vector<float> pointKNNSquaredDistance_1(K_1);

   size_t num_neighbors_1 = kdtree_n.nearestKSearch (first_point, K_1, pointIdxKNNSearch_1, pointKNNSquaredDistance_1);
   max_z = (*arm_cloud)[ std::size_t(pointIdxKNNSearch_1[0])].z;

   for(const auto &point : *trajectory) {
     PointT searchPoint;
     int K = 5;
     searchPoint.x = point.x;
     searchPoint.y = point.y;
     searchPoint.z = max_z;
     std::vector<int> pointIdxKNNSearch(K);
     std::vector<float> pointKNNSquaredDistance(K);

     size_t num_neighbors = kdtree_n.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
     //std::cout << num_neighbors << std::endl;

     if ( num_neighbors > 0 )
     {
       auto avg_depth = 0.0f;
       for(int ind : pointIdxKNNSearch) {
         avg_depth += (*arm_cloud)[ std::size_t(ind)].z;
       }
       avg_depth /= num_neighbors;
       auto inner_circle_avg_depth = 0.0f;
       auto variance = 0.002f;
       unsigned int inner_circle_count = 0;
       for(int ind : pointIdxKNNSearch) {
         if(std::abs((*arm_cloud)[ std::size_t(ind)].z - avg_depth) < variance) {
           inner_circle_avg_depth += (*arm_cloud)[ std::size_t(ind)].z;
           inner_circle_count += 1;
         }
       }
       if(inner_circle_count == 0) {
         inner_circle_avg_depth = avg_depth;
       }
       else{
         inner_circle_avg_depth /= inner_circle_count;
       }
       PointT new_point;
       (*arm_cloud)[ std::size_t(0) ].x = searchPoint.x;
       (*arm_cloud)[ std::size_t(0) ].y = searchPoint.y;
       (*arm_cloud)[ std::size_t(0) ].z = inner_circle_avg_depth;
       trajectory_projected->points.push_back((*arm_cloud)[ std::size_t(0)]);
     }
   }


   /**
     FIND THE NORMAL DIRECTION OF THE TRAJECTORY POINTS AND WRITE THEM INTO A TEXT FILE TO USE TO MOVE THE ROBOT
   **/


   // Create the normal estimation class, and pass the input dataset to it
   pcl::NormalEstimation<PointT, pcl::Normal> ne;
   ne.setInputCloud (trajectory_projected);

   // Pass the original data (before downsampling) as the search surface
   ne.setSearchSurface (arm_cloud);

   // Create an empty kdtree representation, and pass it to the normal estimation object.
   // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
   ne.setSearchMethod (tree);

   // Output datasets
   pcl::PointCloud<pcl::Normal>::Ptr trajectory_normals (new pcl::PointCloud<pcl::Normal>);

   // Use all neighbors in a sphere of radius 2cm
   // TODO can be also 3 cm
   ne.setRadiusSearch (0.03);

   // Compute the features
   ne.compute (*trajectory_normals);

   std::cout << trajectory_projected->points.size() << std::endl;
   std::cout << trajectory_normals->points.size() << std::endl;

   optimize_normals(trajectory_projected, trajectory_normals);

//   pcl::PointCloud<pcl::Normal>::Ptr new_normals (new pcl::PointCloud<pcl::Normal>);

//   trajectory_projected = std::make_shared<PointCloudT>(project_trajectory_onto_surface_method2(trajectory_projected, arm_cloud, trajectory_normals, new_normals));
//   trajectory_normals = new_normals;

   std::cout << trajectory_projected->points.size() << std::endl;
   std::cout << trajectory_normals->points.size() << std::endl;

//   for(auto &point : trajectory_normals->points) {
//     point._Normal::normal_x *= -1;
//     point._Normal::normal_y *= -1;
//     point._Normal::normal_z *= -1;
//   }

   // visualize normals
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (1, 1, 1);
   pcl::visualization::PointCloudColorHandlerCustom<PointT> arm_handler(arm_cloud, 0, 0, 255);
   viewer.addPointCloud(arm_cloud, arm_handler, "trajectory_cloud");
   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(trajectory_projected, trajectory_normals, 1, 0.03f, "normals");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "normals");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "trajectory_cloud");



   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }


   /**
     FIND THE POSES
   **/

   ofstream myfile (catkin_directory_path + "src/artery_in_robot_base.txt");
   if (!myfile.is_open()) {
    std::cout << "ERROR" << std::endl;
   }

   for(size_t i = 1; i < trajectory_projected->size(); i++) {
     const auto &prev_point = trajectory_projected->points[i-1];
     const auto &curr_point = trajectory_projected->points[i];

     Eigen::Vector3f direction = curr_point.getArray3fMap() - prev_point.getArray3fMap();
     Eigen::Quaternionf q = get_rotation(direction, trajectory_normals->points[i].getNormalVector3fMap());
     myfile << prev_point.x << " " << prev_point.y << " " << prev_point.z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
   }

   myfile.close();

   // Extract inliers
   pcl::ExtractIndices<PointT> extract;
   extract.setInputCloud(arm_cloud);
   extract.setIndices(indices_to_extract);
   extract.setNegative(true);     // Extract the inliers
   extract.filter(*arm_cloud); // cloud_inliers contains the plane

  return 0;
}
