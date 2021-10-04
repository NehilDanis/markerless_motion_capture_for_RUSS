#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>

std::vector<Eigen::VectorXd> read_coordinates(std::string file_name) {
  std::string s;
  std::vector<Eigen::VectorXd> return_points;
  std::ifstream scan_poses_file (file_name);
  if (scan_poses_file.is_open())
  {
    while ( getline (scan_poses_file,s) )
    {
      Eigen::VectorXd pointPose(3);
      int ind = 0;
      // parse the string
      std::string delimiter = " ";
      size_t pos = 0;
      std::string token;
      while ((pos = s.find(delimiter)) != std::string::npos) {
          token = s.substr(0, pos);
          pointPose(ind) =  std::stod(token);
          s.erase(0, pos + delimiter.length());
          ind += 1;
      }
      pointPose(ind) = std::stod(s);
      return_points.push_back(pointPose);
    }
    scan_poses_file.close();
  }
  return return_points;

}

int main(void)
{

    std::vector<Eigen::VectorXd> img_1 = read_coordinates("/home/nehil/catkin_ws_registration/src/tracking_results/aruco_in_pos_2.txt");
    std::vector<Eigen::VectorXd> img_2 = read_coordinates("/home/nehil/catkin_ws_registration/src/tracking_results/aruco_in_pos_3.txt");
    std::cout << img_1.size() << std::endl;
    std::cout << img_2.size() << std::endl;
    double distance_x = 0;
    double distance_y = 0;
    double distance_z = 0;
    double distance = 0;
    for(unsigned int i = 0; i < img_1.size(); i++) {
      distance_x += std::abs((img_2[i].x() - img_1[i].x()));
      distance_y += std::abs((img_2[i].y() - img_1[i].y()));
      distance_z += (img_2[i].z() - img_1[i].z());
      auto point_dist = std::sqrt(std::pow(img_2[i].x() - img_1[i].x(), 2) + std::pow(img_2[i].y() - img_1[i].y(), 2) + std::pow(img_2[i].z() - img_1[i].z(), 2)) - 206.0f;
      std::cout << i + 1 << " : " << point_dist << std::endl;
      distance += point_dist;
    }
    //std::cout << "Diff in x: " << distance_x/img_1.size() << "\nDiff in y: " << distance_y/img_1.size() << "\nDiff in z: " << distance_z/img_1.size() << "\n";
    std::cout << "Euclidean distance: " << distance/img_1.size() << std::endl;
    return 0;
}
