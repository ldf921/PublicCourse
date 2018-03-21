// Copyright @2018 Pony AI Inc. All rights reserved.

#include <iostream>
#include <algorithm>
#include <vector>
#include "homework2/pointcloud.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <Eigen/Core>

int main() {
  // ATTENTION!!! : please use absolute path for reading the data file.
  const PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/miu/PublicCourse/homework2/sample_data/VelodyneDevice32c/0.txt");
  std::cout << "Total points read: " << pointcloud.points.size() << std::endl;
  std::cout << "Rotation: " << std::endl;
  std::cout << pointcloud.rotation << std::endl;
  std::cout << "Translation: " << std::endl;
  std::cout << pointcloud.translation.transpose() << std::endl;

  std::vector<double> heights;
  for(const auto &p : pointcloud.points)
  {
    heights.push_back(p.z());
  }
  std::sort(heights.begin(), heights.end());
  // std::cout << "(z<0) " << c << std::endl;
  double h0=-3, r = 0.1;
  int i = 0, j;
  for(; i < heights.size(); h0 += r, i = j)
  {
    for(j = i; j < heights.size() && heights[j] < h0 + r; j++);

    std::cout << heights[j] << ' ' << j - i << std::endl;
  }

  int H = 1024, W = 1024;
  int cx = W / 2, cy = H / 2;
  cv::Mat image = cv::Mat::zeros(H, W, CV_8UC3);
  int fx = 20, fy = 20;
  double pi = 3.1415926536;
  for(const auto &p : pointcloud.points)
    {
      Eigen::Vector3d project_x(1, 0, 0);
      double rot = pi/10;
      Eigen::Vector3d project_y(0, cos(rot), sin(rot));
      Eigen::Vector3d project_z(0, sin(rot), -cos(rot)); 
      Eigen::Vector3d camera(0, -5, 10);
      Eigen::Vector3d d = p - camera;
      if (d.dot(project_z) > 0)
      {
        double x = d.dot(project_x) / d.dot(project_z) * fx;
        double y = d.dot(project_y) / d.dot(project_z) * fy;
        int px = static_cast<int>(round(cy - y));
        int py = static_cast<int>(round(cx + x));
        if (0 <= px && px < W && 0 <= py && py < H)
        {
          if (p.z() > - 0.8) {
            image.at<cv::Vec3b>(px, py) = cv::Vec3b(0, 255, 0);
          }
          else ;
            // image.at<cv::Vec3b>(px, py) = cv::Vec3b(0, 0, 255);
        }
      }
    }
  cv::namedWindow("point_cloud");
  cv::imshow("point_cloud", image);
  cv::waitKey(0);
  cv::imwrite("/home/miu/PublicCourse/homework2/pointcloud.jpg", image);
  return 0;
}
