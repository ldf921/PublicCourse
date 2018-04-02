// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/pointcloud/pointcloud.h"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int coord(double x) 
{
    return static_cast<int>(std::round(x));
}

cv::Mat normalize(cv::Mat img)
{
    cv::Mat result;
    img.convertTo(result, CV_8U, 255.0 / 28.0, 255.0);
    return result;
}


struct Stat
{
    double min_value, max_value;

    Stat() 
    {
        min_value = +1e20;
        max_value = -1e20;
    }

    double min() const
    {
        return min_value;
    }

    double max() const
    {
        return max_value;
    }

    void update(double value) {
        min_value = std::min(value, min_value);
        max_value = std::max(value, max_value);
    }
};

cv::Mat build_image(std::vector<Eigen::Vector3d> &points, double threshold)
{
  int H = 1024, W = 1024;
  int cx = W / 2, cy = H / 2;
  cv::Mat image = cv::Mat::zeros(H, W, CV_8UC3);
  int fx = 80, fy = 80;
  double pi = 3.1415926536;
  for(const auto &p : points)
    {
      Eigen::Vector3d project_x(1, 0, 0);
      double rot = pi/10;
      rot = 0;  
      Eigen::Vector3d project_y(0, cos(rot), sin(rot));
      Eigen::Vector3d project_z(0, sin(rot), -cos(rot)); 
      Eigen::Vector3d camera(0, 0, 0);
      Eigen::Vector3d d = p - camera;
      if (d.dot(project_z) > 0)
      {
        double x = d.dot(project_x) / d.dot(project_z) * fx;
        double y = d.dot(project_y) / d.dot(project_z) * fy;
        int px = static_cast<int>(round(cy - y));
        int py = static_cast<int>(round(cx + x));
        if (0 <= px && px < W && 0 <= py && py < H)
        {
          if (p.z() < threshold) {
            image.at<cv::Vec3b>(px, py) = cv::Vec3b(0, 255, 0);
          }
          else ;
            // image.at<cv::Vec3b>(px, py) = cv::Vec3b(0, 0, 255);
        }
      }
    }
  return image;
}

int main()
{
    std::vector<Eigen::Vector3d> points;
    const char *path = "/home/miu/PublicCourse/pony_data/VelodyneDevice32c";
    const int n = 20;
    
    Stat px, py;
    for(int i = 0; i < n; i++)
    {
        std::stringstream ss;
        ss << path << "/" << i << ".txt";
        PointCloud pc = ReadPointCloudFromTextFile(ss.str().c_str());
        for(auto &p : pc.asWorldCoordinates())
        {
            points.push_back(p);
            px.update(p.x());
            py.update(p.y());
        }
    }
    const int rows = 200, cols = 200;
    const double cx = rows / 2.0, cy = cols / 2.0;
    const int fx = 2, fy = 2;
    cv::Mat height;
    cv::add(cv::Mat::zeros(rows, cols, CV_32F), cv::Scalar(200), height);
    double rx = (px.min() + px.max()) / 2, ry = (py.min() + py.max()) / 2;
    for (auto &p : points) {
        const int y = coord(fy * (p.y() - ry) + cy), x = coord(fx * (p.x() - rx) + cx);
        if (0 <= y && y < rows && 0 <= x && x < cols)
        {
            height.at<float>(y, x) = std::min(height.at<float>(y, x), static_cast<float>(p.z()));
        }
    }
    cv::Mat norm_img = normalize(height);
    cv::namedWindow("height");
    
    int bins[128];
    std::fill(bins, bins + 128, 0);
    for(int y = 0; y < norm_img.rows; y++)
        for(int x = 0; x < norm_img.cols; x++)
        {
            bins[norm_img.at<uchar>(y, x) / 4]++;
        }
    for(int i = 0; i < 32; i++)
        std::cout << bins[i] << ' ';
    std::cout << std::endl;

    for(auto &p : points) {
        p = p - Eigen::Vector3d(rx, ry, 0);
    }
    
    double th = (8.0 - 255.0) / 255.0 * 28.0;
    std::cout << th << std::endl;

    // cv::resize(norm_img, norm_img, cv::Size(), 4, 4);
    // cv::applyColorMap(norm_img, norm_img, cv::COLORMAP_OCEAN);
    cv::Mat img = build_image(points, th);
    imshow("height", img);
    imwrite("/home/miu/PublicCourse/homework3/pointcloud/ground.jpg", img);
    cv::waitKey(0);
    // for(int i = 0; i < n; i++) {

    // }
    return 0;
}