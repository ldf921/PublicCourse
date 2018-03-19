// Copyright @2018 Tingfung Lau All rights reserved.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>

using namespace cv;

uchar color_rgb2gray(uchar r, uchar g, uchar b)
{
  return uchar(std::min(0.299 * r + 0.587 * g + 0.114 * b, 255.0));
}

int main(int argc, char *argv[]) {
  cv::Mat image;
  // ATTENTION!!! : please use absolute path for reading the data file.
  image = imread("/home/miu/PublicCourse/homework2/sample_data/GigECameraDeviceWideAngle/0.jpg", CV_LOAD_IMAGE_COLOR);
  cv::Mat target_image(image.rows, image.cols, CV_8UC1);

  uchar *p, *q;
  for(int i = 0; i < image.rows; i++)
  {
    p = image.ptr<uchar>(i);
    q = target_image.ptr<uchar>(i);
    for(int j = 0, k = 0; j < image.cols * 3; j += 3, k++)
    {
      uchar b, g, r;
      b = p[j];
      g = p[j + 1];
      r = p[j + 2];
      q[k] = color_rgb2gray(r, g, b);
    }
  }

  namedWindow("grayimage");
  imshow("grayimage", target_image);
  waitKey(0);
  return 0;
}
