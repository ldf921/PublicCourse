// Copyright @2018 Pony AI Inc. All rights reserved.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

using namespace cv;

// class BilinearBGR
// {
//     public:
//     struct bgr
//     {
//         uchar b, g, r;
//     };

//     BilinearBGR(const cv::Mat &mat) {
//        this-> _mat = &mat;
//     }

//     typedef Eigen::Array3d return_type;

//     return_type operator()(double x, double y) const {
//         return_type sum(0, 0, 0);
//         double sum_weight = 0;
//         int xg = int(x), yg = int(y);
//         for(int dx = 0; dx < 1; dx++)
//             for(int dy = 0; dy < 1; dy++)
//             {
//                 int ys = yg + dy, xs = xg + dx;
//                 double weight = (1 - std::abs(x - xs)) * (1 - std::abs(y - ys));
//                 if (0 <= ys && ys < this->_mat->rows && 0 <= xs && xs < this->_mat->cols) {
//                     auto color = this->_mat->template at <bgr>(ys, xs);
//                     sum += weight * return_type(color.b, color.g, color.r);
//                     sum_weight += weight;
//                 }
//             }
//         return sum / (sum_weight + 1e-12);
//     }

//     private:
//     const cv::Mat *_mat;
// };

// double solver(double r, double k1, double k2)
// {
//     double xp = r, xn, f, df;
//     while (true)
//     {
//         f = xp + k1 * xp * xp + k2 * xp * xp * xp
//         df = 1 + 2k1 * xp + 3 * k2 * xp * xp
//         xn = xp -  / 
//     }
// }

// cv::Mat distort(const cv::Mat &img, double k1, double k2)
// {
//     BilinearBGR samp(img);
//     cv::Mat dimg(img.rows, img.cols);
//     double cx = (img.rows - 1) / 2, cy = (img.cols - 1) / 2;
//     for(int y= 0; y < img.cols, y++)
//     {
//         for(int x = 0; x < img.rows; x++)
//         {
//             double r = (x - cx);
//         }
//     }
// }

int main() {
  cv::Mat image, distort_image;
  // ATTENTION!!! : please use absolute path for reading the data file.
  image = imread("/home/miu/PublicCourse/homework3/chessboard/chessboard_undistorted.png", CV_LOAD_IMAGE_COLOR);
  namedWindow("chessboard");
  imshow("chessboard", image);
  waitKey(0);
  double camera_intrinstic[3][3] = {{500.0, 0.0, (image.cols - 1) / 2.0}, {0.0, 500.0, (image.rows - 1) / 2.0}, {0.0, 0.0, 1.0}};
  cv::Mat camera(3, 3, CV_64F, camera_intrinstic);
  double distortion[1][4] = {{0.1, 0.1, 0, 0}};
  cv::Mat distort(1, 4, CV_64F, distortion);
  cv::undistort(image, distort_image, camera, distort);
  imshow("chessboard", distort_image);
  waitKey(0);
  imwrite("/home/miu/PublicCourse/homework3/chessboard/distorted.png", distort_image);
  return 0;
}
