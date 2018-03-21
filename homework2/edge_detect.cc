#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <algorithm>

int main(int argc, char *argv[])
{
    cv::Mat img, gray, edge;
    if (argc != 2) {
        printf("Please enter image path");
        return -1;
    }
    cv::namedWindow("edge");
    img = cv::imread(argv[1]);
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
    // imshow("edge", gray);
    // cv::waitKey(0);
    cv::Canny(gray, edge, 10, 30, 3);
    // imshow("edge", img);
    // cv::waitKey(0);
    imshow("edge", edge);
    imwrite("/home/miu/PublicCourse/homework2/edge.jpg", edge);
    cv::waitKey(0);


    // cv::Mat blend;
    // double alpha = 0.8;
    // cv::cvtColor(edge, edge, cv::COLOR_GRAY2BGR);
    // cv::addWeighted(img, alpha, edge, 1 - alpha, 0.0, blend);
    // imshow("edge", blend);
    // cv::waitKey(0);
    return 0;
}