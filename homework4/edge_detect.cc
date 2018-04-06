#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <queue>
#include <algorithm>

typedef Eigen::Vector2d Point2D;
typedef std::tuple<Point2D,Point2D> Line2D;
bool point_in_line(Point2D a, Point2D b, Point2D p, double th_1 = 40, double th_2 = 4.0)
{
    Point2D d = b - a;
    double l = d.norm();
    Point2D nd = d.normalized();
    double x = (p - a).dot(nd);
    double y = (p - a).dot(Point2D(nd.y(), -nd.x()));
    return -th_1 <= x && x <= l + th_1 && std::abs(y) <= th_2;
}

using std::max;
using std::min;

class LineGroup
{
    public:
    double range() 
    {
        if (_range == -1) {
            _range = compute_range();
        }
        return _range;
    }

    std::vector<Line2D> lines;

    private:
    double compute_range() const
    {
        double r = 0;
        for(const auto &line : lines) {
            Point2D d = (std::get<1>(line) - std::get<0>(line)).normalized();
            double mi = 1e40, ma = -1e40, x;
            for(const auto &l2 : lines) {
                x = (std::get<1>(l2) - std::get<0>(line)).dot(d);
                ma = max(ma, x);
                mi = min(mi, x);

                x = (std::get<0>(l2) - std::get<0>(line)).dot(d);
                ma = max(ma, x);
                mi = min(mi, x);
            }
            r = max(r, ma - mi);
        }
        return r;
    }
    double _range = -1;
};


template<typename Depth>
class Bilinear
{
    public:
    Bilinear(const cv::Mat &mat) {
       this-> _mat = &mat;
    }

    typedef Depth mat_depth;
    typedef double return_type;

    return_type operator()(double x, double y) const {
        double sum = 0, sum_weight = 0;
        int xg = int(x), yg = int(y);
        for(int dx = 0; dx < 1; dx++)
            for(int dy = 0; dy < 1; dy++)
            {
                int ys = yg + dy, xs = xg + dx;
                double weight = (1 - std::abs(x - xs)) * (1 - std::abs(y - ys));
                if (0 <= ys && ys < this->_mat->rows && 0 <= xs && xs < this->_mat->cols) {
                    sum += weight * this->_mat->template at <mat_depth>(ys, xs);
                    sum_weight += weight;
                }
            }
        return sum / (sum_weight + 1e-12);
    }

    private:
    const cv::Mat *_mat;
};

class BilinearBGR
{
    public:
    struct bgr
    {
        uchar b, g, r;
    };

    BilinearBGR(const cv::Mat &mat) {
       this-> _mat = &mat;
    }

    typedef Eigen::Array3d return_type;

    return_type operator()(double x, double y) const {
        return_type sum(0, 0, 0);
        double sum_weight = 0;
        int xg = int(x), yg = int(y);
        for(int dx = 0; dx < 1; dx++)
            for(int dy = 0; dy < 1; dy++)
            {
                int ys = yg + dy, xs = xg + dx;
                double weight = (1 - std::abs(x - xs)) * (1 - std::abs(y - ys));
                if (0 <= ys && ys < this->_mat->rows && 0 <= xs && xs < this->_mat->cols) {
                    auto color = this->_mat->template at <bgr>(ys, xs);
                    sum += weight * return_type(color.b, color.g, color.r);
                    sum_weight += weight;
                }
            }
        return sum / (sum_weight + 1e-12);
    }

    private:
    const cv::Mat *_mat;
};

double gradient(Bilinear <uchar>&simg, double rho, double theta, int ps, int pe)
{
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho, width = 1;
    double sum = 0;
    for(int p = ps; p < pe; p++)
    {
        double x = x0 + b * p, y = y0 - a * p;
        sum += simg(x + a * width, y + b * width) - simg(x - a * width, y - b * width);
    }
    return sum / (pe - ps + 1);
}

double check_color(Bilinear <uchar>&simg, double rho, double theta, int ps, int pe)
{
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    double sum = 0;
    for(int p = ps; p < pe; p++)
    {
        double x = x0 + b * p, y = y0 - a * p;
        sum += simg(x, y);
    }
    return sum / (pe - ps + 1);
}

double sign(double x)
{
    return x > 1e-8 ? 1 : (x < -1e-8 ? -1 : 0);
}

double cross(const Point2D &a, const Point2D &b)
{
    return a.x() * b.y() - a.y() * b.x(); 
}

double intersection(Point2D pa, Point2D da, Point2D pb, Point2D db)
{
    return cross(db, pb - pa) / cross(db, da);
}

cv::Point opencv_point(Point2D p)
{
    return cv::Point(cvRound(p.x()), cvRound(p.y()));
}

struct Edge
{
    double rho, theta, sp, ep, grad;   
    
    template<typename Tp>
    void draw(cv::Mat &img, Tp color) 
    {
        double a = cos(theta), b = sin(theta);
        double x0 = rho * a, y0 = rho * b;
        cv::line(img, 
            cv::Point(cvRound(x0 + sp * b), cvRound(y0 - sp * a)), 
            cv::Point(cvRound(x0 + ep * b), cvRound(y0 - ep * a)), 
            color, 2, CV_AA);    
    }

    void draw(cv::Mat &img)
    {
        this->draw(img, cv::Scalar(180,180,0));
    }

    template<typename Scalar>
    void draw_polygan(cv::Mat &img, const Edge &medge, Scalar color)
    {
        Point2D ms, me;
        Point2D ps, pe;
        std::tie(ms, me) = medge.end_points();
        std::tie(ps, pe) = end_points();
        cv::Point points[1][4];
        points[0][0] = opencv_point(ms);
        points[0][1] = opencv_point(me);
        points[0][2] = opencv_point(pe);
        points[0][3] = opencv_point(ps);
        // for(int i = 0; i < 4; i++)
        //     std::cout << points[0][i].x << ' ' << points[0][i].y << std::endl;
        const cv::Point * polygan[1] = {points[0]};
        int npts[] = {4};
        cv::fillPoly(img, polygan, npts, 1, color, cv::LINE_8);
    }

    typedef cv::Point Point;
    void test(cv::Mat &img)
    {
        // int lineType = cv::LINE_8;
        // double w = 1000;
        // Point rook_points[1][20];
        // rook_points[0][0]  = Point(    w/4,   7*w/8 );
        // rook_points[0][1]  = Point(  3*w/4,   7*w/8 );
        // rook_points[0][2]  = Point(  3*w/4,  13*w/16 );
        // rook_points[0][3]  = Point( 11*w/16, 13*w/16 );
        // rook_points[0][4]  = Point( 19*w/32,  3*w/8 );
        // rook_points[0][5]  = Point(  3*w/4,   3*w/8 );
        // rook_points[0][6]  = Point(  3*w/4,     w/8 );
        // rook_points[0][7]  = Point( 26*w/40,    w/8 );
        // rook_points[0][8]  = Point( 26*w/40,    w/4 );
        // rook_points[0][9]  = Point( 22*w/40,    w/4 );
        // rook_points[0][10] = Point( 22*w/40,    w/8 );
        // rook_points[0][11] = Point( 18*w/40,    w/8 );
        // rook_points[0][12] = Point( 18*w/40,    w/4 );
        // rook_points[0][13] = Point( 14*w/40,    w/4 );
        // rook_points[0][14] = Point( 14*w/40,    w/8 );
        // rook_points[0][15] = Point(    w/4,     w/8 );
        // rook_points[0][16] = Point(    w/4,   3*w/8 );
        // rook_points[0][17] = Point( 13*w/32,  3*w/8 );
        // rook_points[0][18] = Point(  5*w/16, 13*w/16 );
        // rook_points[0][19] = Point(    w/4,  13*w/16 );

        // const Point* ppt[1] = { rook_points[0] };
        // int npt[] = { 20 };

        // fillPoly( img,
        //         ppt,
        //         npt,
        //         1,
        //         cv::Scalar( 255, 255, 255 ),
        //         lineType );
    }

    std::tuple<Point2D, Point2D> end_points() const
    {
        double a = cos(theta), b = sin(theta);
        double x0 = rho * a, y0 = rho * b;
        return std::make_tuple(
            Point2D(x0 + sp * b, y0 - sp * a), 
            Point2D(x0 + ep * b, y0 - ep * a)
        );
    }
    double set_gradient(Bilinear <uchar>&simg)
    {
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho, width = 1;
        double sum = 0;
        int samples = 0;
        for(double p = sp; p <= ep; p += 1)
        {
            double x = x0 + b * p, y = y0 - a * p;
            sum += simg(x + a * width, y + b * width) - simg(x - a * width, y - b * width);
            samples++;
        }
        grad = sum / samples;
        return grad;
    }
    template<class Sampler>
    typename Sampler::return_type sample_color(Sampler &simg, double offset = 0.0)
    {
        double a = cos(theta), b = sin(theta);
        double x0 = a * (rho + offset), y0 = b * (rho + offset);
        typename Sampler::return_type sum;
        sum = sum * 0;
        int samples = 0;
        for(double p = sp; p <= ep; p += 1)
        {
            double x = x0 + b * p, y = y0 - a * p;
            sum += simg(x, y);
            samples++;
        }
        return sum / samples;
    }

    template<class Sampler>
    typename Sampler::return_type sample_color_diff(Sampler &simg, double offset = 0.0)
    {
        double a = cos(theta), b = sin(theta);
        double x0 = a * (rho + offset), y0 = b * (rho + offset);
        typename Sampler::return_type sum;
        typename Sampler::return_type psample, sample;
        sum = sum * 0;
        int samples = 0;
        for(double p = sp; p <= ep; p += 1)
        {
            double x = x0 + b * p, y = y0 - a * p;
            sample = simg(x, y);
            if (samples > 0) {
                sum += (psample - sample).abs();
            }
            psample = sample; 
            samples++;
        }
        return sum / samples;
    }
};

class Line
{
    public:
    Line(double rho, double theta)
    {
        _rho = rho;
        _theta = theta;
        _cos = cos(theta);
        _sin = sin(theta);
    }
    
    double point_distance(const Point2D & p) const
    {
        return p.dot(Point2D(_cos, _sin)) - _rho;
    }

    Edge reflect(Edge &edge) const
    {
        double center_theta = (edge.theta + _theta) / 2;
        Point2D a, b;
        std::tie(a,b) = edge.end_points();
        Point2D dir = Point2D(cos(center_theta), sin(center_theta));

        double sp = intersection(base(), direction(), a, dir);
        double ep = intersection(base(), direction(), b, dir);

        if (sp > ep) {
            std::swap(sp, ep);
        }
        return {_rho, _theta, sp, ep, 0};
    }

    Point2D direction() const
    {
        return Point2D(_sin, -_cos);
    }

    Point2D base() const
    {
        return Point2D(_rho * _cos, _rho * _sin);
    }

    private:
    double _rho, _theta, _cos, _sin;
};



std::tuple<cv::Mat, cv::Mat> parse_image(cv::Mat img)
{
    cv::Mat gray, edge, color_edge, lap, abs_lap;    
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
    // imshow("edge", img);
    // cv::waitKey(0);
    cv::Canny(gray, edge, 10, 30, 3);
    // imshow("edge", img);
    // cv::waitKey(0);
    cv::cvtColor(edge, color_edge, cv::COLOR_GRAY2BGR);
    

    // imwrite("/home/miu/PublicCourse/homework2/edge.jpg", edge);

   std::vector<cv::Vec2f> lines;
   cv::Mat blur_edge;
   cv::GaussianBlur(edge, blur_edge, cv::Size(3, 3), 0, 0);
   cv::cvtColor(blur_edge, color_edge, cv::COLOR_GRAY2BGR);
   HoughLines(edge, lines, 1, CV_PI/180, 150, 0, 0 );   
   //std::cout << lines.size() << std::endl;
   Bilinear<uchar> sampler(blur_edge);
   cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
   cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0);
   Bilinear<uchar> simg(gray);
   img.copyTo(color_edge);

   std::vector<Edge> strong_edges;
   std::vector<Line> strong_lines;
   for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        bool found = false;
        int ec = 0, bc = 0, tbc = 0, first_edge, last_edge;
        for(int dp = -1000; dp <= 1000; dp++)
        {
            double x = x0 + b * dp, y = y0 - a * dp;
            if (0 <= x && x < img.cols && 0 <= y && y < img.rows) {
                if (sampler(x, y) > 32) {
                    if (ec == 0) {
                        first_edge = dp;
                        tbc = 0;
                    }
                    ec++;
                    bc = 0;
                    last_edge = dp;
                    // cv::circle(color_edge, cv::Point(x, y),  3, cv::Scalar(255,180,0), 1, CV_AA, 0);                
                }
                else {
                    bc++;
                    tbc++;
                }
                if (bc > 10 || tbc > 0.4 * ec) {

                    if (ec > 40) {
                        double x = gradient(simg, rho, theta, first_edge, last_edge);
                        if (std::abs(x) > 40) {
                            strong_edges.push_back({rho, theta, (double)first_edge, (double)last_edge, x});
                            if (!found) {
                                strong_lines.push_back(Line(rho, theta));
                            }
                            // double mw = 0, grad_max = 0, s = sign(x);
                            // for(double width = 2; width <= 20; width += 2) {
                            //     double grad = gradient(simg, rho + width * s, theta, first_edge, last_edge);
                            //     if (std::abs(grad) > std::abs(grad_max)) {
                            //         mw = width * s;
                            //         grad_max = grad;
                            //     }
                            // }
                            // if (-s * grad_max > 20) {
                            //     // std::cout << x << ' ' << grad_max << std::endl;
                            //     if (check_color(simg, rho + mw * 0.5, theta, first_edge, last_edge) > 255 * 0.6) {
                            //         double color_0 = check_color(simg, rho + mw + 10 * s, theta, first_edge, last_edge);
                            //         double color_1 = check_color(simg, rho + mw * 0.5, theta, first_edge, last_edge);
                            //         double color_2 = check_color(simg, rho - 10 * s, theta, first_edge, last_edge);
                            //         if (color_1 - color_0 > 60 && color_1 - color_2 > 60) {
                            //             cv::line(color_edge, 
                            //                 cv::Point(cvRound(x0 + first_edge * b), cvRound(y0 - first_edge * a)), 
                            //                 cv::Point(cvRound(x0 + last_edge * b), cvRound(y0 - last_edge * a)), 
                            //                 cv::Scalar(180,180,0), 2, CV_AA);    
                            //             cv::line(color_edge, 
                            //             cv::Point(cvRound(x0 + first_edge * b + mw * a), cvRound(y0 - first_edge * a + + mw * b)), 
                            //             cv::Point(cvRound(x0 + last_edge * b + mw * a), cvRound(y0 - last_edge * a + mw * b)), 
                            //             cv::Scalar(0,140,180), 2, CV_AA); 
                            //         }
                            //     }
                            // }
                            
                        }
                    }
                    ec = 0;
                }
            }             
        }
        
        // pt1.x = cvRound(x0 + 1000*(-b));
        // pt1.y = cvRound(y0 + 1000*(a));
        // pt2.x = cvRound(x0 - 1000*(-b));
        // pt2.y = cvRound(y0 - 1000*(a));
        // cv::line( color_edge, pt1, pt2, cv::Scalar(0,0,255), 2, CV_AA);
    }

    BilinearBGR sorig(img);
    cv::Mat lane = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    for(Edge &edge : strong_edges) 
    {
        // edge.draw(color_edge);
        double max_grad = 0, s = sign(edge.grad), mwidth;
        Edge medge;
        Point2D ps, pe; 
        std::tie(ps, pe) = edge.end_points();
        for(Line &line : strong_lines) {
            double width = std::min(-s * line.point_distance(ps), -s * line.point_distance(pe));
            if (0 < width && width < 20)
            {
                Edge rfedge = line.reflect(edge);
                rfedge.set_gradient(simg);
                if (-s * rfedge.grad > max_grad)
                {
                    max_grad = -s * rfedge.grad;
                    medge = rfedge;
                    mwidth = width;
                }
            }
        }

        if (max_grad > 30)
        {
            // edge.draw(color_edge);
            if ( (edge.sample_color(simg, s * mwidth / 2) - edge.sample_color(simg, -s * mwidth / 2)) > 40 && 
                 (medge.sample_color(simg, -s * mwidth / 2) - medge.sample_color(simg, s * mwidth / 2)) > 40)
            {
                auto ic = edge.sample_color_diff(sorig, s * mwidth / 2);
                auto oc = edge.sample_color_diff(sorig, -s * mwidth / 2);
                double od1 = oc.x() + oc.y() + oc.z();
                oc = medge.sample_color_diff(sorig, s * mwidth / 2);
                double od2 = oc.x() + oc.y() + oc.z();
                if ((ic.x() + ic.y() + ic.z()) < 20 &&  od1 < 30 && od2 < 30)
                {
                    auto oc1 = edge.sample_color(sorig, -s * mwidth / 2);
                    auto oc2 = medge.sample_color(sorig, s * mwidth / 2);
                    oc1 = (oc1 - oc2).abs();
                    if ( Eigen::Vector3d(oc1).norm() < 20) {
                        // edge.test(color_edge);
                        edge.draw_polygan(color_edge, medge, cv::Scalar(0, 0, 255));
                        edge.draw_polygan(lane, medge, 255);
                        
                        // medge.draw(color_edge);
                        // std::cout << od1 << ' ' << od2 << std::endl;
                    }
                    
                    // medge.draw(color_edge, cv::Scalar(0, 140, 180));
                }
                // std::cout << int(ic.x()) << ',' << int(ic.y()) << ',' << int(ic.z()) << "   ";
                // std::cout << int(oc.x()) << ',' << int(oc.y()) << ',' << int(oc.z()) << std::endl;
                
            }
        }
    }
    // imshow("edge", color_edge);
    // cv::waitKey(0);   
    // imshow("edge", img);
    // cv::waitKey(0);
    return std::make_tuple(color_edge, lane);
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Please enter image path");
        return -1;
    }
    
    cv::Mat img;
    cv::namedWindow("edge");
    char img_path[200], target_path[200];
    cv::Mat detect, lane;
    for (int i =0; ; i++) {
        sprintf(img_path, "%s/%d.jpg", argv[1], i);
        sprintf(target_path, "%s/detect_%d.jpg", argv[1], i);
        FILE *f = fopen(img_path, "r");
        if (f != NULL) {
            fclose(f);
            std::cout << img_path << std::endl;
            img = cv::imread(img_path);
            std::tie(detect, lane) = parse_image(img);
            imwrite(target_path, detect);
            sprintf(target_path, "%s/lane_%d.jpg", argv[1], i);
            imwrite(target_path, lane);
        }
        else break;
        // break;
    }
    return 0;
}