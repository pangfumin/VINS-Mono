#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <rosbag/bag.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <Eigen/Geometry>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include  <opencv2/highgui/highgui.hpp>




int main (int argc, char ** argv) {

    std::string image_file = "/home/pang/data/dataset/ninebot_scooter/RawDataRec/2019-11-27_14-18-54/fisheye/1574835534352150.jpg";
    cv::Mat image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);

    int input_height = image.rows;
    int input_width = image.cols;

    std::cout << "image size: " << input_height << " " << input_width << std::endl;

    cv::Mat front_mask(input_height, input_width, CV_8UC3, cv::Scalar(255,255,255));

    int n = 4;
    cv::Point border_points[1][4];

    std::vector<cv::Point> point_vec;
    border_points[0][0] = (cv::Point(input_width/10, 0));
    border_points[0][1] = (cv::Point(input_width/7*3, 0));
    border_points[0][2] = (cv::Point(input_width/7*3, input_height));
    border_points[0][3] = (cv::Point(input_width/10, input_height));

    const cv::Point* ppt[1] = {border_points[0]};
    int npt[] = {(int)n};
    cv::polylines(front_mask, ppt, npt, 1, true, cv::Scalar(0,0,0), 1, cv::LINE_8, 0);
    cv::fillPoly(front_mask, ppt, npt, 1, cv::Scalar(0,0,0), cv::LINE_8);


    image &= front_mask;
    cv::imshow("image", image);
    cv::imshow("front_mask", front_mask);

    cv::waitKey();

    cv::imwrite("/home/pang/mask.jpg", front_mask);

    return 0;

}