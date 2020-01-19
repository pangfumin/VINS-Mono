//
// Created by pang on 2019/12/2.
//

#include <stdio.h>
#include <queue>
#include <map>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "vin_system.h"

const string RESET = "\033[0m";
const string BLACK = "0m";
const string RED = "1m";
const string GREEN = "2m";
const string BOLD = "\033[1;3";
const string REGULAR = "\033[0;3";
const string UNDERLINE = "\033[4;3";
const string BACKGROUND = "\033[4";

std::string colouredString(std::string str, std::string colour, std::string option)
{
    return option + colour + str + RESET;
}

std::string IMU_TOPIC;
std::string IMAGE_TOPIC;
void readTopics(const std::string config_file)
{
//    std::string config_file;
//    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["image_topic"] >> IMAGE_TOPIC;



    fsSettings.release();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string rosbag_file = "/home/pang/disk/dataset/euroc/MH_01_easy.bag";
    std::string config_file = "/home/pang/maplab_ws/src/SplineVIO/config/euroc/euroc_config_no_extrinsic.yaml";

    readTopics(config_file);
    VinSystem vinSystem(config_file);



#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    vinSystem.pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);


    rosbag::Bag bag;
    cout << colouredString("\tOpening bag...", RED, REGULAR);
    bag.open(rosbag_file, rosbag::bagmode::Read);
    cout << colouredString("\t[DONE!]", GREEN, REGULAR) << endl;

    cout << colouredString("\tQuering topics bag...", RED, REGULAR);

    vector < string > topic_list;

    rosbag::View view(bag);

    vector<const rosbag::ConnectionInfo *> bag_info = view.getConnections();
    std::set < string > bag_topics;

    for (const rosbag::ConnectionInfo *info : bag_info) {
        string topic_name;
        topic_name = info->topic;

        std::cout << "topic: " << topic_name << std::endl;

        topic_list.push_back(topic_name);
    }

    view.addQuery(bag, rosbag::TopicQuery(topic_list));

    uint64_t lastImuTs = 0;
    uint64_t lastImageTs = 0;
    int image_cnt  = 0 ;
    for (auto bagIt : view ) {
        //if (!ros::ok()) break;

        string topic = bagIt.getTopic();

        if (topic == IMU_TOPIC) {
            sensor_msgs::Imu::ConstPtr imu =
                    bagIt.instantiate<sensor_msgs::Imu>();
            if (lastImuTs < imu->header.stamp.toNSec()) {

//                vinSystem.imu_callback(imu);
                Eigen::Vector3d acc, gyro;
                acc.x() = imu->linear_acceleration.x;
                acc.y() = imu->linear_acceleration.y;
                acc.z() = imu->linear_acceleration.z;

                gyro.x() = imu->angular_velocity.x;
                gyro.y() = imu->angular_velocity.y;
                gyro.z() = imu->angular_velocity.z;

                vinSystem.addImuMeasurement(imu->header.stamp, acc, gyro);
                lastImuTs = imu->header.stamp.toNSec();
            }
        }

        if (topic == IMAGE_TOPIC) {
            sensor_msgs::Image::ConstPtr image =
                    bagIt.instantiate<sensor_msgs::Image>();



            if (lastImageTs < image->header.stamp.toNSec()) {
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
                cv::Mat image_cv = cv_ptr->image;
                image_cv.convertTo(image_cv, CV_8UC1);
                vinSystem.addImage(image->header.stamp, 0, image_cv);
                lastImageTs = image->header.stamp.toNSec();
            }

//            if (image_cnt ++ > 500) break;



//
//            cv::imshow("image", image_cv);
            cv::waitKey(20);

        }


    }

    std::cout << "shutdown" << std::endl;

    return 0;
}


