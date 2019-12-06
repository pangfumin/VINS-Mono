//
// Created by pang on 2019/12/2.
//

#include <stdio.h>
#include <queue>
#include <map>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/chunked_file.h>
#include <sensor_msgs/Imu.h>
#include "scooter_vins/vin_system.h"
#include "ros_visualization.h"

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

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
//    std::string rosbag_file = "/home/pang/software/MYNT-EYE-S-SDK/2019-11-19-10-29-33.bag";
//    std::string config_file = "/home/pang/hl_ws/src/VINS-Mono/config/mynteye/mynteye_s_config.yaml";

//    std::string rosbag_file = "/home/pang/data/dataset/ninebot_scooter/2019-11-29_11-36-46/fisheye_imu1.bag";
//    std::string config_file = "/home/pang/hl_ws/src/VINS-Mono/config/segway/segway_scooter.yaml";

    std::string rosbag_file = "/persist/data/dataset/ninebot/B2_lidar_2018-09-21_14-46-46__B2.bag";
    std::string config_file = "/persist/maplab_ws/src/scooter_vins/config/segway/segway.yaml";

//    std::string rosbag_file = "/home/pang/disk/dataset/euroc/MH_01_easy.bag";
//    std::string config_file = "/home/pang/maplab_ws/src/VINS-Mono/config/euroc/euroc_config.yaml";

    readTopics(config_file);
    RosVisualization rosVisualization(n);

    std::shared_ptr<VinSystem> vinSystem = std::make_shared<VinSystem>(config_file);
    vinSystem->setFullStateCallbackWithExtrinsics(
            std::bind(&RosVisualization::publishFullStateExtrinsicAsCallback, &rosVisualization,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4));
    vinSystem->setFeatureTrackImageCallback(
            std::bind(&RosVisualization::publishFeatureTrackImageAsCallback, &rosVisualization,
                      std::placeholders::_1, std::placeholders::_2));



#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

//    registerPub(n);


    rosbag::Bag bag;
    std::cout << colouredString("\tOpening bag...", RED, REGULAR);
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

    std::cout << colouredString("\tQuering topics bag...", RED, REGULAR);

    std::vector < std::string > topic_list;

    rosbag::View view(bag);

    std::vector<const rosbag::ConnectionInfo *> bag_info = view.getConnections();
    std::set < std::string > bag_topics;

    for (const rosbag::ConnectionInfo *info : bag_info) {
        std::string topic_name;
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

        std::string topic = bagIt.getTopic();

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

                vinSystem->addImuMeasurement(imu->header.stamp, acc, gyro);
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
                vinSystem->addImage(image->header.stamp, 0, image_cv);
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


