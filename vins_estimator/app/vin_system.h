//
// Created by pang on 2019/12/2.
//

#ifndef VINS_ESTIMATOR_VIN_SYSTEM_H
#define VINS_ESTIMATOR_VIN_SYSTEM_H

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/chunked_file.h>

#include "utility/visualization.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include "feature_track/feature_tracker.h"
#include "feature_track/parameters.h"





class VinSystem {
public:


    VinSystem();
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void img_callback(const sensor_msgs::ImageConstPtr &img_msg);

    ros::Publisher pub_match;
    ros::Publisher pub_restart;

private:
    void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg);

    void predict(const sensor_msgs::ImuConstPtr &imu_msg);
    void update();
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
    getMeasurements();

    void restart_callback(const std_msgs::BoolConstPtr &restart_msg);
    void process();

    Estimator estimator;
    std::thread measurement_process;
    std::condition_variable con;
    double current_time = -1;
    queue<sensor_msgs::ImuConstPtr> imu_buf;
    queue<sensor_msgs::PointCloudConstPtr> feature_buf;
    int sum_of_wait = 0;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex m_estimator;

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;

#define SHOW_UNDISTORTION 0




    feature_track::FeatureTracker trackerData[feature_track::NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;
};


#endif //VINS_ESTIMATOR_VIN_SYSTEM_H
