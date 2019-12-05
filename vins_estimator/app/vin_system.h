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
#include <opencv2/opencv.hpp>



#include "vio_interface.h"
#include "types.h"

#include "vins_estimator/utility/visualization.h"



#include <std_msgs/Bool.h>



class Estimator;

namespace feature_track {
    class FeatureTracker;
}



class VinSystem : public VioInterface{
public:
    VinSystem(const std::string config_file);
    virtual ~VinSystem();


    bool addImage(const ros::Time & stamp, size_t cameraIndex,
                          const cv::Mat & image,
                          const std::vector<cv::KeyPoint> * keypoints = 0,
                          bool* asKeyframe = 0);

    bool addImuMeasurement(const ros::Time & stamp,
                                   const Eigen::Vector3d & alpha,
                                   const Eigen::Vector3d & omega) ;



    void shutdown();
//    ros::Publisher pub_match;
//    ros::Publisher pub_restart;

private:
    void feature_callback(const PointCloudMeasurement &feature_msg);

    void predict(const ImuMeasurement &imu_msg);
    void update();
    std::vector<std::pair<std::vector<ImuMeasurement>, PointCloudMeasurement>>
    getMeasurements();

    void restart_callback(const std_msgs::BoolConstPtr &restart_msg);
    void process();
    void processImageLoop();
    void processRawImage(const CameraMeasurement &img_msg);

    std::shared_ptr<Estimator> estimator_;
    std::condition_variable con;
    double current_time = -1;
    queue<ImuMeasurement> imu_buf;
    queue<PointCloudMeasurement> feature_buf;
    std::mutex m_image_mutex;

    std::deque<CameraMeasurement> m_image_buffer;

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


    std::vector<std::shared_ptr<feature_track::FeatureTracker>> trackerData_;
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;


    std::thread measurement_process_thread_;
    std::thread image_process_thread_;
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
    void RequestFinish();
    bool isFinishRequested();
    void SetFinish();
    bool isFinished();

    bool mbFeatureTrackFinishRequested;
    bool mbFeatureTrackFinished;
    std::mutex mFeatureTrackMutexFinish;
    void RequestFeatureTrackFinish();
    bool isFeatureTrackFinishRequested();
    void SetFeatureTrackFinish();
    bool isFeatureTrackFinished();

};


#endif //VINS_ESTIMATOR_VIN_SYSTEM_H
