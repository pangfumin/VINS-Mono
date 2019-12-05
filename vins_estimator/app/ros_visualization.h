//
// Created by pang on 2019/12/5.
//

#ifndef VINS_ESTIMATOR_ROS_VISUALIZATION_H
#define VINS_ESTIMATOR_ROS_VISUALIZATION_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "vins_estimator/utility/CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
//#include "../estimator.h"
//#include "../parameters.h"
#include <fstream>
#include <opencv2/core/core.hpp>

class RosVisualization {
public:
    RosVisualization(ros::NodeHandle &n);

    void publishFullStateExtrinsicAsCallback(
        const std::vector<ros::Time> & ts_vec,
        const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> & T_WS_vec,
        const std::vector<Eigen::Matrix<double, 9, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 9, 1>>> & sb_vec,
        const std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d> >& extrinsic_vec);

    void publishFeatureTrackImageAsCallback(
            const ros::Time ts,
            const cv::Mat image
            );

private:
    void pubCameraPose(
            const Eigen::Isometry3d &T_WS,
            const Eigen::Isometry3d T_IC,
            const std_msgs::Header &header);
    void pubOdometry(const Eigen::Isometry3d T_WS,
            const Eigen::Vector3d v,
            const std_msgs::Header &header);

    void pubTF(const Eigen::Isometry3d &T_WS,
                                 const Eigen::Isometry3d T_IC,
                                 const std_msgs::Header &header);

    ros::Publisher pub_odometry_;
    ros::Publisher pub_path_, pub_pose_;
    ros::Publisher pub_cloud_, pub_map_;
    ros::Publisher pub_key_poses_;
    ros::Publisher pub_ref_pose_, pub_cur_pose_;
    ros::Publisher pub_key_;
    nav_msgs::Path path_;
    ros::Publisher pub_pose_graph_;
    int IMAGE_ROW_, IMAGE_COL_;
    ros::Publisher pub_camera_pose_;
    ros::Publisher pub_camera_pose_visual_;
    ros::Publisher pub_extrinsic_;
    ros::Publisher pub_match_;

    nav_msgs::Path relo_path_;

    CameraPoseVisualization cameraposevisual_;
    CameraPoseVisualization keyframebasevisual_;
};


#endif //VINS_ESTIMATOR_ROS_VISUALIZATION_H
