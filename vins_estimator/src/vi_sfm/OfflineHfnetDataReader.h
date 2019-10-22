//
// Created by pang on 2019/10/22.
//

#ifndef VINS_ESTIMATOR_OFFLINEHFNETDATAREADER_H
#define VINS_ESTIMATOR_OFFLINEHFNETDATAREADER_H

#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <ros/time.h>

typedef Eigen::Matrix<double, 4096, 1> GlobalDescriptor;
typedef Eigen::Matrix<double, 256, 1> LocalDescriptor;
typedef Eigen::Vector2f Keypoint;

class OfflineHfnetDataReader {
public:

    OfflineHfnetDataReader(const std::string& dataset);

    int size() {return kp_file_names_.size();}

    std::pair<ros::Time, GlobalDescriptor>  getGlobalDescriptor(int cnt);
    std::pair<ros::Time,std::vector< LocalDescriptor>>  getLocalDescriptor(int cnt);
    std::pair<ros::Time,std::vector< Keypoint>> getKeypoints(int cnt);
private:
    std::string dataset_path_;
    int cur_cnt_;

    std::string kp_files_folder_;
    std::string local_des_files_folder_;
    std::string global_des_files_folder_;

    std::vector<std::string> kp_file_names_;
    std::vector<std::string> local_file_names_;
    std::vector<std::string> global_file_names_;



};


#endif //VINS_ESTIMATOR_OFFLINEHFNETDATAREADER_H
