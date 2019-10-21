//
// Created by pang on 2019/10/21.
//

#ifndef VINS_ESTIMATOR_TYPES_H
#define VINS_ESTIMATOR_TYPES_H
#include <string>
#include <vector>
#include <Eigen/Core>
struct ImageMeas {
    uint64 ts_;
    std::string image_file_;
};
struct ImuMeas {
    uint64_t ts_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
};

struct SensorConfig {
    Eigen::Matrix4d T_BC;
    Eigen::Vector3d acc_noise_;
    Eigen::Vector3d gyro_noise_;
    Eigen::Vector3d acc_bias_noise_;
    Eigen::Vector3d gyro_noise_noise_;
};
#endif //VINS_ESTIMATOR_TYPES_H


