//
// Created by pang on 2019/10/21.
//

#ifndef VINS_ESTIMATOR_VISFM_H
#define VINS_ESTIMATOR_VISFM_H

#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/projection_factor.h"
#include "../factor/projection_td_factor.h"
#include "../factor/marginalization_factor.h"
#include "../feature_manager.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

class ViSfm {
public:
    typedef Eigen::Matrix<double,7,1> Vec7;
    typedef Eigen::Matrix<double,9,1> Vec9;
    ViSfm();
    void addState(const ViState& viState) {
        states_.push_back(viState);
        Vec7 vec_pose;
        Vec9 vec_sb;
        vec_pose << viState.t_, viState.q_.coeffs();
        vec_sb << viState.v_, viState.ba_, viState.bg_;
        para_Poses_.push_back(vec_pose);
        para_SpeedBiases_.push_back(vec_sb);
    }
    void addImuFactor(const IntegrationBase& imufactor) {
        imuFactors_.push_back(imufactor);
    }
    void addFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image) {
        f_manager_.addFeatureCheckParallax(frame_count_, image, 0);
        frame_count_ ++;
    }

    void triangulation();
    void ba();


    int frame_count_;
    FeatureManager f_manager_;
    std::vector<ViState> states_;
    std::vector<Vec7> para_Poses_;
    std::vector<Vec9> para_SpeedBiases_;
    std::vector<double> para_Features_;
    std::vector<IntegrationBase> imuFactors_;
    std::vector<FeatureOneFrame> frames_;

};


#endif //VINS_ESTIMATOR_VISFM_H
