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

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

class ViSfm {
public:
    ViSfm();
    void addState(const ViState& viState) {
        states_.push_back(viState);
    }
    void addImuFactor(const IntegrationBase& imufactor) {
        imuFactors_.push_back(imufactor);
    }
    void addFrame(const FeatureOneFrame& frame) {
        frames_.push_back(frame);
    }

    std::vector<ViState> states_;
    std::vector<IntegrationBase> imuFactors_;
    std::vector<FeatureOneFrame> frames_;

};


#endif //VINS_ESTIMATOR_VISFM_H
