#ifndef POSESPLINESAMPLEERROR_H
#define  POSESPLINESAMPLEERROR_H

//#include "vins_estimator/spline/QuaternionSpline.hpp"
#include <ceres/ceres.h>
#include <iostream>
#include "vins_estimator/spline/QuaternionLocalParameter.hpp"
#include "vins_estimator/spline/Pose.hpp"


class PoseSplineSampleError
    : public ceres::SizedCostFunction<6, 7, 7, 7, 7> {
public:

    typedef Eigen::Matrix<double, 6, 6> covariance_t;
    typedef covariance_t information_t;

    PoseSplineSampleError(double t_meas, Pose<double> T_meas);
    virtual ~PoseSplineSampleError();

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;
    bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                      double* residuals,
                                      double** jacobians,
                                      double** jacobiansMinimal) const;
private:

    double t_meas_;
    Pose<double> T_Meas_;
    mutable information_t information_; ///< The information matrix for this error term.
    mutable information_t squareRootInformation_; ///< The square root information matrix for this error term.
};

#endif