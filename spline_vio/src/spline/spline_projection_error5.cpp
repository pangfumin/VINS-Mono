#include "spline_vio/spline/spline_projection_error5.h"
#include "spline_vio/spline/QuaternionSplineUtility.hpp"
#include "spline_vio/spline/PoseLocalParameter.hpp"

SplineProjectError5::SplineProjectError5(const SplineProjectFunctor5& functor):
functor_(functor){}


bool SplineProjectError5::Evaluate(double const *const *parameters,
                                   double *residuals,
                                   double **jacobians) const {
    return EvaluateWithMinimalJacobians(parameters,
                                        residuals,
                                        jacobians, NULL);
}


bool SplineProjectError5::EvaluateWithMinimalJacobians(double const *const *parameters,
                                                       double *residuals,
                                                       double **jacobians,
                                                       double **jacobiansMinimal) const {

    if (!jacobians) {
        return ceres::internal::VariadicEvaluate<
                SplineProjectFunctor5, double, 7, 7, 7, 7, 7, 1,1,0,0,0>
        ::Call(functor_, parameters, residuals);
    }




    bool success =  ceres::internal::AutoDiff<SplineProjectFunctor5, double,
            7,7,7,7,7,1,1>::Differentiate(
            functor_,
            parameters,
            2,
            residuals,
            jacobians);

    if (success && jacobiansMinimal!= NULL) {
        Pose<double> T0(parameters[0]);
        Pose<double> T1(parameters[1]);
        Pose<double> T2(parameters[2]);
        Pose<double> T3(parameters[3]);
        Pose<double> T4(parameters[4]);
        if( jacobiansMinimal[0] != NULL){
            Eigen::Map<Eigen::Matrix<double,2,6,Eigen::RowMajor>> J0_minimal_map(jacobiansMinimal[0]);
            Eigen::Matrix<double,7,6,Eigen::RowMajor> J_plus;
            PoseLocalParameter::plusJacobian(T0.parameterPtr(),J_plus.data());
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> J0_map(jacobians[0]);
            J0_minimal_map = J0_map*J_plus;
        }

        if( jacobiansMinimal[1] != NULL){
            Eigen::Map<Eigen::Matrix<double,2,6,Eigen::RowMajor>> J1_minimal_map(jacobiansMinimal[1]);
            Eigen::Matrix<double,7,6,Eigen::RowMajor> J_plus;
            PoseLocalParameter::plusJacobian(T1.parameterPtr(),J_plus.data());
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> J1_map(jacobians[1]);
            J1_minimal_map = J1_map*J_plus;
        }

        if( jacobiansMinimal[2] != NULL){

            Eigen::Map<Eigen::Matrix<double,2,6,Eigen::RowMajor>> J2_minimal_map(jacobiansMinimal[2]);
            Eigen::Matrix<double,7,6,Eigen::RowMajor> J_plus;
            PoseLocalParameter::plusJacobian(T2.parameterPtr(),J_plus.data());
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> J2_map(jacobians[2]);
            J2_minimal_map = J2_map*J_plus;
        }

        if( jacobiansMinimal[3] != NULL){

            Eigen::Map<Eigen::Matrix<double,2,6,Eigen::RowMajor>> J3_minimal_map(jacobiansMinimal[3]);
            Eigen::Matrix<double,7,6,Eigen::RowMajor> J_plus;
            PoseLocalParameter::plusJacobian(T3.parameterPtr(),J_plus.data());
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> J3_map(jacobians[3]);
            J3_minimal_map = J3_map*J_plus;
        }

        if( jacobiansMinimal[4] != NULL){

            Eigen::Map<Eigen::Matrix<double,2,6,Eigen::RowMajor>> J4_minimal_map(jacobiansMinimal[4]);
            Eigen::Matrix<double,7,6,Eigen::RowMajor> J_plus;
            PoseLocalParameter::plusJacobian(T4.parameterPtr(),J_plus.data());
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> J4_map(jacobians[4]);
            J4_minimal_map = J4_map*J_plus;
        }

        if( jacobiansMinimal[5] != NULL){
            Eigen::Map<Eigen::Matrix<double,2,1>> J5_minimal_map(jacobiansMinimal[5]);
            Eigen::Map<Eigen::Matrix<double,2,1>> J5_map(jacobians[5]);
            J5_minimal_map = J5_map;
        }

        if( jacobiansMinimal[6] != NULL){
            Eigen::Map<Eigen::Matrix<double,2,1>> J_minimal_map(jacobiansMinimal[6]);
            Eigen::Map<Eigen::Matrix<double,2,1>> J_map(jacobians[6]);
            J_minimal_map = J_map;
        }


    }

    return true;
}