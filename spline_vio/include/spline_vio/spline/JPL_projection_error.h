
#ifndef INCLUDE_JPL_PROJECTION_ERROR_HPP_
#define INCLUDE_JPL_PROJECTION_ERROR_HPP_


#include "spline_vio/utility/utility.h"

#include <ceres/ceres.h>
#include "spline_vio/spline/Quaternion.hpp"
#include "spline_vio/spline/PoseLocalParameter.hpp"
#include "spline_vio/spline/QuaternionLocalParameter.hpp"
#include "spline_vio/parameters.h"

namespace  JPL {


class ProjectionBase{
    public:
    ProjectionBase() = delete;


    ProjectionBase(const double _t0, const Eigen::Vector3d& uv_C0,
                        const double _t1, const Eigen::Vector3d& uv_C1,
                        const Eigen::Isometry3d _T_IC) :
                t0_(_t0), C0uv_(uv_C0), t1_(_t1), C1uv_(uv_C1),
                T_IC_(_T_IC) {}


        template <typename  T>
        Eigen::Matrix<T, 2, 1>
        evaluate(const Eigen::Matrix<T,3,1> &Pi, const QuaternionTemplate<T> &Qi,
                 const Eigen::Matrix<T,3,1> &Pj, const QuaternionTemplate<T> &Qj,
                 const double& rho,
                 T **jacobians = NULL) {

            auto t_WI0_hat = Pi;
            auto t_WI1_hat = Pj;
            Eigen::Matrix<T,3,3> R_WI0 = quatToRotMat(Qi);
            Eigen::Matrix<T,3,3> R_WI1 = quatToRotMat(Qj);

            T inv_dep =  T(rho);
            Eigen::Matrix<T,3,3> R_IC = T_IC_.matrix().topLeftCorner(3,3).cast<T>();
            Eigen::Matrix<T,3,1> t_IC = T_IC_.matrix().topRightCorner(3,1).cast<T>();
//
            Eigen::Matrix<T,3,1> C0p = C0uv_.cast<T>() / inv_dep;
            Eigen::Matrix<T,3,1> I0p = R_IC * C0p + t_IC;
            Eigen::Matrix<T,3,1> Wp = R_WI0 * I0p + t_WI0_hat;
            Eigen::Matrix<T,3,1> I1p = R_WI1.inverse() * (Wp - t_WI1_hat);
            Eigen::Matrix<T,3,1> C1p = R_IC.inverse() * (I1p - t_IC);

            T inv_z = T(1.0)/C1p(2);
            Eigen::Matrix<T,2,1> hat_C1uv(C1p(0)*inv_z, C1p(1)*inv_z);

            Eigen::Matrix<T, 2, 1> residuals = hat_C1uv - C1uv_.head<2>().cast<T>();

            return residuals;
        }

    double t0_,t1_;
    Eigen::Vector3d C0uv_;
    Eigen::Vector3d C1uv_;
    Eigen::Isometry3d T_IC_;
    };


    class JPLProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 1> {
    public:
        JPLProjectionFactor() = delete;

        JPLProjectionFactor(ProjectionBase *_projection_base) : projection_base_(_projection_base) {
        }

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
            return EvaluateWithMinimalJacobians(parameters,
                                                residuals,
                                                jacobians, NULL);
        }

        bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                          double *residuals,
                                          double **jacobians,
                                          double **jacobiansMinimal) const {

            Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
            QuaternionTemplate<double> Qi(parameters[0][3], parameters[0][4], parameters[0][5], parameters[0][6]);



            Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
            QuaternionTemplate<double> Qj(parameters[1][3], parameters[1][4], parameters[1][5], parameters[1][6]);

            double inv_dep = parameters[2][0];

            Eigen::Map<Eigen::Vector2d> error(residuals);
            error = projection_base_->evaluate(Pi, Qi,
                                               Pj, Qj,
                                               inv_dep
                                               );


            return true;
        }


        ProjectionBase *projection_base_;

    };

}
#endif
