
#ifndef INCLUDE_DYNAMIC_SPLINE_PROJECTION_ERROR_HPP_
#define INCLUDE_DYNAMIC_SPLINE_PROJECTION_ERROR_HPP_


#include "spline_vio/utility/utility.h"

#include <ceres/ceres.h>
#include "spline_vio/spline/Quaternion.hpp"
#include "spline_vio/spline/PoseLocalParameter.hpp"
#include "spline_vio/spline/JPL_projection_error.h"
#include "spline_vio/spline/QuaternionSplineUtility.hpp"
#include "../parameters.h"
namespace  JPL {

class SplineProjectionFactor4: public ceres::SizedCostFunction<2,7,7,7,7,1,1> {
    public:
    SplineProjectionFactor4() = delete;

    SplineProjectionFactor4(std::shared_ptr<ProjectionBase> projection_base,
                const double& u0, const double& u1) :
                projection_base_(projection_base),
                        t0_(u0), t1_(u1){
            sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
        }


        template <typename T>
        bool operator()(T const*  parameters_0,
                        T const*  parameters_1,
                        T const*  parameters_2,
                        T const*  parameters_3,
                        T const*  param_inv_depth,
                        T const*  param_delta_t,
                        T* residual) const {

            int VertexNum = 4;
            std::vector<Pose<T>> T_vec(VertexNum);

            T_vec[0] =  Pose<T>(parameters_0);
            T_vec[1] =  Pose<T>(parameters_1);
            T_vec[2] =  Pose<T>(parameters_2);
            T_vec[3] =  Pose<T>(parameters_3);


            T inv_depth = T(param_inv_depth[0]);


            const int SKIP = VertexNum - 4;

            // for i
            QuaternionTemplate<T> Q00 = T_vec[0].rotation();
            QuaternionTemplate<T> Q01 = T_vec[1].rotation();
            QuaternionTemplate<T> Q02 = T_vec[2].rotation();
            QuaternionTemplate<T> Q03 = T_vec[3].rotation();

            Eigen::Matrix<T,3,1> t00 = T_vec[0].translation();
            Eigen::Matrix<T,3,1> t01 = T_vec[1].translation();
            Eigen::Matrix<T,3,1> t02 = T_vec[2].translation();
            Eigen::Matrix<T,3,1> t03 = T_vec[3].translation();



            T ts0 = T(t0_ + param_delta_t[0]);
            T ts1 = T(t1_ + param_delta_t[0]);
            T  Beta01 = QSUtility::beta1(ts0);
            T  Beta02 = QSUtility::beta2((ts0));
            T  Beta03 = QSUtility::beta3((ts0));

//            std::cout << "U: " << t0_ << " " << Beta01 << " " << Beta02 << " " << Beta03 << std::endl;


            T  Beta11 = QSUtility::beta1((ts1));
            T  Beta12 = QSUtility::beta2((ts1));
            T  Beta13 = QSUtility::beta3((ts1));

            Eigen::Matrix<T,3,1> phi01 = QSUtility::Phi<T>(Q00,Q01);
            Eigen::Matrix<T,3,1> phi02 = QSUtility::Phi<T>(Q01,Q02);
            Eigen::Matrix<T,3,1> phi03 = QSUtility::Phi<T>(Q02,Q03);

            QuaternionTemplate<T> r_01 = QSUtility::r(Beta01,phi01);
            QuaternionTemplate<T> r_02 = QSUtility::r(Beta02,phi02);
            QuaternionTemplate<T> r_03 = QSUtility::r(Beta03,phi03);



            Eigen::Matrix<T,3,1> Pi = t00 + Beta01*(t01 - t00) +  Beta02*(t02 - t01) + Beta03*(t03 - t02);
            QuaternionTemplate<T> Qi = quatLeftComp(Q00)*quatLeftComp(r_01)*quatLeftComp(r_02)*r_03;
            if (Qi[3] < T(0)) Qi = -Qi;




            // for j
            QuaternionTemplate<T> Q10 = T_vec[0 + SKIP].rotation();
            QuaternionTemplate<T> Q11 = T_vec[1 + SKIP].rotation();
            QuaternionTemplate<T> Q12 = T_vec[2 + SKIP].rotation();
            QuaternionTemplate<T> Q13 = T_vec[3 + SKIP].rotation();

            Eigen::Matrix<T,3,1> t10 = T_vec[0 + SKIP].translation();
            Eigen::Matrix<T,3,1> t11 = T_vec[1 + SKIP].translation();
            Eigen::Matrix<T,3,1> t12 = T_vec[2 + SKIP].translation();
            Eigen::Matrix<T,3,1> t13 = T_vec[3 + SKIP].translation();




            Eigen::Matrix<T,3,1> phi11 = QSUtility::Phi<T>(Q10,Q11);
            Eigen::Matrix<T,3,1> phi12 = QSUtility::Phi<T>(Q11,Q12);
            Eigen::Matrix<T,3,1> phi13 = QSUtility::Phi<T>(Q12,Q13);

            QuaternionTemplate<T> r_11 = QSUtility::r(Beta11,phi11);
            QuaternionTemplate<T> r_12 = QSUtility::r(Beta12,phi12);
            QuaternionTemplate<T> r_13 = QSUtility::r(Beta13,phi13);




            Eigen::Matrix<T,3,1> Pj = t10 + Beta11*(t11 - t10) +  Beta12*(t12 - t11) + Beta13*(t13 - t12);
            QuaternionTemplate<T> Qj = quatLeftComp(Q10)*quatLeftComp(r_11)*quatLeftComp(r_12)*r_13;
            if (Qj[3] < T(0)) Qj = -Qj;


//
//            std::cout << "eva_T_i: " << Pi.transpose() << " " <<  Qi.transpose()  << std::endl;
//            std::cout << "eva_T_j: " << Pj.transpose() << " " <<  Qj.transpose()  << std::endl;



            // compute error
            Eigen::Map<Eigen::Matrix<T, 2, 1>> error(residual);
            error = projection_base_->evaluate<T>(Pi, Qi,
                                                 Pj, Qj,
                                                 inv_depth,
                                                 NULL);

            error = sqrt_info * error;


        }

        bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
            if (jacobians == nullptr) {
                return ceres::internal::VariadicEvaluate<
                        SplineProjectionFactor4, double, 7, 7, 7, 7, 1,1,0,0,0,0>
                ::Call(*this, parameters, residuals);
            }


            bool success =  ceres::internal::AutoDiff<SplineProjectionFactor4, double,
                    7,7,7,7,1,1>::Differentiate(
                    *this,
                    parameters,
                    2,
                    residuals,
                    jacobians);

            return true;


        }

        std::shared_ptr<ProjectionBase> projection_base_;
        double t0_;
        double t1_;
        Eigen::Matrix2d sqrt_info;


    };
}
#endif
