#ifndef _BSPLINE_BASE_H_
#define _BSPLINE_BASE_H_

#include <vector>
#include <Eigen/Core>
#include <glog/logging.h>
#include "TypeTraits.hpp"
#include <ros/time.h>
#include "spline_vio/utility/eigen-proto.h"
#include "spline_vio/spline.pb.h"

using real_t = double;

template <typename ElementType, int SplineOrder>
class BSplineBase {
public:
    typedef  ElementType base_element_type;
    typedef Eigen::Matrix<double,TypeTraits<ElementType>::Dim,1> StateVector;
    BSplineBase(double interval):mSplineOrder(SplineOrder),
                                 mTimeInterval(interval) {};
    virtual ~BSplineBase() {
        clear();
    };

    int spline_order() const {
        return SplineOrder;
    }

    /**
     *
     * @return The degree of polynomial used by the spline.
     */
    int polynomialDegree() const {
        return SplineOrder - 1;
    }

    /**
     *
     * @return the minimum number of knots required to have at least one valid
     * time segment.
     */
    int minimumKnotsRequired() const
    {
        return numKnotsRequired(1);
    }

    int numCoefficientsRequired(int num_time_segments) const
    {
        return num_time_segments + SplineOrder - 1;
    }

    int numKnotsRequired(int num_time_segments) const
    {
        return numCoefficientsRequired(num_time_segments) + SplineOrder;
    }

    real_t t_min() const
    {
        CHECK_GE((int)knots_.size(), minimumKnotsRequired())
            << "The B-spline is not well initialized";
        return knots_[SplineOrder - 1];
    }

    real_t t_max() const
    {
        CHECK_GE((int)knots_.size(), minimumKnotsRequired())
            << "The B-spline is not well initialized";
        return knots_[knots_.size() - SplineOrder];
    }

    std::pair<real_t,int> computeTIndex(real_t t) const
    {
        CHECK_GE(t, t_min()) << "The time is out of range by " << (t - t_min());

        //// HACK - avoids numerical problems on initialisation
        if (std::abs(t_max() - t) < 1e-10)
        {
            t = t_max();
        }
        //// \HACK

        CHECK_LE(t, t_max())
            << "The time is out of range by " << (t_max() - t);
        std::vector<real_t>::const_iterator i;
        if(t == t_max())
        {
            // This is a special case to allow us to evaluate the spline at the boundary of the
            // interval. This is not stricly correct but it will be useful when we start doing
            // estimation and defining knots at our measurement times.
            i = knots_.end() - SplineOrder;
        }
        else
        {
            i = std::upper_bound(knots_.begin(), knots_.end(), t);
        }
        //CHECK_NE(i, knots_.end()) << "Something very bad has happened in computeTIndex(" << t << ")";

        // Returns the index of the knot segment this time lies on and the width of this knot segment.
        return std::make_pair(*i - *(i-1),(i - knots_.begin()) - 1);

    }

    std::pair<real_t,int> computeUAndTIndex(real_t t) const
    {
        std::pair<real_t,int> ui = computeTIndex(t);

        int index = ui.second;
        real_t denom = ui.first;

        if(denom <= 0.0)
        {
            // The case of duplicate knots.
            //std::cout << "Duplicate knots\n";
            return std::make_pair(0, index);
        }
        else
        {
            real_t u = (t - knots_[index])/denom;

            return std::make_pair(u, index);
        }
    }

    void setTimeInterval(double timeInterval){
        mTimeInterval = timeInterval;

    }
    double getTimeInterval(){
        return mTimeInterval ;

    }
    bool isTsEvaluable(double ts){
        return ts >= t_min() && ts < t_max();

    }

    void initialSplineKnot(double t){
        // Initialize the spline so that it interpolates the two points
        // and moves between them with a constant velocity.

        // How many knots are required for one time segment?
        int K = numKnotsRequired(1);
        // How many coefficients are required for one time segment?
        int C = numCoefficientsRequired(1);

        // Initialize a uniform knot sequence
        real_t dt = mTimeInterval;
        std::vector<real_t> knots(K);
        for(int i = 0; i < K; i++)
        {
            knots[i] = t + (i - SplineOrder + 1) * dt;
        }

        knots_ = knots;

        for(int i = 0; i < C; i++){
            initialNewControlPoint();
        }
    }



    void addControlPointsUntil(double t){
        if(getControlPointNum() == 0){
            initialSplineKnot(t);
        }else if(getControlPointNum() >= numCoefficientsRequired(1) ){
            if(t < t_min()){
                std::cerr<<"[Error] Inserted t is smaller than t_min()！"<<std::endl;
//                LOG(FATAL) << "Inserted "<<Time(t)<<" is smaller than t_min() "<<Time(t_min())<<std::endl;
            }else if(t >= t_max()){
                // add new knot and control Points
                while(t >= t_max()){
                    knots_.push_back(knots_.back() + mTimeInterval); // append one;
                    initialNewControlPoint();
                }
            }
        }

        CHECK_EQ(knots_.size() - SplineOrder, getControlPointNum());

    }

    void removeControlPointsUntil(double t){
        if (knots_.size() < numKnotsRequired(1)) return;
        std::pair<real_t,int> ui = computeUAndTIndex(t);
        int need_pop = ui.second - (SplineOrder -1);
        for (int i = 0; i < need_pop; i++) {
            knots_.erase(knots_.begin());

            mControlPointsParameter.erase(mControlPointsParameter.begin());
        }

    }


    inline size_t getControlPointNum(){
        return mControlPointsParameter.size();
    }

    inline double* getControlPoint(unsigned int i){
        return mControlPointsParameter.at(i).data();
    }

    void clear() {
        knots_.clear();
        mControlPointsParameter.clear();
    }

    std::vector<double> getKnots() {
        return knots_;
    }

    void serialize(spline_vio::proto::BaseSpline* proto) const  {
        CHECK_NOTNULL(proto);

        proto->set_spline_dt(mTimeInterval);
        proto->set_spline_order(mSplineOrder);


        proto->set_knot_cnt(knots_.size());
        proto->set_control_point_cnt(mControlPointsParameter.size());
        Eigen::VectorXd knots_eigen(knots_.size());
        for (int i = 0; i < knots_.size(); i++) {
            knots_eigen(i) = knots_.at(i);

        }
        common::eigen_proto::serialize(knots_eigen, proto->mutable_knot());

        Eigen::Matrix<double, TypeTraits<ElementType>::Dim, Eigen::Dynamic >
                cps_eigen(int(TypeTraits<ElementType>::Dim), mControlPointsParameter.size());
        for (int i = 0; i < mControlPointsParameter.size(); i ++) {
            cps_eigen.col(i) = mControlPointsParameter.at(i);
        }
        common::eigen_proto::serialize(cps_eigen, proto->mutable_control_point());

    }

    void deserialize(
            const spline_vio::proto::BaseSpline& proto) {
        mSplineOrder = proto.spline_order();
        mTimeInterval = proto.spline_dt();
        int64_t knots_cnt = proto.knot_cnt();
        const int64_t cp_cnt = proto.control_point_cnt();
        Eigen::VectorXd knots_eigen(knots_cnt);
        common::eigen_proto::deserialize( proto.knot(), &knots_eigen);
        knots_.clear();
        mControlPointsParameter.clear();

        for (int i = 0; i <knots_cnt ; i++ ) {
            knots_.push_back(knots_eigen(i));
        }

        Eigen::Matrix<double, TypeTraits<ElementType>::Dim, Eigen::Dynamic > 
                cps_eigen(int(TypeTraits<ElementType>::Dim), cp_cnt);
        common::eigen_proto::deserialize( proto.control_point(), &cps_eigen);

        for (int i = 0; i < cp_cnt; i++) {
            mControlPointsParameter.push_back(cps_eigen.col(i));
        }

    }



private:
    void initialNewControlPoint(){
        typename TypeTraits<ElementType>::TypeT zero_ele = TypeTraits<ElementType>::zero();
        Eigen::Map<StateVector> data(zero_ele.data());

        mControlPointsParameter.push_back(data);
    }

    /// The knot sequence used by the B-spline.
    std::vector<real_t> knots_;

    std::vector<StateVector> mControlPointsParameter;
    int mSplineOrder;
    double mTimeInterval;
};
#endif