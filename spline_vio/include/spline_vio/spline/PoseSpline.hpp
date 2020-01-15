#ifndef POSE_SPLINE_H_
#define POSE_SPLINE_H_


#include "spline_vio/spline/BSplineBase.hpp"
#include "spline_vio/spline/Pose.hpp"
#include <maplab-common/proto-serialization-helper.h>
#include <maplab-common/file-system-tools.h>


class PoseSpline : public BSplineBase<Pose<double>, 4> {

public:
    typedef  BSplineBase<Pose<double>, 4> base_t;
    typedef  base_t::base_element_type ElementType;
    PoseSpline();

    PoseSpline( double interval);

    virtual ~PoseSpline() {}

    void initialPoseSpline(const std::vector<std::pair<ros::Time, Pose<double>>>& Meas) ;

    Pose<double> evalPoseSpline(real_t t);
    Eigen::Vector3d evalLinearVelocity(real_t t );

    Eigen::Vector3d evalLinearAccelerator(real_t t, const Eigen::Vector3d& gravity);
    Eigen::Vector3d evalOmega(real_t t);

    void serialize(spline_vio::proto::PoseSpline* proto) const ;
    void deserialize(
            const spline_vio::proto::PoseSpline& proto);

    void save (const std::string& file) const {
        spline_vio::proto::PoseSpline poseSpline_proto;
        serialize(&poseSpline_proto);
        std::string path, file_name;
        common::splitPathAndFilename(file, &path, &file_name);
        common::proto_serialization_helper::serializeProtoToFile(path, file_name, poseSpline_proto, true);
    }
    void load(const std::string& file ) {
        spline_vio::proto::PoseSpline poseSpline_proto;
        std::string path, file_name;
        common::splitPathAndFilename(file, &path, &file_name);
        common::proto_serialization_helper::parseProtoFromFile(path, file_name, &poseSpline_proto, true);
        deserialize(poseSpline_proto);
    }

private:
    std::vector<std::pair<ros::Time, Pose<double>>> samples_;
};
#endif