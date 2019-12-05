

#ifndef INCLUDE_OKVIS_MEASUREMENTS_HPP_
#define INCLUDE_OKVIS_MEASUREMENTS_HPP_

#include <deque>
#include <vector>
#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#include <Eigen/Dense>
//#include <okvis/Time.hpp>
//#include <okvis/kinematics/Transformation.hpp>
#include <ros/time.h>
#include <Eigen/Geometry>

/**
 * \brief Generic measurements
 *
 * They always come with a timestamp such that we can perform
 * any kind of asynchronous operation.
 * \tparam MEASUREMENT_T Measurement data type.
 */
    template<class MEASUREMENT_T>
    struct Measurement {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ros::Time timeStamp;      ///< Measurement timestamp
        MEASUREMENT_T measurement;  ///< Actual measurement.
        int sensorId = -1;          ///< Sensor ID. E.g. camera index in a multicamera setup

        /// \brief Default constructor.
        Measurement()
                : timeStamp(0.0) {
        }
        /**
         * @brief Constructor
         * @param timeStamp_ Measurement timestamp.
         * @param measurement_ Actual measurement.
         * @param sensorId Sensor ID (optional).
         */
        Measurement(const ros::Time& timeStamp_, const MEASUREMENT_T& measurement_,
                    int sensorId = -1)
                : timeStamp(timeStamp_),
                  measurement(measurement_),
                  sensorId(sensorId) {
        }
    };

/// \brief IMU measurements. For now assume they are synchronized:
    struct ImuSensorReadings {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /// \brief Default constructor.
        ImuSensorReadings()
                : gyroscopes(),
                  accelerometers() {
        }
        /**
         * @brief Constructor.
         * @param gyroscopes_ Gyroscope measurement.
         * @param accelerometers_ Accelerometer measurement.
         */
        ImuSensorReadings(Eigen::Vector3d gyroscopes_,
                          Eigen::Vector3d accelerometers_)
                : gyroscopes(gyroscopes_),
                  accelerometers(accelerometers_) {
        }
        Eigen::Vector3d gyroscopes;     ///< Gyroscope measurement.
        Eigen::Vector3d accelerometers; ///< Accelerometer measurement.
    };

// this is how we store raw measurements before more advanced filling into
// data structures happens:
    typedef Measurement<ImuSensorReadings> ImuMeasurement;
    typedef std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement> > ImuMeasurementDeque;
/// \brief Camera measurement.
    struct CameraData {
        cv::Mat image;  ///< Image.
        std::vector<cv::KeyPoint> keypoints; ///< Keypoints if available.
        bool deliversKeypoints; ///< Are the keypoints delivered too?
    };

    typedef Measurement<CameraData> CameraMeasurement;

    typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;


#endif // INCLUDE_OKVIS_MEASUREMENTS_HPP_
