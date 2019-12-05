#ifndef INCLUDE_OKVIS_VIOINTERFACE_HPP_
#define INCLUDE_OKVIS_VIOINTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <functional>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#pragma GCC diagnostic pop
//#include <okvis/assert_macros.hpp>
#include <ros/time.h>
#include <Eigen/Geometry>
#include <map>
//#include <okvis/FrameTypedefs.hpp>
//#include <okvis/kinematics/Transformation.hpp>


/**
 * @brief An abstract base class for interfaces between Front- and Backend.
 */
    class VioInterface {
    public:
//        OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

//        typedef std::function<
//                void(const ros::Time &, const Eigen::Isometry3d &)> StateCallback;
//        typedef std::function<
//                void(const ros::Time &, const Eigen::Isometry3d &,
//                     const Eigen::Matrix<double, 9, 1> &,
//                     const Eigen::Matrix<double, 3, 1> &)> FullStateCallback;
        typedef std::function<
                void(
                        const std::vector<ros::Time> &,
                        const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &,
                        const std::vector<Eigen::Matrix<double, 9, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 9, 1>>> &,
                        const std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d> >&)> FullStateCallbackWithExtrinsics;
        typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> EigenImage;
//        typedef std::function<
//                void(const ros::Time &, const okvis::MapPointVector &,
//                     const okvis::MapPointVector &)> LandmarksCallback;

        VioInterface();
        virtual ~VioInterface();

        /// \name Setters
        /// \{

        /// \brief              Set a CVS file where the IMU data will be saved to.
        /// \param csvFile      The file.
        bool setImuCsvFile(std::fstream& csvFile);
        /// \brief              Set a CVS file where the IMU data will be saved to.
        /// \param csvFileName  The filename of a new file.
        bool setImuCsvFile(const std::string& csvFileName);

        /// \brief              Set a CVS file where the tracks (data associations) will be saved to.
        /// \param cameraId     The camera ID.
        /// \param csvFile      The file.
        bool setTracksCsvFile(size_t cameraId, std::fstream& csvFile);
        /// \brief              Set a CVS file where the tracks (data associations) will be saved to.
        /// \param cameraId     The camera ID.
        /// \param csvFileName  The filename of a new file.
        bool setTracksCsvFile(size_t cameraId, const std::string& csvFileName);

        /// \brief              Set a CVS file where the position measurements will be saved to.
        /// \param csvFile      The file.
        bool setPosCsvFile(std::fstream& csvFile);
        /// \brief              Set a CVS file where the position measurements will be saved to.
        /// \param csvFileName  The filename of a new file.
        bool setPosCsvFile(const std::string& csvFileName);

        /// \brief              Set a CVS file where the magnetometer measurements will be saved to.
        /// \param csvFile      The file.
        bool setMagCsvFile(std::fstream& csvFile);
        /// \brief              Set a CVS file where the magnetometer measurements will be saved to.
        /// \param csvFileName  The filename of a new file.
        bool setMagCsvFile(const std::string& csvFileName);

        /// \}
        /// \name Add measurements to the algorithm
        /// \{
        /**
         * \brief              Add a new image.
         * \param stamp        The image timestamp.
         * \param cameraIndex  The index of the camera that the image originates from.
         * \param image        The image.
         * \param keypoints    Optionally aready pass keypoints. This will skip the detection part.
         * \param asKeyframe   Use the new image as keyframe. Not implemented.
         * \warning The frame consumer loop does not support using existing keypoints yet.
         * \warning Already specifying whether this frame should be a keyframe is not implemented yet.
         * \return             Returns true normally. False, if the previous one has not been processed yet.
         */
        virtual bool addImage(const ros::Time & stamp, size_t cameraIndex,
                              const cv::Mat & image,
                              const std::vector<cv::KeyPoint> * keypoints = 0,
                              bool* asKeyframe = 0) = 0;


        /// \brief          Add an IMU measurement.
        /// \param stamp    The measurement timestamp.
        /// \param alpha    The acceleration measured at this time.
        /// \param omega    The angular velocity measured at this time.
        virtual bool addImuMeasurement(const ros::Time & stamp,
                                       const Eigen::Vector3d & alpha,
                                       const Eigen::Vector3d & omega) = 0;

        /// \}
        /// \name Setters
        /// \{


        /// \brief Set the fullStateCallbackWithExtrinsics to be called every time a new state is estimated.
        ///        When an implementing class has an estimate, they can call:
        ///        _fullStateCallbackWithEctrinsics( stamp, T_w_vk, speedAndBiases, omega_S, vector_of_T_SCi);
        ///        where stamp is the timestamp
        ///        and T_w_vk is the transformation (and uncertainty) that
        ///        transforms points from the vehicle frame to the world frame. speedAndBiases contain
        ///        speed in world frame followed by gyro and acc biases.
        ///        omega_S is the rotation speed
        ///        vector_of_T_SCi contains the (uncertain) transformations of extrinsics T_SCi
        virtual void setFullStateCallbackWithExtrinsics(
                const FullStateCallbackWithExtrinsics & fullStateCallbackWithExtrinsics);

        /// \brief Set the landmarksCallback to be called every time a new state is estimated.
        ///        When an implementing class has an estimate, they can call:
        ///        landmarksCallback_( stamp, landmarksVector );
        ///        where stamp is the timestamp
        ///        landmarksVector contains all 3D-landmarks with id.
//        virtual void setLandmarksCallback(
//                const LandmarksCallback & landmarksCallback);

        /**
         * \brief Set the blocking variable that indicates whether the addMeasurement() functions
         *        should return immediately (blocking=false), or only when the processing is complete.
         */
        virtual void setBlocking(bool blocking);

        /// \}

    protected:

        /// \brief Write first line of IMU CSV file to describe columns.
        bool writeImuCsvDescription();
        /// \brief Write first line of position CSV file to describe columns.
        bool writePosCsvDescription();
        /// \brief Write first line of magnetometer CSV file to describe columns.
        bool writeMagCsvDescription();
        /// \brief Write first line of tracks (data associations) CSV file to describe columns.
        bool writeTracksCsvDescription(size_t cameraId);

//        StateCallback stateCallback_; ///< State callback function.
//        FullStateCallback fullStateCallback_; ///< Full state callback function.
        FullStateCallbackWithExtrinsics fullStateCallbackWithExtrinsics_; ///< Full state and extrinsics callback function.
//        LandmarksCallback landmarksCallback_; ///< Landmarks callback function.
        std::shared_ptr<std::fstream> csvImuFile_;  ///< IMU CSV file.
        std::shared_ptr<std::fstream> csvPosFile_;  ///< Position CSV File.
        std::shared_ptr<std::fstream> csvMagFile_;  ///< Magnetometer CSV File
        typedef std::map<size_t, std::shared_ptr<std::fstream>> FilePtrMap;
        FilePtrMap csvTracksFiles_; ///< Tracks CSV Files.
        bool blocking_; ///< Blocking option. Whether the addMeasurement() functions should wait until proccessing is complete.
    };


#endif