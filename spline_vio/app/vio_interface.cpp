

#include <fstream>

#include "vio_interface.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

/// \brief okvis Main namespace of this package.
    VioInterface::VioInterface() {
    }
    VioInterface::~VioInterface() {
        if (csvImuFile_)
            csvImuFile_->close();
        if (csvPosFile_)
            csvPosFile_->close();
        if (csvMagFile_)
            csvMagFile_->close();
        // also close all registered tracks files
        for (FilePtrMap::iterator it = csvTracksFiles_.begin();
             it != csvTracksFiles_.end(); ++it) {
            if (it->second)
                it->second->close();
        }
    }



// Set the callback to be called every time a new state is estimated.
    void VioInterface::setStateCallback(const StateCallback & stateCallback) {
        stateCallback_ = stateCallback;
    }

// Set the fullStateCallback to be called every time a new state is estimated.
    void VioInterface::setFullStateCallback(
            const FullStateCallback & fullStateCallback) {
        fullStateCallback_ = fullStateCallback;
    }

// Set the callback to be called every time a new state is estimated.
    void VioInterface::setFullStateCallbackWithExtrinsics(
            const FullStateCallbackWithExtrinsics & fullStateCallbackWithExtrinsics) {
        fullStateCallbackWithExtrinsics_ = fullStateCallbackWithExtrinsics;
    }

// Set the fullStateCallbackWithExtrinsics to be called every time a new state is estimated.
//    void VioInterface::setLandmarksCallback(
//            const LandmarksCallback & landmarksCallback) {
//        landmarksCallback_ = landmarksCallback;
//    }

// Set the blocking variable that indicates whether the addMeasurement() functions
// should return immediately (blocking=false), or only when the processing is complete.
    void VioInterface::setBlocking(bool blocking) {
        blocking_ = blocking;
    }

// Write first line of IMU CSV file to describe columns.
    bool VioInterface::writeImuCsvDescription() {
        if (!csvImuFile_)
            return false;
        if (!csvImuFile_->good())
            return false;
        *csvImuFile_ << "timestamp" << ", " << "omega_tilde_WS_S_x" << ", "
                     << "omega_tilde_WS_S_y" << ", " << "omega_tilde_WS_S_z" << ", "
                     << "a_tilde_WS_S_x" << ", " << "a_tilde_WS_S_y" << ", "
                     << "a_tilde_WS_S_z" << std::endl;
        return true;
    }

// Write first line of position CSV file to describe columns.
    bool VioInterface::writePosCsvDescription() {
        if (!csvPosFile_)
            return false;
        if (!csvPosFile_->good())
            return false;
        *csvPosFile_ << "timestamp" << ", " << "pos_E" << ", " << "pos_N" << ", "
                     << "pos_U" << std::endl;
        return true;
    }

// Write first line of magnetometer CSV file to describe columns.
    bool VioInterface::writeMagCsvDescription() {
        if (!csvMagFile_)
            return false;
        if (!csvMagFile_->good())
            return false;
        *csvMagFile_ << "timestamp" << ", " << "mag_x" << ", " << "mag_y" << ", "
                     << "mag_z" << std::endl;
        return true;
    }

// Write first line of tracks (data associations) CSV file to describe columns.
    bool VioInterface::writeTracksCsvDescription(size_t cameraId) {
        if (!csvTracksFiles_[cameraId])
            return false;
        if (!csvTracksFiles_[cameraId]->good())
            return false;
        *csvTracksFiles_[cameraId] << "timestamp" << ", " << "landmark_id" << ", "
                                   << "z_tilde_x" << ", " << "z_tilde_y" << ", " << "z_tilde_stdev" << ", "
                                   << "descriptor" << std::endl;
        return false;
    }

// Set a CVS file where the IMU data will be saved to.
    bool VioInterface::setImuCsvFile(std::fstream& csvFile) {
        if (csvImuFile_) {
            csvImuFile_->close();
        }
        csvImuFile_.reset(&csvFile);
        writeImuCsvDescription();
        return csvImuFile_->good();
    }

// Set a CVS file where the IMU data will be saved to.
    bool VioInterface::setImuCsvFile(const std::string& csvFileName) {
        csvImuFile_.reset(
                new std::fstream(csvFileName.c_str(), std::ios_base::out));
        writeImuCsvDescription();
        return csvImuFile_->good();
    }

// Set a CVS file where the position measurements will be saved to.
    bool VioInterface::setPosCsvFile(std::fstream& csvFile) {
        if (csvPosFile_) {
            csvPosFile_->close();
        }
        csvPosFile_.reset(&csvFile);
        writePosCsvDescription();
        return csvPosFile_->good();
    }

// Set a CVS file where the position measurements will be saved to.
    bool VioInterface::setPosCsvFile(const std::string& csvFileName) {
        csvPosFile_.reset(
                new std::fstream(csvFileName.c_str(), std::ios_base::out));
        writePosCsvDescription();
        return csvPosFile_->good();
    }

// Set a CVS file where the magnetometer measurements will be saved to.
    bool VioInterface::setMagCsvFile(std::fstream& csvFile) {
        if (csvMagFile_) {
            csvMagFile_->close();
        }
        csvMagFile_.reset(&csvFile);
        writeMagCsvDescription();
        return csvMagFile_->good();
    }

// Set a CVS file where the magnetometer measurements will be saved to.
    bool VioInterface::setMagCsvFile(const std::string& csvFileName) {
        csvMagFile_.reset(
                new std::fstream(csvFileName.c_str(), std::ios_base::out));
        writeMagCsvDescription();
        return csvMagFile_->good();
    }

// Set a CVS file where the tracks (data associations) will be saved to.
    bool VioInterface::setTracksCsvFile(size_t cameraId, std::fstream& csvFile) {
        if (csvTracksFiles_[cameraId]) {
            csvTracksFiles_[cameraId]->close();
        }
        csvTracksFiles_[cameraId].reset(&csvFile);
        writeTracksCsvDescription(cameraId);
        return csvTracksFiles_[cameraId]->good();
    }

// Set a CVS file where the tracks (data associations) will be saved to.
    bool VioInterface::setTracksCsvFile(size_t cameraId,
                                        const std::string& csvFileName) {
        csvTracksFiles_[cameraId].reset(
                new std::fstream(csvFileName.c_str(), std::ios_base::out));
        writeTracksCsvDescription(cameraId);
        return csvTracksFiles_[cameraId]->good();
    }

