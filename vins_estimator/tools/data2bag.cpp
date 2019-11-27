#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <rosbag/bag.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <Eigen/Geometry>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include  <opencv2/highgui/highgui.hpp>

#include "csv.h"




class SegwayRobotDatasetReader {
public:
    const std::string FISHEYE_TOPIC = "/fisheye";
    const std::string IMU_TOPIC = "/imu";


    SegwayRobotDatasetReader() {}

    void setDataPath(std::string path) {
        datasetPath_ = path;
    }

    explicit SegwayRobotDatasetReader(const std::string& datasetPath)
            : datasetPath_(datasetPath){
    }

    bool readData(bool use_imu=true) {

        data_bag_file_name_ = datasetPath_ + "/fisheye_imu1.bag";

        std::cout << "Saving bag to :" << data_bag_file_name_ << std::endl;
        data_bag_ptr_ =  std::make_shared<rosbag::Bag>(data_bag_file_name_,
                                                       rosbag::bagmode::Read|rosbag::bagmode::Write);

        /// load fisheye
        std::string fisheyeListPath = datasetPath_ + "/fisheye_timestamps.txt";
        if (!loadFisheye(fisheyeListPath, FISHEYE_TOPIC)) {
            std::cout << "Can Not load fisheye data!" << std::endl;
            return false;
        }

        /// load imu
        std::string imuListPath = datasetPath_ + "/imu_raw.txt";
        if (use_imu) {
            if (!loadIMUReadings(imuListPath, IMU_TOPIC)) {
                std::cout << "Can Not load imu data!" << std::endl;
                return false;

            }
        }
        std::cout<< "data bag size: " << data_bag_ptr_->getSize() << std::endl;
        data_bag_ptr_->close();

        return true;
    }

    bool loadFisheye(const std::string list_file, const std::string topic) {
        std::ifstream ifs(list_file);
        if (!ifs.is_open()) {
            std::cerr << "Failed to open fisheye list file: " << list_file
                      << std::endl;
            return false;
        }

        bool first_msg = true;
        ros::Time last_timestamp = ros::Time(0);
        ros::Time cur_ts = ros::Time(0);
        std::string one_line;
        int image_seq = 0;
        while (!ifs.eof()) {
            std::getline(ifs, one_line);

            std::stringstream stream(one_line);
            std::string image_name, timestamp_us_str, tmp;

            stream >> image_name >> timestamp_us_str >> tmp >> tmp;
            if (image_name.empty())
                break;

            int64_t sec = std::stoi(timestamp_us_str.substr(0,10));
            int64_t usec = std::stoi(timestamp_us_str.substr(10,6));
//            std::cout << "sec: " << sec << " usec: " << usec << std::endl;
            cur_ts = ros::Time(sec, usec*1000);

            if(first_msg) {
                start_times_.push_back(cur_ts);
                first_msg = false;
            }
            if (cur_ts  <= last_timestamp) {
                std::cout << "BAD fisheye ts: " << cur_ts << " while the last is: "
                          << last_timestamp << std::endl;
                continue;
            }

            last_timestamp = cur_ts;

            sensor_msgs::Image ros_image_msg;
            std_msgs::Header header;
            header.stamp = cur_ts;
            header.seq = image_seq;
            header.frame_id ="cam0";

            std::cout << "fisheye: " << datasetPath_ + "/" + image_name << std::endl;
            std::string image_file = datasetPath_ + "/" + image_name;

            cv::Mat image = cv::imread(image_file, CV_LOAD_IMAGE_GRAYSCALE);


            cv::imshow("image", image);
            cv::waitKey(1);

            cv_bridge::CvImage img_bridge;
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image);
            img_bridge.toImageMsg(ros_image_msg);

            data_bag_ptr_->write("/mav/cam0", cur_ts, ros_image_msg);
            image_seq  ++;
        }

        end_times_.push_back(cur_ts);
        ifs.close();
        std::cout << "load " << topic << " data from: "
                  << start_times_.back() << " to " << end_times_.back() << " total " << image_seq
                  << " with duration: " << end_times_.back().toSec() - start_times_.back().toSec()
                  << " s."<< std::endl;
        return true;
    }


    bool loadIMUReadings(const std::string list_file, const std::string topic) {

        bool first_msg = true;
        ros::Time last_timestamp = ros::Time(0);
        ros::Time cur_ts = ros::Time(0);
        std::string one_line;
        int imu_seq = 0;

        /// Note: imu file is a comma separated value (CSV) files
        io::CSVReader<7> in(list_file);
        std::string  timestamp_us_str, tmp;
        double ax, ay, az, gx, gy, gz;
        while(in.read_row(timestamp_us_str, ax, ay,az, gx,gy,gz)){
            if (timestamp_us_str.empty())
                break;

            int64_t sec = std::stoi(timestamp_us_str.substr(0,10));
            int64_t usec = std::stoi(timestamp_us_str.substr(10,6));
//            std::cout << "sec: " << sec << " usec: " << usec << std::endl;
            cur_ts = ros::Time(sec, usec*1000);

            if(first_msg) {
                start_times_.push_back(cur_ts);
                first_msg = false;
            }
            if (cur_ts  <= last_timestamp) {
                std::cout << "BAD imu ts: " << cur_ts << " while the last is: "
                          << last_timestamp << std::endl;
                continue;
            }

            last_timestamp = cur_ts;

            sensor_msgs::Imu imu;
            imu.header.stamp = cur_ts;
            imu.header.seq = imu_seq;

            imu.linear_acceleration.x = ax;
            imu.linear_acceleration.y = ay;
            imu.linear_acceleration.z = az;

            imu.angular_velocity.x = gx;
            imu.angular_velocity.y = gy;
            imu.angular_velocity.z = gz;

            data_bag_ptr_->write(topic, cur_ts, imu);
            imu_seq  ++;
        }

        end_times_.push_back(cur_ts);

        std::cout << "load " << topic << " data from: "
                  << start_times_.back() << " to " << end_times_.back() << " total " << imu_seq
                  << " with duration: " << end_times_.back().toSec() - start_times_.back().toSec()
                  << " s."<< std::endl;
        return true;
    }



    inline std::string getDataBagFileName() {
        return data_bag_file_name_;
    }
    inline std::shared_ptr<rosbag::Bag> getDataBagPtr() const {
        return data_bag_ptr_;
    }

    inline ros::Time getBagStartTs() {
        return bag_start_ts_;
    }

    inline ros::Time getBagEndTs() {
        return bag_end_ts_;
    }





private:
    std::string datasetPath_;
    std::string intermediate_bag_out_folder_;
    std::string data_bag_file_name_;
    std::shared_ptr<rosbag::Bag> data_bag_ptr_;
    std::vector<ros::Time> start_times_, end_times_;
    ros::Time bag_start_ts_, bag_end_ts_;


};


int main (int argc, char ** argv) {

    std::string dataset_folder_path;
    std::string temp_folder;
    if (argc ==2 ) {
        dataset_folder_path = std::string(argv[1]);
        temp_folder = dataset_folder_path;
    } else {
        std::cerr << "Usage: test_data_source dataset_path [temp_folder]"  << std::endl;
        return -1;
    }

    SegwayRobotDatasetReader datasetReader(dataset_folder_path);
    datasetReader.readData(true);

    return 0;

}