//
// Created by pang on 2019/10/22.
//

#include "OfflineHfnetDataReader.h"
#include "file-system-tools.h"
#include <ros/time.h>
OfflineHfnetDataReader::OfflineHfnetDataReader(const std::string& dataset):dataset_path_(dataset),
cur_cnt_(0) {
    image_files_folder_ = dataset_path_ + "/fisheye1/";
    kp_files_folder_ = dataset_path_ + "/fisheye1_hfnet_kps/";
    local_des_files_folder_ = dataset_path_ + "/fisheye1_hfnet_local_des/";
    global_des_files_folder_ = dataset_path_ + "/fisheye1_hfnet_global_des/";

    if (utility::pathExists(image_files_folder_)) {
        utility::getAllFilesInFolder(image_files_folder_, &image_file_names_);
        std::cout<<"disp_list: " << image_file_names_.size() << std::endl;
        std::sort(image_file_names_.begin(),image_file_names_.end(), [](std::string a, std::string b) {
            return !utility::compareNumericPartsOfStrings(a,b);
        });
    }

    if (utility::pathExists(kp_files_folder_)) {
        utility::getAllFilesInFolder(kp_files_folder_, &kp_file_names_);
        std::cout<<"disp_list: " << kp_file_names_.size() << std::endl;
        std::sort(kp_file_names_.begin(),kp_file_names_.end(), [](std::string a, std::string b) {
            return !utility::compareNumericPartsOfStrings(a,b);
        });
    }

    if (utility::pathExists(local_des_files_folder_)) {
        utility::getAllFilesInFolder(local_des_files_folder_, &local_file_names_);
        std::cout<<"disp_list: " << local_file_names_.size() << std::endl;
        std::sort(local_file_names_.begin(),local_file_names_.end(), [](std::string a, std::string b) {
            return !utility::compareNumericPartsOfStrings(a,b);
        });
    }

    if (utility::pathExists(global_des_files_folder_)) {
        utility::getAllFilesInFolder(global_des_files_folder_, &global_file_names_);
        std::cout<<"disp_list: " << global_file_names_.size() << std::endl;
        std::sort(global_file_names_.begin(),global_file_names_.end(), [](std::string a, std::string b) {
            return !utility::compareNumericPartsOfStrings(a,b);
        });
    }


    std::cout << "kp_file_names_: " << kp_file_names_.size() << std::endl;
    std::cout << "local_file_names_: " << local_file_names_.size() << std::endl;
    std::cout << "global_file_names_: " << global_file_names_.size() << std::endl;




}

cv::Mat  OfflineHfnetDataReader::getImage(int cnt) {
    std::string list_file = image_file_names_.at(cnt);
    return cv::imread(list_file,CV_LOAD_IMAGE_GRAYSCALE);
}


std::pair<ros::Time, GlobalDescriptor>  OfflineHfnetDataReader::getGlobalDescriptor(int cnt) {
    std::string list_file = global_file_names_.at(cnt);

    std::pair<ros::Time, GlobalDescriptor> global;
    std::ifstream ifs(list_file);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open kp list file: " << list_file
                  << std::endl;
        return global;
    }

    std::string path, file_name, raw_file_name, ext;
    utility::splitPathAndFilename(list_file, &path, &file_name);
    utility::splitFilePathAndExtension(file_name, &raw_file_name, &ext);


    uint64_t ts = std::stoull(raw_file_name);

    bool first_msg = true;
    ros::Time last_timestamp = ros::Time(0);
    ros::Time cur_ts = ros::Time(0);
    std::string one_line;
    int image_seq = 0;

    global.first = ros::Time(ts / (double)1e6);

    GlobalDescriptor globalDescriptor;


    int i = 0;
    while (!ifs.eof() && i < 4096) {
        std::getline(ifs, one_line);
        std::stringstream stream(one_line);
        double u;
        stream >> u;

        globalDescriptor(i, 0) = u;
        i ++;
    }


    global.second = globalDescriptor;

    ifs.close();

    return global;

}



