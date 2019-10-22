#include "vi_sfm/OfflineHfnetDataReader.h"

int main () {
    std::string dataset_path = "/home/pang/data/dataset/iros2019slam/corridor/corridor-1-1";

    OfflineHfnetDataReader offlineHfnetDataReader(dataset_path);
    for (int i = 0; i < offlineHfnetDataReader.size(); i ++) {
        auto global = offlineHfnetDataReader.getGlobalDescriptor(i);
        std::cout << global.first << " " << global.second.size() << std::endl;

        std::cout << "getGlobalDescriptor: " << global.second.transpose() * global.second << std::endl;
    }

    return 1;
}