#include "vi_sfm/OfflineHfnetDataReader.h"

int main () {
    std::string dataset_path = "/home/pang/data/dataset/iros2019slam/corridor/corridor-1-1";

    std::string result_file = "/home/pang/hfnet.txt";
    std::ofstream ofs(result_file);
    OfflineHfnetDataReader offlineHfnetDataReader(dataset_path);
    int query_id = 400;
    int skip_start_id = 700;
    int skip_end_id = 7800;
    auto query_global = offlineHfnetDataReader.getGlobalDescriptor(query_id);


    int best_match_id = 0;
    double best_score = 0.0;
    cv::Mat best_match_image;
    for (int i = 0; i < offlineHfnetDataReader.size(); i ++) {
        if (i  > skip_start_id && i < skip_end_id) continue;

        auto global = offlineHfnetDataReader.getGlobalDescriptor(i);
        std::cout << global.first << " " << global.second.size() << std::endl;

        double similarity = query_global.second.transpose() * global.second;
        std::cout << "getGlobalDescriptor: " <<  similarity << " " <<  i   << std::endl;

        cv::Mat image = offlineHfnetDataReader.getImage(i);
        cv::imshow("image", image);
        cv::waitKey(2);

        if (similarity > best_score && i > skip_end_id) {
            best_score = similarity;
            best_match_id = i;
            best_match_image = image;
        }

        ofs << similarity << std::endl;
    }

    ofs.close();


    std::cout << "best_score: " << best_score << std::endl;
    std::cout << "best_match_id: " << best_match_id << std::endl;
    auto query_image = offlineHfnetDataReader.getImage(query_id);

    cv::imshow("query_image", query_image);
    cv::imshow("best_match_image", best_match_image);
    cv::waitKey();


    return 1;
}