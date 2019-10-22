#include "vi_sfm/OfflineHfnetDataReader.h"
#include <opencv2/imgproc/imgproc.hpp>

int main () {
    std::string dataset_path = "/home/pang/data/dataset/iros2019slam/corridor/corridor-1-1";

    std::string result_file = "/home/pang/hfnet.txt";
    std::ofstream ofs(result_file);
    OfflineHfnetDataReader offlineHfnetDataReader(dataset_path);
    int query_id = 400;
    int skip_start_id = 700;
    int skip_end_id = 7950;
    auto query_global = offlineHfnetDataReader.getGlobalDescriptor(query_id);


    int best_match_id = 0;
    double best_score = 0.0;
    cv::Mat best_match_image;
    for (int i = skip_end_id; i < offlineHfnetDataReader.size(); i ++) {
        if (i  > skip_start_id && i < skip_end_id) continue;

        auto global = offlineHfnetDataReader.getGlobalDescriptor(i);
        std::cout << global.first << " " << global.second.size() << std::endl;

        double similarity = query_global.second.transpose() * global.second;
        std::cout << "getGlobalDescriptor: " <<  similarity << " " <<  i   << std::endl;

        auto kp = offlineHfnetDataReader.getKeypoints(i);

        std::cout << "kp: " << kp.second.size() << std::endl;

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


    auto query_kp  = offlineHfnetDataReader.getKeypoints(query_id);
    auto best_match_kp = offlineHfnetDataReader.getKeypoints(best_match_id);



    cv::Mat color_query, color_match;
    cv::cvtColor(query_image, color_query, CV_GRAY2RGB);
    cv::cvtColor(best_match_image, color_match, CV_GRAY2RGB);

    for (int i = 0; i < query_kp.second.size(); i ++) {
        cv::Point2f p (query_kp.second.at(i)(0), query_kp.second.at(i)(1));
        cv::circle(color_query, p, 2,cv::Scalar(255,0,0), 2);
    }

    for (int i = 0; i < best_match_kp.second.size(); i ++) {
        cv::Point2f p (best_match_kp.second.at(i)(0), best_match_kp.second.at(i)(1));
        cv::circle(color_match, p, 2,cv::Scalar(0,255,0), 2);
    }

    cv::imshow("query_image", color_query);
    cv::imshow("best_match_image", color_match);
    cv::waitKey();


    return 1;
}