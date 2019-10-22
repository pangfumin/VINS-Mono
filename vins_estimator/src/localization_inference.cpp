#include "vi_sfm/OfflineHfnetDataReader.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/EquidistantCamera.h"

std::vector< std::vector< cv::DMatch > > simple_match(
        const std::vector<LocalDescriptor> & des0, const std::vector<cv::KeyPoint>& kps0,
        const std::vector<LocalDescriptor> & des1, const std::vector<cv::KeyPoint>& kps1,
        const double th ) {
    std::vector< std::vector< cv::DMatch > > matches;
    for (int i = 0 ; i  < des0.size(); i ++) {
        LocalDescriptor d0 = des0.at(i);
        double best = 0;
        int best_id;
        for (int j = 0 ; j < des1.size(); j ++) {
           double score = d0.transpose() * des1.at(j);
           if (score > best) {
               best = score;
               best_id = j;
           }
        }

        // check
        if (best > th && abs (kps0.at(i).pt.y - kps1.at(best_id).pt.y) < 50) {
            std::vector< cv::DMatch > match;
            match.push_back(cv::DMatch(i,best_id, 0));
            matches.push_back(match);

        }
    }
    return matches;
}

void reduceVector(std::vector< std::vector< cv::DMatch > > &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

int main () {
    std::string dataset_path = "/home/pang/data/dataset/iros2019slam/corridor/corridor-1-1";

    std::string result_file = "/home/pang/hfnet.txt";
    std::ofstream ofs(result_file);
    OfflineHfnetDataReader offlineHfnetDataReader(dataset_path);
    int query_id = 400;
    int skip_start_id = 700;
    int skip_end_id = 7950;
    auto query_global = offlineHfnetDataReader.getGlobalDescriptor(query_id);
    int image_width =  848;
    int image_height =  800;
    double k2 = -7.3047108016908169e-03;
    double k3 = 4.3499931693077087e-02;
    double k4 = -4.1283041238784790e-02;
    double k5 = 7.6524601317942142e-03;
    double mu = 2.8498089599609375e+02;
    double mv = 2.8610238647460938e+02;
    double u0 = 4.2524438476562500e+02;
    double v0 = 3.9846759033203125e+02;
    camodocal::EquidistantCamera equidistantCamera("camera",
            image_width, image_height,
            k2, k3, k4, k5, mu, mv, u0, v0);

    int best_match_id = 0;
    double best_score = 0.0;
    cv::Mat best_match_image;
    for (int i = skip_end_id; i < offlineHfnetDataReader.size(); i ++) {
        if (i  > skip_start_id && i < skip_end_id) continue;

        auto global = offlineHfnetDataReader.getGlobalDescriptor(i);
        std::cout << global.first << " " << global.second.size() << std::endl;

        double similarity = query_global.second.transpose() * global.second;
        std::cout << "getGlobalDescriptor: " <<  similarity << " " <<  i   << std::endl;

//        auto kp = offlineHfnetDataReader.getKeypoints(i);
//
//        std::cout << "kp: " << kp.second.size() << std::endl;

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

    auto query_des = offlineHfnetDataReader.getLocalDescriptor(query_id);
    auto best_match_des = offlineHfnetDataReader.getLocalDescriptor(best_match_id);

    std::cout << "query_kp: " << query_kp.second.size() << " " << query_des.second.size() << std::endl;
    std::cout << "best_match_kp: " << best_match_kp.second.size() << " " << best_match_des.second.size() << std::endl;




    cv::Mat color_query, color_match;
    cv::cvtColor(query_image, color_query, CV_GRAY2RGB);
    cv::cvtColor(best_match_image, color_match, CV_GRAY2RGB);

    std::vector< cv::KeyPoint > kps0, kps1;
    for (int i = 0; i < query_kp.second.size(); i ++) {
        cv::Point2f p (query_kp.second.at(i)(0), query_kp.second.at(i)(1));
        cv::circle(color_query, p, 2,cv::Scalar(255,0,0), 2);
        cv::KeyPoint kp;
        kp.pt = p;
        kps0.push_back(kp);
    }

    for (int i = 0; i < best_match_kp.second.size(); i ++) {
        cv::Point2f p (best_match_kp.second.at(i)(0), best_match_kp.second.at(i)(1));
        cv::circle(color_match, p, 2,cv::Scalar(0,255,0), 2);
        cv::KeyPoint kp;
        kp.pt = p;
        kps1.push_back(kp);
    }

    /// brute force match
    auto matches = simple_match(query_des.second, kps0, best_match_des.second, kps1, 0.7);
    std::cout << "matches: " << matches.size() << std::endl;

    std::vector<cv::Point2f> simple_matched_kps0, simple_matched_kps1;

    for (auto match: matches) {
        cv::DMatch dMatch = match.front();
        auto kp0 =  kps0.at(dMatch.queryIdx);
        auto kp1 =  kps1.at(dMatch.trainIdx);
        Eigen::Vector2d p0(kp0.pt.x, kp0.pt.y);
        Eigen::Vector3d P0;
        equidistantCamera.liftProjective(p0, P0);
        simple_matched_kps0.push_back(cv::Point2f(P0(0)/P0(2), P0(1)/P0(2)));

        Eigen::Vector2d p1(kp1.pt.x, kp1.pt.y);
        Eigen::Vector3d P1;
        equidistantCamera.liftProjective(p1, P1);
        simple_matched_kps1.push_back(cv::Point2f(P1(0)/P1(2), P1(1)/P1(2)));
    }

    /// Ransac
    std::vector<uchar> status;
    cv::findFundamentalMat(simple_matched_kps0, simple_matched_kps1,
            cv::FM_RANSAC, 3.0 / mu, 0.99, status);

    int before_match_cnt = matches.size();
    reduceVector(matches, status);

    int after_match_cnt = matches.size();
    std::cout << "Ransac: " << before_match_cnt << " " << after_match_cnt << std::endl;

    /// show
    cv::Mat outImage;
    cv::drawMatches(color_query,kps0,color_match,kps1,matches, outImage);

    cv::imshow("query_image", outImage);
//    cv::imshow("best_match_image", color_match);
    cv::waitKey();


    return 1;
}