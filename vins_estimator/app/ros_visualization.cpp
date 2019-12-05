//
// Created by pang on 2019/12/5.
//

#include "ros_visualization.h"

RosVisualization::RosVisualization(ros::NodeHandle &n):
        cameraposevisual_(0, 1, 0, 1),
        keyframebasevisual_(0.0, 0.0, 1.0, 1.0){
//    pub_latest_odometry_ = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path_ = n.advertise<nav_msgs::Path>("path", 1000);
//    pub_relo_path_ = n.advertise<nav_msgs::Path>("relocalization_path", 1000);
    pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1000);
//    pub_point_cloud_ = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
//    pub_margin_cloud_ = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
//    pub_key_poses_ = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose_ = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual_ = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
//    pub_keyframe_pose_ = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
//    pub_keyframe_point_ = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic_ = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
//    pub_relo_relative_pose_ =  n.advertise<nav_msgs::Odometry>("relo_relative_pose", 1000);

    cameraposevisual_.setScale(1);
    cameraposevisual_.setLineWidth(0.05);
    keyframebasevisual_.setScale(0.1);
    keyframebasevisual_.setLineWidth(0.01);
}

void RosVisualization::publishFullStateExtrinsicAsCallback(
        const std::vector<ros::Time> & ts_vec,
        const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> & T_WS_vec,
        const std::vector<Eigen::Matrix<double, 9, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 9, 1>>> & sb_vec,
const std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d> >& extrinsic_vec) {
    std::cout << "fullStateWithExtrinsicsCallback: " << ts_vec.size() << std::endl;
    std_msgs::Header header, header1;
    header.stamp = ts_vec[ts_vec.size() - 1];
    header1.stamp = ts_vec.back();


    pubCameraPose(T_WS_vec[T_WS_vec.size() - 1], extrinsic_vec[0], header);
    pubOdometry(T_WS_vec.back(), sb_vec.back().head<3>(), header1);
    pubTF(T_WS_vec.back(), extrinsic_vec.back(), header1);

}


void RosVisualization::pubCameraPose(
        const Eigen::Isometry3d &T_WS,
        const Eigen::Isometry3d T_IC,
        const std_msgs::Header &header)
{

    Eigen::Isometry3d T_WC = T_WS * T_IC;
    Eigen::Vector3d P = T_WC.translation();
    Eigen::Matrix3d R = T_WC.matrix().topLeftCorner(3,3);
    Eigen::Quaterniond q(R);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();

        pub_camera_pose_.publish(odometry);

        cameraposevisual_.reset();
        cameraposevisual_.add_pose(P, q);
        cameraposevisual_.publish_by(pub_camera_pose_visual_, odometry.header);

}


void RosVisualization::pubOdometry(const Eigen::Isometry3d T_WS, const Eigen::Vector3d v, const std_msgs::Header &header)
{

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Eigen::Quaterniond tmp_Q;
        Eigen::Matrix3d R_WS = T_WS.matrix().topLeftCorner(3,3);
        Eigen::Vector3d t_WS = T_WS.translation();

        tmp_Q = Eigen::Quaterniond(R_WS);
        odometry.pose.pose.position.x = t_WS.x();
        odometry.pose.pose.position.y = t_WS.y();
        odometry.pose.pose.position.z = t_WS.z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = v.x();
        odometry.twist.twist.linear.y = v.y();
        odometry.twist.twist.linear.z = v.z();
        pub_odometry_.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path_.header = header;
        path_.header.frame_id = "world";
        path_.poses.push_back(pose_stamped);
        pub_path_.publish(path_);

//        Vector3d correct_t;
//        Vector3d correct_v;
//        Quaterniond correct_q;
//        correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] + estimator.drift_correct_t;
//        correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
//        odometry.pose.pose.position.x = correct_t.x();
//        odometry.pose.pose.position.y = correct_t.y();
//        odometry.pose.pose.position.z = correct_t.z();
//        odometry.pose.pose.orientation.x = correct_q.x();
//        odometry.pose.pose.orientation.y = correct_q.y();
//        odometry.pose.pose.orientation.z = correct_q.z();
//        odometry.pose.pose.orientation.w = correct_q.w();
//
//        pose_stamped.pose = odometry.pose.pose;
//        relo_path.header = header;
//        relo_path.header.frame_id = "world";
//        relo_path.poses.push_back(pose_stamped);
//        pub_relo_path.publish(relo_path);
}


void RosVisualization::pubTF(const Eigen::Isometry3d &T_WS,
                             const Eigen::Isometry3d T_IC,
                             const std_msgs::Header &header)
{

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Eigen::Vector3d correct_t = T_WS.translation();
    Eigen::Matrix3d  correct_R = T_WS.matrix().topLeftCorner(3,3);
    Eigen::Quaterniond correct_q(correct_R);

    Eigen::Vector3d t_IC = T_IC.translation();
    Eigen::Matrix3d  R_IC = T_IC.matrix().topLeftCorner(3,3);
    Eigen::Quaterniond q_IC(R_IC);



    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(t_IC.x(),
                                    t_IC.y(),
                                    t_IC.z()));
    q.setW(q_IC.w());
    q.setX(q_IC.x());
    q.setY(q_IC.y());
    q.setZ(q_IC.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = t_IC.x();
    odometry.pose.pose.position.y = t_IC.y();
    odometry.pose.pose.position.z = t_IC.z();
//    Eigen::Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = q_IC.x();
    odometry.pose.pose.orientation.y = q_IC.y();
    odometry.pose.pose.orientation.z = q_IC.z();
    odometry.pose.pose.orientation.w = q_IC.w();
    pub_extrinsic_.publish(odometry);

}



