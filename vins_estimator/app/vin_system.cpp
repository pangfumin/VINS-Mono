//
// Created by pang on 2019/12/2.
//

#include "vin_system.h"
#include "vins_estimator/feature_track/parameters.h"
#include "vins_estimator/parameters.h"
#include "vins_estimator/feature_track/feature_tracker.h"
#include "vins_estimator/estimator.h"



VinSystem::VinSystem(const std::string config_file) {

    readParameters(config_file);
    feature_track::readParameters(config_file);
    estimator_ = std::make_shared<Estimator>();
    estimator_->setParameter();


    mbFinishRequested = false;
    mbFinished = true;

    mbFeatureTrackFinishRequested = false;
    mbFeatureTrackFinished = true;

    for (int i = 0; i < NUM_OF_CAM; i++) {
        trackerData_.push_back(std::make_shared<feature_track::FeatureTracker>());
        trackerData_[i]->readIntrinsicParameter(feature_track::CAM_NAMES[i]);
    }

    if(feature_track::FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData_[i]->fisheye_mask = cv::imread(feature_track::FISHEYE_MASK, 0);
            if(!trackerData_[i]->fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    measurement_process_thread_ = std::thread(&VinSystem::process, this);
    image_process_thread_ = std::thread(&VinSystem::processImageLoop, this);
}
VinSystem::~VinSystem() {
    shutdown();
    estimator_->resetState();
}


void VinSystem::feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

//void VinSystem::img_callback(const sensor_msgs::ImageConstPtr img_msg) {
//    m_image_mutex.lock();
//    m_image_buffer.push_back(*img_msg);
//    m_image_mutex.unlock();
//}
void VinSystem::processRawImage(const CameraMeasurement &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg.timeStamp.toSec();
        last_image_time = img_msg.timeStamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (img_msg.timeStamp.toSec() - last_image_time > 1.0 || img_msg.timeStamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg.timeStamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (img_msg.timeStamp.toSec() - first_image_time)) <= feature_track::FREQ)
    {
        feature_track::PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg.timeStamp.toSec() - first_image_time) - feature_track::FREQ) < 0.01 * feature_track::FREQ)
        {
            first_image_time = img_msg.timeStamp.toSec();
            pub_count = 0;
        }
    }
    else
        feature_track::PUB_THIS_FRAME = false;

//    cv_bridge::CvImageConstPtr ptr;
//    if (img_msg.encoding == "8UC1")
//    {
//        sensor_msgs::Image img;
//        img.header = img_msg.header;
//        img.height = img_msg.height;
//        img.width = img_msg.width;
//        img.is_bigendian = img_msg.is_bigendian;
//        img.step = img_msg.step;
//        img.data = img_msg.data;
//        img.encoding = "mono8";
//        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//    }
//    else
//        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = img_msg.measurement.image.clone();
    // feature_track::TicToc t_r;
    for (int i = 0; i < feature_track::NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !feature_track::STEREO_TRACK)
            trackerData_[i]->readImage(img_msg.measurement.image.rowRange(feature_track::ROW * i, feature_track::ROW * (i + 1)),
                                     img_msg.timeStamp.toSec());
        else
        {
            if (feature_track::EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(img_msg.measurement.image.rowRange(feature_track::ROW * i, feature_track::ROW * (i + 1)), trackerData_[i]->cur_img);
            }
            else
                trackerData_[i]->cur_img = img_msg.measurement.image.rowRange(feature_track::ROW * i, feature_track::ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData_[i]->showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < feature_track::NUM_OF_CAM; j++)
            if (j != 1 || !feature_track::STEREO_TRACK)
                completed |= trackerData_[j]->updateID(i);
        if (!completed)
            break;
    }

    if (feature_track::PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;


//        feature_points->header = img_msg.header;
        feature_points->header.frame_id = "world";
        feature_points->header.stamp = img_msg.timeStamp;

        vector<set<int>> hash_ids(feature_track::NUM_OF_CAM);
        for (int i = 0; i < feature_track::NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData_[i]->cur_un_pts;
            auto &cur_pts = trackerData_[i]->cur_pts;
            auto &ids = trackerData_[i]->ids;
            auto &pts_velocity = trackerData_[i]->pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData_[i]->track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * feature_track::NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else {
//            pub_img.publish(feature_points);
            feature_callback(feature_points);
        }


        if (feature_track::SHOW_TRACK)
        {
            //ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            //cv::Mat stereo_img = img_msg.measurement.image.clone();
            cv::cvtColor(img_msg.measurement.image, stereo_img, CV_GRAY2RGB);

            for (int i = 0; i < feature_track::NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * feature_track::ROW, (i + 1) * feature_track::ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData_[i]->cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData_[i]->track_cnt[j] / feature_track::WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData_[i]->cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData_[i]->cur_un_pts[j].x, trackerData_[i]->cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData_[i]->pts_velocity[j].x, trackerData_[i]->pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData_[i]->m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData_[i]->cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData_[i]->ids[j]);
                    //cv::putText(tmp_img, name, trackerData_[i]->cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = img_msg.timeStamp;// Same timestamp and tf frame as input image
            out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
            out_msg.image    = stereo_img; // Your cv::Mat
            pub_match.publish(out_msg.toImageMsg());
        }
    }
    // ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}


void VinSystem::predict(const ImuMeasurement &imu_msg)
{
    double t = imu_msg.timeStamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    Eigen::Vector3d linear_acceleration = imu_msg.measurement.accelerometers;
    Eigen::Vector3d angular_velocity = imu_msg.measurement.gyroscopes;

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator_->g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator_->g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void VinSystem::update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator_->Ps[WINDOW_SIZE];
    tmp_Q = estimator_->Rs[WINDOW_SIZE];
    tmp_V = estimator_->Vs[WINDOW_SIZE];
    tmp_Ba = estimator_->Bas[WINDOW_SIZE];
    tmp_Bg = estimator_->Bgs[WINDOW_SIZE];
    acc_0 = estimator_->acc_0;
    gyr_0 = estimator_->gyr_0;

    queue<ImuMeasurement> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<ImuMeasurement>, sensor_msgs::PointCloudConstPtr>>
VinSystem::getMeasurements()
{
    std::vector<std::pair<std::vector<ImuMeasurement>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back().timeStamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator_->td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front().timeStamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator_->td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<ImuMeasurement> IMUs;
        while (imu_buf.front().timeStamp.toSec() < img_msg->header.stamp.toSec() + estimator_->td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}
//
//void VinSystem::imu_callback(const sensor_msgs::ImuConstPtr imu_msg)
//{
//    if (imu_msg->header.stamp.toSec() <= last_imu_t)
//    {
//        ROS_WARN("imu message in disorder!");
//        return;
//    }
//    last_imu_t = imu_msg->header.stamp.toSec();
//
//    m_buf.lock();
//    imu_buf.push(imu_msg);
//    m_buf.unlock();
//    con.notify_one();
//
//    last_imu_t = imu_msg->header.stamp.toSec();
//
//    {
//        std::lock_guard<std::mutex> lg(m_state);
//        predict(imu_msg);
//        std_msgs::Header header = imu_msg->header;
//        header.frame_id = "world";
//        if (estimator_->solver_flag == Estimator::SolverFlag::NON_LINEAR)
//            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
//    }
//}

bool VinSystem::addImage(const ros::Time & stamp, size_t cameraIndex,
                      const cv::Mat & image,
                      const std::vector<cv::KeyPoint> * keypoints,
                      bool* asKeyframe) {
    CameraMeasurement cam_meas;
    cam_meas.timeStamp = stamp;
    cam_meas.measurement.image = image;
    m_image_mutex.lock();
    m_image_buffer.push_back(cam_meas);
    m_image_mutex.unlock();

    return true;
}

bool VinSystem::addImuMeasurement(const ros::Time & stamp,
                               const Eigen::Vector3d & alpha,
                               const Eigen::Vector3d & omega)  {
    if (stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return false;
    }
    last_imu_t = stamp.toSec();

    ImuMeasurement imu_msg;
    imu_msg.timeStamp = stamp;
    imu_msg.measurement.accelerometers = alpha;
    imu_msg.measurement.gyroscopes = omega;
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = "world";
        if (estimator_->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
    return true;
}





void VinSystem::restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator_->clearState();
        estimator_->setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}



// thread: visual-inertial odometry
void VinSystem::process() {
    mbFinished = false;
    while (!isFinishRequested()) {
        std::vector<std::pair<std::vector<ImuMeasurement>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&] {
            return ((measurements = getMeasurements()).size() != 0 || isFinishRequested());
        });
        lk.unlock();

        {
            if (isFinishRequested()) {
//                ALOGI("NinebotSlam: [shutdown] CheckFinish  break measurement process loop");
                break;
            }
        }

        m_estimator.lock();
        for (auto &measurement : measurements) {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first) {
                double t = imu_msg.timeStamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator_->td;
                if (t <= img_t) {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;

                    estimator_->processIMU(dt, imu_msg.measurement.accelerometers, imu_msg.measurement.gyroscopes);
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                } else {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);

                    estimator_->processIMU(dt_1, imu_msg.measurement.accelerometers, imu_msg.measurement.gyroscopes);
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++) {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            estimator_->processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(*estimator_, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(*estimator_, header);
            pubKeyPoses(*estimator_, header);
            pubCameraPose(*estimator_, header);
            pubPointCloud(*estimator_, header);
            pubTF(*estimator_, header);
            pubKeyframe(*estimator_);

        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator_->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
    SetFinish();
    return;

}

void VinSystem::processImageLoop() {
    pthread_setname_np(pthread_self(), "VSLAM_Vins2");
    mbFeatureTrackFinished = false;
    int cnt =0;
    while(!isFeatureTrackFinishRequested()) {
        bool hasNewImage = false;

        m_image_mutex.lock();
        int  pop_cnt = 0;

        CameraMeasurement im;
        if (m_image_buffer.size() > 0) {
            //ALOGI("NinebotSlam: m_image_buffer size : %d", m_image_buffer.size());
            im = m_image_buffer.front();
            m_image_buffer.pop_front();
            hasNewImage = true;
        }
        m_image_mutex.unlock();

        if (hasNewImage) {
            processRawImage(im);

            if (cnt ++ > 50 ) {
#if defined(ANDROID) || defined(__ANDROID__)
                ALOGI("NinebotSlam: vio klt thread id: %d.", gettid());
#else
               // ALOGI("NinebotSlam: vio klt thread id: %d.", syscall(SYS_gettid));
#endif

                cnt = 0;
            }

        } else {
            usleep(5*1000);   // Waiting
        }

    }

    SetFeatureTrackFinish();
    return;
}



void VinSystem::shutdown(){
    bool joinFromThisRequest = false; // For avoiding duplicate join.
    while(!isFinished()){
        joinFromThisRequest = true;
        RequestFinish();
        usleep(1000);
    }
    if(joinFromThisRequest)
        measurement_process_thread_.join();

    bool joinFeatureTrackFromThisRequest = false; // For avoiding duplicate join.
    while(!isFeatureTrackFinished()){
        joinFeatureTrackFromThisRequest = true;
        RequestFeatureTrackFinish();
        usleep(1000);
    }
    if(joinFeatureTrackFromThisRequest)
        image_process_thread_.join();
//        ALOGI("NinebotSlam: [shutdown] shutdown feature tracking");
}

//void VinSystem::release(){
//    m_isVinsInitialized = false;
//}

void VinSystem::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
    con.notify_all();
//        ALOGI("NinebotSlam: [shutdown] RequestFinish");
}

bool VinSystem::isFinishRequested()
{
    unique_lock<mutex> lock(mMutexFinish);
//            ALOGI("NinebotSlam: [shutdown] isFinishRequested: %s",
//                  mbFinishRequested ?  "true" : "false");

    return mbFinishRequested;
}

void VinSystem::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
//        ALOGI("NinebotSlam: [shutdown] SetFinish");
}

bool VinSystem::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
//        ALOGI("NinebotSlam: [shutdown] isFinished: %s",
//              mbFinished ?  "true" : "false");

    return mbFinished;
}

void VinSystem::RequestFeatureTrackFinish()
{
    unique_lock<mutex> lock(mFeatureTrackMutexFinish);
    mbFeatureTrackFinishRequested = true;
//        ALOGI("NinebotSlam: [shutdown] RequestFeatureTrackFinish");

}

bool VinSystem::isFeatureTrackFinishRequested()
{
    unique_lock<mutex> lock(mFeatureTrackMutexFinish);
//            ALOGI("NinebotSlam: [shutdown] isFeatureTrackFinishRequested: %s",
//                  mbFeatureTrackFinishRequested ?  "true" : "false");
    return mbFeatureTrackFinishRequested;
}

void VinSystem::SetFeatureTrackFinish()
{
    unique_lock<mutex> lock(mFeatureTrackMutexFinish);
    mbFeatureTrackFinished = true;
//        ALOGI("NinebotSlam: [shutdown] SetFeatureTrackFinish");
}

bool VinSystem::isFeatureTrackFinished()
{
    unique_lock<mutex> lock(mFeatureTrackMutexFinish);
//        ALOGI("NinebotSlam: [shutdown] isFeatureTrackFinished: %s",
//              mbFeatureTrackFinished ? "true" : "false");
    return mbFeatureTrackFinished;
}


