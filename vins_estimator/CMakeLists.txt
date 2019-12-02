cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    rosbag
    )

find_package(OpenCV 3.4 REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

include_directories("include")

add_library(camera_model
        src/camera_model/chessboard/Chessboard.cc
        src/camera_model/calib/CameraCalibration.cc
        src/camera_model/camera_models/Camera.cc
        src/camera_model/camera_models/CameraFactory.cc
        src/camera_model/camera_models/CostFunctionFactory.cc
        src/camera_model/camera_models/PinholeCamera.cc
        src/camera_model/camera_models/CataCamera.cc
        src/camera_model/camera_models/EquidistantCamera.cc
        src/camera_model/camera_models/ScaramuzzaCamera.cc
        src/camera_model/camera_models/FovCamera.cc
        src/camera_model/sparse_graph/Transform.cc
        src/camera_model/gpl/gpl.cc
        src/camera_model/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)


add_library(${PROJECT_NAME}
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
        src/feature_track/feature_tracker.cpp
        src/feature_track/parameters.cpp
    )


target_link_libraries(${PROJECT_NAME} camera_model ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)


add_executable(vins_estimator_node src/estimator_node.cpp)
target_link_libraries(vins_estimator_node ${PROJECT_NAME} ${catkin_LIBRARIES}
        ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)

#add_executable(feature_tracker_node src/feature_tracker_node.cpp)
#target_link_libraries(feature_tracker_node camera_model ${PROJECT_NAME} ${catkin_LIBRARIES}
#        ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)


add_executable(data2bag tools/data2bag.cpp )
target_link_libraries(data2bag camera_model ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)

include_directories(src)
add_executable(vio_app app/vio_app.cpp app/vin_system.cpp)
target_link_libraries(vio_app camera_model ${PROJECT_NAME} ${catkin_LIBRARIES}
        ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)