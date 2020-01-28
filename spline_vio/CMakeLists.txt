cmake_minimum_required(VERSION 2.8.3)
project(spline_vio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PROTO_DEFNS proto/spline_vio/spline.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})


#############
# LIBRARIES #
#############

SET(SPLINE_VIO_SOURCE
        src/camodocal/chessboard/Chessboard.cc
        src/camodocal/calib/CameraCalibration.cc
        src/camodocal/camera_models/Camera.cc
        src/camodocal/camera_models/CameraFactory.cc
        src/camodocal/camera_models/CostFunctionFactory.cc
        src/camodocal/camera_models/PinholeCamera.cc
        src/camodocal/camera_models/CataCamera.cc
        src/camodocal/camera_models/EquidistantCamera.cc
        src/camodocal/camera_models/ScaramuzzaCamera.cc
        src/camodocal/camera_models/FovCamera.cc
        src/camodocal/sparse_graph/Transform.cc
        src/camodocal/gpl/gpl.cc
        src/camodocal/gpl/EigenQuaternionParameterization.cc

        src/parameters.cpp

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

        src/spline/PoseSplineUtility.cpp
        src/spline/PoseSpline.cpp
        src/spline/PoseSplineSampleError.cpp
        src/spline/QuaternionSplineUtility.cpp
        src/spline/spline_projection_error4.cpp
        src/spline/spline_projection_error5.cpp
        src/spline/spline_projection_error6.cpp
        src/spline/spline_projection_error7.cpp
        src/spline/spline_projection_error8.cpp
        )

cs_add_library(${PROJECT_NAME}_lib ${SPLINE_VIO_SOURCE} ${PROTO_SRCS})

#######
# APP #
#######
cs_add_executable(vio_app
        app/vio_app.cpp
        app/vio_interface.cpp
        app/vin_system.cpp
        app/estimator.cpp
        )
target_link_libraries(vio_app ${PROJECT_NAME}_lib)

cs_add_executable(data2bag tools/data2bag.cpp)
target_link_libraries(data2bag ${PROJECT_NAME}_lib)

cs_add_executable(generate_mask tools/generate_mask.cpp)
target_link_libraries(generate_mask ${PROJECT_NAME}_lib)

#cs_add_executable(test_spline_initialization test/test_spline_initialization.cpp)
#target_link_libraries(test_spline_initialization ${PROJECT_NAME}_lib)
#########
# SHARE #
#########
#catkin_add_env_hooks(10.rovio-config-template
#        SHELLS sh
#        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
#        )



############
## EXPORT ##
############
cs_install()
cs_export()
