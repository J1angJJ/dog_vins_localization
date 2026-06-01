cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_models
    cv_bridge
    roslib
    )

option(DOG_VINS_PREFER_ROS_OPENCV "Prefer ROS/system OpenCV over /usr/local OpenCV" ON)
if(DOG_VINS_PREFER_ROS_OPENCV)
    set(_ros_opencv_dir "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4")
    if(EXISTS "${_ros_opencv_dir}/OpenCVConfig.cmake")
        set(OpenCV_DIR "${_ros_opencv_dir}" CACHE PATH "OpenCV config directory" FORCE)
    endif()
endif()
find_package(OpenCV REQUIRED)
message(STATUS "loop_fusion using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")


find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )

target_link_libraries(loop_fusion_node ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
target_compile_options(loop_fusion_node PRIVATE -include ${CMAKE_SOURCE_DIR}/opencv4_compat.hpp)
