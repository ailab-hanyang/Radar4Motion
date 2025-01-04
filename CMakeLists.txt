cmake_minimum_required(VERSION 3.2)
project(radar4motion)

add_compile_options(-std=c++17)

set(CATKIN_PACKAGE_DEPENDENCIES
    # open3d_catkin
)

find_package(catkin REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    CATKIN_DEPENDS
)

option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" OFF)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" OFF)
option(USE_SYSTEM_TSL-ROBIN-MAP "Use system pre-installed tsl_robin" OFF)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" OFF)
include(3rdparty/find_dependencies.cmake)

include_directories(
    include
    include/radar4motion
    include/radar4motion/ini_handler_cpp
    include/radar4motion/kiss-icp
    ${catkin_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
    # main
    src/main.cpp
    # radar4motion
    src/core.cpp
    src/radar4motion/odometry.cpp
    src/radar4motion/preprocessing.cpp
    # kiss-icp
    include/radar4motion/kiss-icp/Preprocessing.cpp
    include/radar4motion/kiss-icp/Registration.cpp
    include/radar4motion/kiss-icp/VoxelHashMap.cpp
    include/radar4motion/kiss-icp/Threshold.cpp
    include/radar4motion/kiss-icp/VoxelUtils.cpp
    include/radar4motion/kiss-icp/Deskew.cpp
    # radar ego motion estimation
    include/ego_motion_estimator/ego_motion_estimator.cpp
    include/ego_motion_estimator/radar_point_cloud.cpp
    # Vod parse utils
    include/vod_utils/vod_parse_utils.cpp
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}

    Eigen3::Eigen
    tsl::robin_map
    Sophus::Sophus
    TBB::tbb
    ${OpenMP_CXX_LIBRARIES}
)