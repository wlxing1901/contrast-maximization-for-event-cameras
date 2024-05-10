# include all cmake files in this directory
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Ceres-solver
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# tbb
find_package(TBB REQUIRED)

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ros
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        pcl_ros
        pcl_conversions
        dv_ros_msgs REQUIRED
)
include_directories(${catkin_INCLUDE_DIRS})

# ros msg header
include_directories(${CMAKE_BINARY_DIR}/devel/include)

# python
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
include_directories(${PYTHON_INCLUDE_DIRS})

# set the third party libraries
set(third_party_libs
        ${catkin_LIBRARIES}
        #        ${g2o_libs}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        #        glog gflags
        ${yaml-cpp_LIBRARIES}
        yaml-cpp
        TBB::tbb
        ${Python3_LIBRARIES}
)
