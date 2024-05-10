//
// Created by wlxing on 21/9/2022.
//

#ifndef LIVOX_DVS_CALIB_EVENTS_VISUALIZER_HPP
#define LIVOX_DVS_CALIB_EVENTS_VISUALIZER_HPP

#include <pcl/visualization/pcl_visualizer.h>
//#include "dvs_utils.hpp"
#include <mutex>
#include <thread>
#include "string"


class EventStreamVis {
public:
    explicit EventStreamVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &update) {
        cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*update, *cloud_);
    }

    explicit EventStreamVis() {
        cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    };

    ~EventStreamVis() {
        current_thread_.join();
    }

    void update_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &update) {
        cloud_mutex_.lock();
        cloud_->clear();
        pcl::copyPointCloud(*update, *cloud_);
        cloud_mutex_.unlock();
    }

    void run() {
        current_thread_ = std::thread(&EventStreamVis::draw, this);
    }

    pcl::visualization::PCLVisualizer::Ptr rgbVis() {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud_-----
        // --------------------------------------------

        auto thread_id = std::this_thread::get_id();
        std::stringstream thread_id_ss;
        thread_id_ss << " ";
        thread_id_ss << thread_id;

        pcl::visualization::PCLVisualizer::Ptr viewer(
                new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(255, 255, 255);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud_, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                                 "sample cloud");
        viewer->addCoordinateSystem(40.0);
        viewer->initCameraParameters();
        viewer->setCameraPosition(173, 130, -300, 173, 130, 1, 0, -1, 0);
        return (viewer);
    }

    void draw() {
        pcl::visualization::PCLVisualizer::Ptr viewer;
        viewer = rgbVis();

        auto thread_id = std::this_thread::get_id();
        std::stringstream thread_id_ss;
        thread_id_ss << " ";
        thread_id_ss << thread_id;


        while (!viewer->wasStopped()) {
            wait_milliseconds(100);

            cloud_mutex_.lock();
            viewer->spinOnce(100);
            // do not wait it will slow visualizer
            if (cloud_) {
                if (!viewer->updatePointCloud(cloud_, "sample cloud")) {
                    viewer->addPointCloud(cloud_, "sample cloud");
                }
            }
            cloud_mutex_.unlock();
        }
    }


private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ = nullptr;
    std::mutex cloud_mutex_;
    std::thread current_thread_;
};

/**
 * visualize point cloud, this function can be used in a thread
 * @param cloud
 * @return return a viewer
 */
pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------

    auto thread_id = std::this_thread::get_id();
    std::stringstream thread_id_ss;
    thread_id_ss << " ";
    thread_id_ss << thread_id;

    pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                             "sample cloud");
    viewer->addCoordinateSystem(40.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(173, 130, -300, 173, 130, 1, 0, -1, 0);
    return (viewer);
}

/**
 * visualize point cloud, this function can be used in a thread
 * @param point_cloud_ptr point cloud
 * @param point_cloud_mutex the mutex of the point cloud
 */
void draw_color_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr, std::mutex &point_cloud_mutex) {
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = rgbVis(point_cloud_ptr);

    auto thread_id = std::this_thread::get_id();
    std::stringstream thread_id_ss;
    thread_id_ss << " ";
    thread_id_ss << thread_id;


    while (!viewer->wasStopped()) {
        wait_milliseconds(100);
        std::lock_guard<std::mutex> pc_update_lck(point_cloud_mutex);
        viewer->spinOnce(100);
        // do not wait it will slow visualizer
        if (point_cloud_ptr) {
            if (!viewer->updatePointCloud(point_cloud_ptr, "sample cloud")) {
                viewer->addPointCloud(point_cloud_ptr, "sample cloud");
            }
        }
    }
}

#endif //LIVOX_DVS_CALIB_EVENTS_VISUALIZER_HPP
