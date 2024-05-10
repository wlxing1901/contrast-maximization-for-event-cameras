//
// Created by wlxing on 21/9/2022.
//

#ifndef LIVOX_DVS_CALIB_DVS_UTILS_HPP
#define LIVOX_DVS_CALIB_DVS_UTILS_HPP

#include <thread>
#include <opencv2/opencv.hpp>
#include <string>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <cstdlib>
#include <chrono>

#include <dv_ros_msgs/EventArray.h>
#include <dv_ros_msgs/Event.h>
#include <dvs_cm.hpp>
/**
 * split string
 * @param s
 * @param tokens
 * @param delimiters
 */
void string_split(const std::string &s, std::vector<std::string> &tokens, const std::string &delimiters = " ") {
    std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos) {
        tokens.push_back(s.substr(lastPos, pos - lastPos));//use emplace_back after C++11
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}


/**
 * wait time
 * @param milliseconds
 */
void wait_milliseconds(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

/**
 * convert event stream to image
 * @param es
 * @param zoom_factor
 * @return
 */
cv::Mat event_stream_to_image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &es, int zoom_factor = 2,
                              bool non_maximum_suppression = false, int x_size = 346,
                              int y_size = 260,
                              int add_value = 20.0) {
    cv::Mat image = cv::Mat::zeros(y_size * zoom_factor, x_size * zoom_factor, CV_8UC3);
    for (auto &p: es->points) {
        int channel = (p.r > 0 ? 0 : 2);
        int u = p.x * zoom_factor;
        int v = p.y * zoom_factor;
        if (u < 0 * zoom_factor || v < 0 * zoom_factor ||
            u >= (x_size - 1) * zoom_factor || v >= (y_size - 1) * zoom_factor) {
            continue;
        }
        for (int delta_u = 0; delta_u < zoom_factor; delta_u++) {
            for (int delta_v = 0; delta_v < zoom_factor; delta_v++) {
                if (image.at<cv::Vec3b>(v + delta_v, u + delta_u)[channel] > 255 - add_value) {
                    image.at<cv::Vec3b>(v + delta_v, u + delta_u)[channel] = 255;
                    continue;
                }
                image.at<cv::Vec3b>(v + delta_v, u + delta_u)[channel] += add_value;
            }
        }
    }

    if (non_maximum_suppression) {
        for (int u = 0; u < x_size * zoom_factor; u++) {
            for (int v = 0; v < y_size * zoom_factor; v++) {
                for (int channel = 0; channel < 3; channel++) {
                    if (image.at<cv::Vec3b>(v, u)[channel] < 80) {
                        image.at<cv::Vec3b>(v, u)[channel] = 0;
                    }
                }
            }
        }
    }
    return image;
}

cv::Mat event_stream_to_image(dv_ros_msgs::EventArrayPtr &event_array, int zoom_factor = 2,
                              bool non_maximum_suppression = false, int x_size = 346,
                              int y_size = 260,
                              int add_value = 20.0) {
    cv::Mat image = cv::Mat::zeros(y_size * zoom_factor, x_size * zoom_factor, CV_8UC3);
    for (auto &event: event_array->events) {
        int channel = (event.polarity > 0 ? 0 : 2);
        int u = event.x * zoom_factor;
        int v = event.y * zoom_factor;
        if (u < 0 * zoom_factor || v < 0 * zoom_factor ||
            u >= (x_size - 1) * zoom_factor || v >= (y_size - 1) * zoom_factor) {
            continue;
        }
        for (int delta_u = 0; delta_u < zoom_factor; delta_u++) {
            for (int delta_v = 0; delta_v < zoom_factor; delta_v++) {
                if (image.at<cv::Vec3b>(v + delta_v, u + delta_u)[channel] > 255 - add_value) {
                    image.at<cv::Vec3b>(v + delta_v, u + delta_u)[channel] = 255;
                    continue;
                }
                image.at<cv::Vec3b>(v + delta_v, u + delta_u)[channel] += add_value;
            }
        }
    }

    if (non_maximum_suppression) {
        for (int u = 0; u < x_size * zoom_factor; u++) {
            for (int v = 0; v < y_size * zoom_factor; v++) {
                for (int channel = 0; channel < 3; channel++) {
                    if (image.at<cv::Vec3b>(v, u)[channel] < 80) {
                        image.at<cv::Vec3b>(v, u)[channel] = 0;
                    }
                }
            }
        }
    }
    return image;
}


/**
 * read events from pcd file
 * @param filename
 * @param es
 * @param bi_es
 */
void read_event_stream_from_pcd(const std::string &filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &es,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &bi_es) {
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB>(filename, *es);
    for (auto &ce: es->points) {
        pcl::PointXYZI e;
        e.x = ce.x;
        e.y = ce.y;
        e.z = ce.z;
        e.intensity = (ce.r > 5.0 ? 1.0 : 0.0);
        bi_es->push_back(e);
    }
}

/**
 * get timestamp
 * @return
 */
std::string get_timestamp() {
    const auto p1 = std::chrono::system_clock::now();

    return std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
            p1.time_since_epoch()).count());
}


class TicToc {
public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


#endif //LIVOX_DVS_CALIB_DVS_UTILS_HPP
