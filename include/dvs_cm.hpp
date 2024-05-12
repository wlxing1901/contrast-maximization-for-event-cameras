//
// Created by wlxing on 9/29/22.
//

#ifndef LIVOX_DVS_CALIB_DVS_CM_HPP
#define LIVOX_DVS_CALIB_DVS_CM_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <execution>
#include <thread>
#include <opencv2/opencv.hpp>
#include <string>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <cstdlib>
#include <chrono>

#include <dv_ros_msgs/EventArray.h>
#include <dv_ros_msgs/Event.h>

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

/**
 * wait time
 * @param milliseconds
 */
void wait_milliseconds(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

namespace fx {
    template<typename T>
    class Event {
    public:
        Event(T x, T y, ros::Time ts, bool polarity) : x(x), y(y), ts(ts), polarity(polarity) {}

        T x;
        T y;
        ros::Time ts;
        bool polarity;
    };

    template<typename T>
    class EventArray {
    public:
        EventArray() = default;

        void emplace_back(const Event<T> &event) {
            events.emplace_back(event);
        }

        void push_back(const Event<T> &event) {
            events.push_back(event);
        }

        void copy_from(const EventArray<T> &event_array) {
            events = event_array.events;
        }

        void copy_from(const std::shared_ptr<EventArray<T>> &event_array) {
            events = event_array->events;
        }

        void copy_from(const dv_ros_msgs::EventArrayPtr &event_array) {
            events.clear();
            for (auto &e: event_array->events) {
                Event<T> temp(e.x, e.y, e.ts, e.polarity);
                events.emplace_back(temp);
            }
        }

        std::vector<Event<T>> events;
    };

    using EventType = Event<double>;
    using EventArrayType = EventArray<double>;
    using EventArrayPtr = std::shared_ptr<EventArray<double>>;
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

cv::Mat event_stream_to_image(fx::EventArrayPtr &event_array, int zoom_factor = 2,
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
 * compute 2d gauss function
 * @param x
 * @param y
 * @param x0 center x
 * @param y0 center y
 * @param sigma
 * @return
 */
inline double gauss_func(double x, double y, double x0, double y0, double sigma = 1.0) {
    double res = 0.0;
    res = 1.0 / (2 * M_PI * sigma * sigma) *
          exp(-1.0 * ((x - x0) * (x - x0) + (y - y0) * (y - y0)) / (2.0 * sigma * sigma));
    return res;
}

/**
 * covert event stream from image plane to uv coordinate
 * @param ip
 * @param uv
 * @param K
 */
inline void event_stream_image_plane_to_uv(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ip,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &uv,
                                           const std::vector<std::vector<double>> &K) {
    uv->clear();
    double fx = K[0][0];
    double cx = K[0][2];
    double fy = K[1][1];
    double cy = K[1][2];

    for (auto &e: ip->points) {
        pcl::PointXYZRGB temp;
        temp.x = fx * e.x + cx;
        temp.y = fy * e.y + cy;
        temp.z = e.z;
        temp.rgb = e.rgb;
        uv->push_back(temp);
    }
}

inline void event_stream_image_plane_to_uv(dv_ros_msgs::EventArrayPtr &ip,
                                           dv_ros_msgs::EventArrayPtr &uv,
                                           const std::vector<std::vector<double>> &K) {
    uv->events.clear();
    double fx = K[0][0];
    double cx = K[0][2];
    double fy = K[1][1];
    double cy = K[1][2];

    for (auto &e: ip->events) {
        dv_ros_msgs::Event temp;
        temp.x = fx * e.x + cx;
        temp.y = fy * e.y + cy;
        temp.ts = e.ts;
        temp.polarity = e.polarity;
        uv->events.push_back(temp);
    }
}

inline void event_stream_image_plane_to_uv(fx::EventArrayPtr &ip,
                                           fx::EventArrayPtr &uv,
                                           const std::vector<std::vector<double>> &K) {
    uv->events.clear();
    double fx = K[0][0];
    double cx = K[0][2];
    double fy = K[1][1];
    double cy = K[1][2];

    for (auto &e: ip->events) {
        fx::Event<double> temp(fx * e.x + cx, fy * e.y + cy, e.ts, e.polarity);
        uv->events.push_back(temp);
    }
}


/**
 * convert event stream from uv to image plane coordinate
 * @param uv
 * @param ip
 * @param K
 */
inline void
event_stream_uv_to_image_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &uv, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ip,
                               std::vector<std::vector<double>> &K) {
    ip->clear();
    double fx = K[0][0];
    double cx = K[0][2];
    double fy = K[1][1];
    double cy = K[1][2];

    for (auto &e: uv->points) {
        pcl::PointXYZRGB temp;
        temp.x = (e.x - cx) / fx;
        temp.y = (e.y - cy) / fy;
        temp.z = e.z;
        temp.rgb = e.rgb;
        ip->push_back(temp);
    }
}

inline void
event_stream_uv_to_image_plane(dv_ros_msgs::EventArrayPtr &uv, dv_ros_msgs::EventArrayPtr &ip,
                               std::vector<std::vector<double>> &K) {
    ip->events.clear();
    double fx = K[0][0];
    double cx = K[0][2];
    double fy = K[1][1];
    double cy = K[1][2];

    for (auto &e: uv->events) {
        dv_ros_msgs::Event temp;
        temp.x = (e.x - cx) / fx;
        temp.y = (e.y - cy) / fy;
        temp.ts = e.ts;
        temp.polarity = e.polarity;
        ip->events.push_back(temp);

    }
}

inline void
event_stream_uv_to_image_plane(fx::EventArrayPtr &uv, fx::EventArrayPtr &ip,
                               std::vector<std::vector<double>> &K) {
    ip->events.clear();
    double fx = K[0][0];
    double cx = K[0][2];
    double fy = K[1][1];
    double cy = K[1][2];

    for (auto &e: uv->events) {
        fx::Event<double> temp((e.x - cx) / fx, (e.y - cy) / fy, e.ts, e.polarity);
        ip->events.push_back(temp);
    }
}


/**
 * compute contrast value
 * @param es event stream in uv coordinate
 * @return
 */
inline double compute_contrast(pcl::PointCloud<pcl::PointXYZRGB>::Ptr es) {
    double mu = double(es->points.size()) / (346.0 * 260.0);
    double var = 0.0;
    double warped_image[260][346] = {0};
    std::mutex mutexes[260][346] = {std::mutex()};

    std::for_each(std::execution::par_unseq, es->points.begin(), es->points.end(), [&](pcl::PointXYZRGB &p) {
        double temp_x = p.x;
        double temp_y = p.y;
        double temp_polarity = (p.r - p.b) / 255.0;
//        double region_size = 8.0;
        double region_size = 4.0;
        for (int i = std::max(temp_x - region_size, 0.0); i <= std::min(259.0, temp_x + region_size); i++) {
            for (int j = std::max(temp_y - region_size, 0.0); j <= std::min(345.0, temp_y + region_size); j++) {
                std::lock_guard<std::mutex> lock(mutexes[i][j]);
                warped_image[i][j] += gauss_func(i, j, temp_x, temp_y, 1.0);
            }
        }
    });


    for (auto &i: warped_image) {
        for (double j: i) {
            var += j * j;
        }
    }
    var /= 346.0 * 260.0;
    var = 0.0005 * sqrt(1.0 / var);
    return var;
}

inline double compute_contrast(dv_ros_msgs::EventArrayPtr &es) {
    double mu = double(es->events.size()) / (346.0 * 260.0);
    double var = 0.0;
    double warped_image[260][346] = {0};
    std::mutex mutexes[260][346] = {std::mutex()};

    std::for_each(std::execution::par_unseq, es->events.begin(), es->events.end(), [&](dv_ros_msgs::Event &e) {
        double temp_x = e.x;
        double temp_y = e.y;
        double temp_polarity = e.polarity;
//        double region_size = 8.0;
        double region_size = 4.0;
        for (int i = std::max(temp_x - region_size, 0.0); i <= std::min(259.0, temp_x + region_size); i++) {
            for (int j = std::max(temp_y - region_size, 0.0); j <= std::min(345.0, temp_y + region_size); j++) {
                std::lock_guard<std::mutex> lock(mutexes[i][j]);
                warped_image[i][j] += gauss_func(i, j, temp_x, temp_y, 1.0);
            }
        }
    });

    for (auto &i: warped_image) {
        for (double j: i) {
            var += j * j;
        }
    }

    var /= 346.0 * 260.0;
    var = 0.0005 * sqrt(1.0 / var);
    return var;
}


inline void
warp_event_stream(const dv_ros_msgs::EventArrayPtr &ip_in, dv_ros_msgs::EventArrayPtr &ip_out,
                  const double *const omega) {
    ip_out->events.clear();
    for (auto &e: ip_in->events) {
        double delta_t = -e.ts.toSec();
        double p_before[3], p_after[3];
        p_before[0] = e.x;
        p_before[1] = e.y;
        p_before[2] = 1.0;
        double delta_omega[3] = {omega[0] * delta_t,
                                 omega[1] * delta_t,
                                 omega[2] * delta_t};
        ceres::AngleAxisRotatePoint(delta_omega, p_before, p_after);
        double temp_x = p_after[0] / p_after[2];
        double temp_y = p_after[1] / p_after[2];
        dv_ros_msgs::Event temp;
        temp.x = temp_x;
        temp.y = temp_y;
        temp.ts = e.ts;
        temp.polarity = e.polarity;
        ip_out->events.push_back(temp);
    }
}


inline void
warp_event_stream(const fx::EventArrayPtr &ip_in, fx::EventArrayPtr &ip_out,
                  const double *const omega) {
    ip_out->events.clear();
    ros::Time start_time = ip_in->events.front().ts;
    for (auto &e: ip_in->events) {
        double delta_t = start_time.toSec() - e.ts.toSec();
        double p_before[3], p_after[3];
        p_before[0] = e.x;
        p_before[1] = e.y;
        p_before[2] = 1.0;
        double delta_omega[3] = {omega[0] * delta_t,
                                 omega[1] * delta_t,
                                 omega[2] * delta_t};
        ceres::AngleAxisRotatePoint(delta_omega, p_before, p_after);
        double temp_x = p_after[0] / p_after[2];
        double temp_y = p_after[1] / p_after[2];
        fx::Event<double> temp(temp_x, temp_y, e.ts, e.polarity);
        ip_out->events.push_back(temp);
    }
}


/**
 * Functor to find optimal rotation angular velocity omega
 */
class EventCameraRotation3D {
public:
    bool operator()(const double *const omega, double *const res) const {
        fx::EventArrayPtr event_array_warped = std::make_shared<fx::EventArray<double>>();
        fx::EventArrayPtr event_array_warped_ip = std::make_shared<fx::EventArray<double>>();
        warp_event_stream(event_array_ip_, event_array_warped_ip, omega);
        event_stream_image_plane_to_uv(event_array_warped_ip, event_array_warped, K_);
        res[0] = this->compute_contrast(event_array_warped);
        return true;
    }

    explicit EventCameraRotation3D(fx::EventArrayPtr &event_array,
                                   std::vector<std::vector<double>> &K) : K_(K) {
        int event_num = event_array->events.size();
        double filter_ratio = (double) event_num / (double) event_num_threshold_;

        if (filter_ratio > 1.0) {
            double filter_cnt = filter_ratio;
            event_array_->events.clear();
            for (int i = 0; i < event_num; i++) {
                if (i >= filter_cnt) {
                    filter_cnt += filter_ratio;
                    event_array_->events.push_back(event_array->events[i]);
                }
            }
        } else {
            event_array_->copy_from(event_array);
        }

        LOG(INFO) << "optimization use event number: " << event_array_->events.size() << std::endl;

        event_stream_uv_to_image_plane(event_array_, event_array_ip_, K);


        double sigma = 1.0;
        for (int i = 0; i < gauss_func_table_size_; i++) {
            double res = 0.0;
            res = 1.0 / (2 * M_PI * sigma * sigma) *
                  exp(-1.0 * (double(i) / 10) / (2.0 * sigma * sigma));
            gauss_func_table_.push_back(res);
        }
    }

    void GetEventArrayWarped(fx::EventArrayPtr &event_array_warped,
                             const double *const omega) {
        fx::EventArrayPtr event_array_warped_ip = std::make_shared<fx::EventArray<double>>();
        warp_event_stream(event_array_ip_, event_array_warped_ip, omega);
        event_stream_image_plane_to_uv(event_array_warped_ip, event_array_warped, K_);
    }

private:
    fx::EventArrayPtr event_array_ = std::make_shared<fx::EventArray<double>>();
    fx::EventArrayPtr event_array_ip_ = std::make_shared<fx::EventArray<double>>();
    std::vector<std::vector<double>> K_;

    int event_num_threshold_ = 12000;

    std::vector<double> gauss_func_table_;
    int gauss_func_table_size_ = 200;

    [[nodiscard]] inline double
    find_gauss_func_table(double x, double y, double x0, double y0, double sigma = 1.0) const {
        double res = 0.0;
        double index = (x - x0) * (x - x0) + (y - y0) * (y - y0);
        index *= 10.0;
        if (index < gauss_func_table_size_) {
            // linear interpolation with two side values
            res = gauss_func_table_[int(index)] +
                  (gauss_func_table_[int(index) + 1] - gauss_func_table_[index]) * (index - (int) index);
        } else {
            res = 0.0;
        }
        return res;
    }


    inline double compute_contrast(fx::EventArrayPtr &es) const {
        double mu = double(es->events.size()) / (346.0 * 260.0);
        double var = 0.0;
        double warped_image[260][346] = {0};
        std::mutex mutexes[260][346] = {std::mutex()};

        std::for_each(std::execution::par_unseq, es->events.begin(), es->events.end(), [&](fx::Event<double> &e) {
            double temp_x = e.x;
            double temp_y = e.y;
            double temp_polarity = e.polarity;
//        double region_size = 8.0;
            double region_size = 4.0;
            for (int i = std::max(temp_x - region_size, 0.0); i <= std::min(259.0, temp_x + region_size); i++) {
                for (int j = std::max(temp_y - region_size, 0.0); j <= std::min(345.0, temp_y + region_size); j++) {
                    std::lock_guard<std::mutex> lock(mutexes[i][j]);
                    warped_image[i][j] += gauss_func(i, j, temp_x, temp_y, 1.0);
//                    warped_image[i][j] += find_gauss_func_table(i, j, temp_x, temp_y, 1.0);
                }
            }
        });

        for (auto &i: warped_image) {
            for (double j: i) {
                var += j * j;
            }
        }

        var /= 346.0 * 260.0;
        var = 0.0005 * sqrt(1.0 / var);
        return var;
    }

};

struct NumericContrastMaximizationTranslationPixel2D {
public:
    bool operator()(const double *const v, double *const res) const {
//        double mu = double(ip_->points.size()) / (x_size_ * y_size_);
        double var = 0.0;
        double warped_image[260][346] = {0};
        for (auto &p: pc_->points) {
            double temp_x = p.x - p.z * v[0];
            double temp_y = p.y - p.z * v[1];
            double region_size = 4.0;
            for (int i = std::max(temp_x - region_size, 0.0);
                 i <= std::min(x_size_ - 1, temp_x + region_size); i++) {
                for (int j = std::max(temp_y - region_size, 0.0);
                     j <= std::min(y_size_ - 1, temp_y + region_size); j++) {
                    warped_image[j][i] += gauss_func(i, j, temp_x, temp_y, 1.0);
                }
            }
        }
        for (auto &i: warped_image) {
            for (double j: i) {
                var += j * j;
            }
        }
        var /= x_size_ * y_size_;
        res[0] = 1.0 * sqrt(1.0 / var);
        return true;
    }

    explicit NumericContrastMaximizationTranslationPixel2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc) : pc_(pc) {
//        warped_image = std::vector<std::vector<double>>(x_size_, std::vector<double>(y_size_, 0));
    }

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_;
//    std::vector<std::vector<double>> warped_image;
    double x_size_ = 346;
    double y_size_ = 260;

};


#endif //LIVOX_DVS_CALIB_DVS_CM_HPP
