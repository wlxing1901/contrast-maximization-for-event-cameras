//
// Created by wlxing on 2/1/23.
//


#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <dv_ros_msgs/EventArray.h>
#include <dv_ros_msgs/Event.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/console/time.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <numeric>
#include <cmath>
#include <thread>
#include <future>

#include <iostream>
#include <ros/ros.h>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

#include <dv_ros_msgs/EventArray.h>
#include <dv_ros_msgs/Event.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//#include "dvs_utils.hpp"
//#include "dvs_cm.hpp"


using namespace std;


cv::Mat event_stream_to_image(std::vector<dv_ros_msgs::Event> &event_vec_, int zoom_factor = 2,
                              bool non_maximum_suppression = false, int x_size = 346,
                              int y_size = 260,
                              int add_value = 20.0) {
    cv::Mat image = cv::Mat::zeros(y_size * zoom_factor, x_size * zoom_factor, CV_8UC3);
    for (auto &p: event_vec_) {
        int channel = (p.polarity == 1 ? 0 : 2);
        int u = p.x;
        int v = p.y;
        if (u < 0 || v < 0 || u >= x_size || v >= y_size) {
            continue;
        }
        u *= zoom_factor;
        v *= zoom_factor;
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
 * @brief Calculate the gauss function
 * @param x
 * @param y
 * @param x0
 * @param y0
 * @param sigma
 * @return
 */
inline double gauss_func(double x, double y, double x0, double y0, double sigma = 1.0) {
    double res = 0.0;
    res = 1.0 / (2 * M_PI * sigma * sigma) *
          exp(-1.0 * ((x - x0) * (x - x0) + (y - y0) * (y - y0)) / (2.0 * sigma * sigma));
    return res;
}


struct EventVelocity2D {
public:
    bool operator()(const double *const v, double *const res) const {
        double var = 0.0;
        // futures to store the results of each thread
        std::vector<std::future<Eigen::MatrixXd>> futures(num_threads_);
        for (int i = 0; i < num_threads_; ++i) {
            auto start_iter = event_selected_vec_.cbegin() + i * num_events_per_thread_;
            auto end_iter = (i == num_threads_ - 1) ? event_selected_vec_.cend() : (event_selected_vec_.cbegin() +
                                                                                    (i + 1) * num_events_per_thread_);

            // multi-thread, to speed up the calculation
            futures[i] = std::async(std::launch::async,
                                    [this, start_iter, end_iter](double vx, double vy) {
                                        return this->process_events(start_iter, end_iter, vx, vy);
                                    }, v[0], v[1]);
        }

        Eigen::MatrixXd warped_image = Eigen::MatrixXd::Zero(y_size_, x_size_);
        // wait for all threads to finish, the fut.get() will block until the result is ready
        for (auto &fut: futures) {
            warped_image += fut.get();
        }
        var = warped_image.squaredNorm() / (x_size_ * y_size_);
        res[0] = -sqrt(var) + 1000.0;
        return true;
    }

    explicit EventVelocity2D(std::vector<dv_ros_msgs::Event> &event_vec) {
        for (int i = 0; i < gauss_func_table_size_; i++) {
            double res = 0.0;
            res = 1.0 / (2 * M_PI * sigma_ * sigma_) *
                  exp(-1.0 * (double(i) / 10) / (2.0 * sigma_ * sigma_));
            gauss_func_table_.push_back(res);
        }
        if (event_vec.size() > event_selected_num_) {
            compute_ratio_ = event_vec.size() / event_selected_num_;
        }

        int event_cnt = 0;
        int event_used_cnt = 0;
        double event_selected = 0;
        for (const auto &e: event_vec) {
            event_cnt++;
            if (compute_ratio_ > 1.01) {
                if (event_cnt >= event_selected) {
                    event_selected += compute_ratio_;
                } else {
                    continue;
                }
            }
            event_used_cnt++;
            event_selected_vec_.push_back(e);
        }
        num_events_per_thread_ = event_selected_vec_.size() / num_threads_;
        start_event_ts_ = event_vec[0].ts.toSec();
    }

private:
    std::vector<dv_ros_msgs::Event> event_selected_vec_;
    double x_size_ = 346;
    double y_size_ = 260;
    double compute_ratio_ = 1;
    double event_selected_num_ = 12000.0;
    std::vector<double> gauss_func_table_;
    int gauss_func_table_size_ = 200;
    int num_threads_ = 4;
    int num_events_per_thread_ = 0;
    int patch_size_ = 4;
    double start_event_ts_ = 0;
    double sigma_ = 1.0;

    inline double find_gauss_func_table(double x, double y, double x0, double y0) const {
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

    Eigen::MatrixXd process_events(const std::vector<dv_ros_msgs::Event>::const_iterator start,
                                   const std::vector<dv_ros_msgs::Event>::const_iterator end,
                                   const double v0,
                                   const double v1) const {
        Eigen::MatrixXd warped_image = Eigen::MatrixXd::Zero(y_size_, x_size_);

        for (auto e_iter = start; e_iter != end; ++e_iter) {
            const auto &e = *e_iter;
            double temp_x = e.x - (e.ts.toSec() - start_event_ts_) * v0;
            double temp_y = e.y - (e.ts.toSec() - start_event_ts_) * v1;
            if (temp_x < 0 || temp_y < 0 || temp_x >= x_size_ || temp_y >= y_size_) {
                continue;
            }

            int temp_x_int = int(std::round(temp_x) + 1e-6);
            int temp_y_int = int(std::round(temp_y) + 1e-6);
            // find the corresponding pixel in the warped image
            for (int i = -patch_size_; i <= patch_size_; i++) {
                for (int j = -patch_size_; j <= patch_size_; j++) {
                    if (temp_x_int + i < 0 || temp_y_int + j < 0 || temp_x_int + i >= x_size_ ||
                        temp_y_int + j >= y_size_) {
                        continue;
                    }
                    warped_image(temp_y_int + j, temp_x_int + i) +=
                            find_gauss_func_table(temp_x_int + i, temp_y_int + j, temp_x, temp_y);

                }
            }
        }
        return warped_image;
    }
};

/**
 * @brief Track the velocity of the event camera
 * @param events_frame The events in the current frame
 * @param warped_frame The events in the warped frame
 * @param v2d The velocity in the x and y direction
 * @param p2d The position in the x and y direction
 */
void event_track_2d(std::vector<dv_ros_msgs::Event> &events_frame, std::vector<dv_ros_msgs::Event> &warped_frame,
                    std::vector<double> &v2d, std::vector<double> &p2d) {

    double v[2] = {v2d[0], v2d[1]};
    double v_initial[2] = {v[0], v[1]};

    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5000;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;

    problem.AddResidualBlock(
            new ceres::NumericDiffCostFunction<EventVelocity2D, ceres::CENTRAL, 1, 2>(
                    new EventVelocity2D(events_frame)),
            nullptr,
            v);

    pcl::console::TicToc t_solve;
    t_solve.tic();
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "ceres solver solving time: " << t_solve.toc() / 1000 << " s" << std::endl;
//    LOG(INFO) << summary.FullReport() << std::endl;
    LOG(INFO) << "v: " << v_initial[0] << ", " << v_initial[1] << " --> " << v[0] << ", " << v[1] << std::endl;

    for (auto &e: events_frame) {
        dv_ros_msgs::Event temp;
        temp.ts = e.ts;
        temp.x = e.x - v[0] * (e.ts.toSec() - events_frame[0].ts.toSec());
        temp.y = e.y - v[1] * (e.ts.toSec() - events_frame[0].ts.toSec());
        temp.polarity = e.polarity;
        temp.ts = e.ts;
        warped_frame.push_back(temp);
    }

    v2d = {v[0], v[1]};
    p2d = {std::accumulate(warped_frame.begin(), warped_frame.end(), 0.0,
                           [](double sum, const dv_ros_msgs::Event &e) {
                               return sum + e.x;
                           }) / warped_frame.size(),
           std::accumulate(warped_frame.begin(), warped_frame.end(), 0.0,
                           [](double sum, const dv_ros_msgs::Event &e) {
                               return sum + e.y;
                           }) / warped_frame.size()};
}


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "********** cm_translation_2d **********" << endl;

    // get ros params
    ros::init(argc, argv, "cm_translation_2d");
    ros::NodeHandle nh;
    int width, height;
    nh.param<int>("width", width, -1);
    nh.param<int>("height", height, -1);
    std::string topic_name, rosbag_file, output_path;
    nh.param<std::string>("events_topic", topic_name, "/dvs/events");
    nh.param<std::string>("rosbag_file", rosbag_file, "");
    nh.param<std::string>("output_path", output_path, "");
    vector<vector<double>> K = {{200.0, 0.0,   173.0},
                                {0.0,   200.0, 130.0},
                                {0.0,   0.0,   1.0}};


    // log params
    LOG(INFO) << "********** ros params: **********" << endl;
    LOG(INFO) << "width: " << width << ", height: " << height << endl;
    LOG(INFO) << "intrinsic matrix: " << [](vector<vector<double>> K) {
        stringstream ss;
        ss << endl;
        for (int i = 0; i < K.size(); i++) {
            for (int j = 0; j < K[i].size(); j++) {
                ss << K[i][j] << " ";
            }
            ss << endl;
        }
        return ss.str();
    }(K);
    LOG(INFO) << "events topic: " << topic_name << endl;

    // read all event_array from rosbag with topic_name
    std::vector<dv_ros_msgs::Event> event_vec;
    std::vector<std::vector<dv_ros_msgs::Event>> all_frames;
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    rosbag::View view(bag);
    for (const rosbag::MessageInstance &m: view) {
        dv_ros_msgs::EventArray::Ptr event_array = m.instantiate<dv_ros_msgs::EventArray>();
        if (event_array != nullptr) {
            for (auto &e: event_array->events) {
                event_vec.push_back(e);
            }
        }
    }
    ros::Time start_time = event_vec[0].ts, end_time = event_vec.back().ts;
    LOG(INFO) << "start_time: " << start_time << ", end_time: " << end_time << std::endl;
    double duration = 0.10;
    for (double begin_time = start_time.toSec(); begin_time < end_time.toSec(); begin_time += duration) {
        std::vector<dv_ros_msgs::Event> events_frame;
        // use lower_bound() to find the first element with a timestamp greater than or equal to the start time
        auto start_it = std::lower_bound(event_vec.begin(), event_vec.end(), begin_time,
                                         [](const dv_ros_msgs::Event &e, double t) { return e.ts.toSec() < t; });
        auto end_it = std::upper_bound(event_vec.begin(), event_vec.end(), begin_time + duration,
                                       [](double t, const dv_ros_msgs::Event &e) { return t < e.ts.toSec(); });
        std::copy(start_it, end_it, std::back_inserter(events_frame));
        LOG(INFO) << "events_frame has " << events_frame.size() << " events" << std::endl;
        all_frames.push_back(events_frame);
    }
    std::vector<double> v2d = {60.0, -20.0}, p2d = {0.0, 0.0};

    //write t_wait to a text file
    std::ofstream outfile_tcm;
    outfile_tcm.open("/home/wlxing/Codes/catkin_ws/src/event_lidar_velocity/cache/t_cm_used_numdiff_o3_multithread.txt",
                     std::ios::out);

    std::ofstream outfile_twait;
    outfile_twait.open("/home/wlxing/Codes/catkin_ws/src/event_lidar_velocity/cache/t_wait_numdiff_o3_multithread.txt",
                       std::ios::out);
    // write v2d to a text file
    std::ofstream outfile_v2d;
    outfile_v2d.open("/home/wlxing/Codes/catkin_ws/src/event_lidar_velocity/cache/v2d_numdiff_o3_multithread.txt",
                     std::ios::out);

    pcl::console::TicToc t_used;
    int count = 0;
    for (auto events_frame: all_frames) {
        t_used.tic();
        std::vector<dv_ros_msgs::Event> warped_frame;
        pcl::console::TicToc t_cm_used;
        t_cm_used.tic();

        event_track_2d(events_frame, warped_frame, v2d, p2d);
        double t_cm_used_double = t_cm_used.toc() / 1000.0;

        cv::Mat image = event_stream_to_image(events_frame, 2, true, 346, 260, 50.0);
        cv::imshow("image", image);

        cv::imwrite(output_path + "/img_es_" + to_string(count) + ".png", image);
        cv::Mat warped_image = event_stream_to_image(warped_frame, 2, true, 346, 260, 50.0);

        //draw arrow on warped_image
        cv::Point2f p1 = cv::Point2f(p2d[0], p2d[1]);
        cv::Point2f p2 = cv::Point2f(p2d[0] + v2d[0], p2d[1] + v2d[1]);
        cv::arrowedLine(warped_image, 2 * p1, 2 * p2, cv::Scalar(0, 255, 0), 2, 8, 0, 0.1);

        cv::imshow("warped_image", warped_image);
        cv::imwrite(output_path + "/img_es_warped_" + to_string(count) + ".png", warped_image);
        cv::waitKey(1);

        double t_wait = duration - t_used.toc() / 1000;
        LOG(INFO) << "t_wait: " << t_wait << std::endl;
        if (t_wait > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds((int) (t_wait * 1000)));
        }
        outfile_tcm << t_cm_used_double << std::endl;
        outfile_twait << t_wait << std::endl;
        outfile_v2d << v2d[0] << " " << v2d[1] << std::endl;
        count++;
    }
    outfile_tcm.close();
    outfile_twait.close();
    outfile_v2d.close();

    cv::waitKey(0);
    return 0;
}
