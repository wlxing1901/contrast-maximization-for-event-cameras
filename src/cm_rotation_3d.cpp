//
// Created by wlxing on 20/9/2022.
//
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
#include "dvs_cm.hpp"
//#include "events_visualizer.hpp"


using namespace std;


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "********** cm_rotation_3d **********" << endl;

    // get ros params
    ros::init(argc, argv, "cm_rotation_3d");
    ros::NodeHandle nh;
    int width, height;
    nh.param<int>("width", width);
    nh.param<int>("height", height);
    std::string topic_name, rosbag_file;
    nh.param<std::string>("events_topic", topic_name, "/dvs/events");
    nh.param<std::string>("rosbag_file", rosbag_file, "");
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

    ros::Publisher event_image_pub = nh.advertise<sensor_msgs::Image>("event_image", 10);
    ros::Publisher event_image_warped_pub = nh.advertise<sensor_msgs::Image>("event_image_warped", 10);



    // read all event_array from rosbag with topic_name
    std::vector<fx::EventArrayPtr> event_array_vec;
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    rosbag::View view(bag);
    for (const rosbag::MessageInstance &m: view) {
        dv_ros_msgs::EventArray::Ptr event_array = m.instantiate<dv_ros_msgs::EventArray>();
        fx::EventArrayPtr fx_event_array = std::make_shared<fx::EventArrayType>();
        if (event_array != nullptr) {
            fx_event_array->copy_from(event_array);
            event_array_vec.push_back(fx_event_array);
        }
    }

    // show event image
    for (auto &event_array: event_array_vec) {
        cv::Mat img_es = event_stream_to_image(event_array, 2);
        cv::imshow("img_es", img_es);
        cv::waitKey(10);
    }


    for (auto &event_array: event_array_vec) {
        LOG(INFO) << "events number: " << event_array->events.size() << endl;
        double omega[3] = {0.2, 0.7, 0.7};
        double omega_initial[3] = {omega[0], omega[1], omega[2]};

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 15;

        problem.AddResidualBlock(
                new ceres::NumericDiffCostFunction<EventCameraRotation3D, ceres::CENTRAL, 1, 3>(
                        new EventCameraRotation3D(event_array, K)),
                nullptr,
                omega);


        problem.SetParameterLowerBound(omega, 0, -2.0);
        problem.SetParameterUpperBound(omega, 0, 2.0);
        problem.SetParameterLowerBound(omega, 1, -2.0);
        problem.SetParameterUpperBound(omega, 1, 2.0);
        problem.SetParameterLowerBound(omega, 2, -2.0);
        problem.SetParameterUpperBound(omega, 2, 2.0);

        TicToc t_solve;
        ceres::Solve(options, &problem, &summary);
        LOG(INFO) << "ceres solver solving time: " << t_solve.toc() << " ms" << endl;

        wait_milliseconds(100);
        cout << summary.FullReport() << endl;
        cout << "omega: " << omega_initial[0] << ", " << omega_initial[1] << ", " << omega_initial[2] << " --> "
             << omega[0] << ", " << omega[1] << ", " << omega[2]
             << endl;


        fx::EventArrayPtr event_array_ip = std::make_shared<fx::EventArrayType>();
        fx::EventArrayPtr event_array_warped = std::make_shared<fx::EventArrayType>();
        fx::EventArrayPtr event_array_warped_ip = std::make_shared<fx::EventArrayType>();

        event_stream_uv_to_image_plane(event_array, event_array_ip, K);
        warp_event_stream(event_array_ip, event_array_warped_ip, omega);
        event_stream_image_plane_to_uv(event_array_warped_ip, event_array_warped, K);

//        LOG(INFO) << "Ceres var: " << EventCameraRotation3D::compute_contrast(event_array_warped_ip) << ", w: "
//                  << omega[0] << ", " << omega[1]
//                  << ", "
//                  << omega[2]
//                  << endl;

        cv::Mat img_es = event_stream_to_image(event_array, 2);
        cv::Mat img_warped = event_stream_to_image(event_array_warped, 2);

        cv::imshow("img_es", img_es);
        cv::imshow("img_warped", img_warped);

        cv::waitKey(10);
    }


    cv::waitKey();
    return 0;
}
