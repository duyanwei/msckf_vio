/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 *
 * @author duyanwei0702@gmail.com
 */

#ifndef MSCKF_VIO_BENCHMARK_H_
#define MSCKF_VIO_BENCHMARK_H_

// c++
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// 3rdparty
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <boost/optional.hpp>

namespace msckf_vio {

class Benchmark {
public:
    typedef std::shared_ptr<Benchmark> Ptr;
    typedef std::shared_ptr<const Benchmark> ConstPtr;
    struct Data {
        double t;
        float x, y, z;
        float qw, qx, qy, qz;
        float vx, vy, vz;
        float wx, wy, wz;
        float ax, ay, az;
    };

    /**
     * @brief Construct a new Benchmak object
     *
     * @param nh ros node handler
     */
    Benchmark(ros::NodeHandle& nh);

    /**
     * @brief disable copy constructor
     */
    Benchmark(const Benchmark&) = delete;

    /**
     * @brief 
     * 
     * @return Benchmark 
     */
    Benchmark operator=(const Benchmark&) = delete;

    ~Benchmark() {}

    /**
     * @brief initialize benchmark
     *
     * @return true
     * @return false
     */
    bool initialize();

private:
    /**
     * @brief load data
     *
     * @param name file
     */
    std::vector<Data> loadData(const std::string& name) const;

    /**
     * @brief input estimated odometry callback
     *
     * @param odo_msg
     */
    void inputEstimatedOdometryCallback(
        const nav_msgs::OdometryConstPtr& odo_msg);

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    nav_msgs::Path gt_path_;

    ros::Subscriber odom_sub_;

    std::string fixed_frame_id_;
    std::string child_frame_id_;

    std::vector<Data> gt_;

    boost::optional<Eigen::Isometry3d> anchor_pose_;
};  // class Benchmark

typedef Benchmark::Ptr BenchmarkPtr;

}  // namespace msckf_vio

#endif  // MSCKF_VIO_BENCHMARK_H_