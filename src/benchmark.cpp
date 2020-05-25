/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

// c++
#include <msckf_vio/benchmark.h>

namespace msckf_vio {

Benchmark::Benchmark(ros::NodeHandle& nh)
  : nh_(nh)
  , fixed_frame_id_()
  , child_frame_id_()
  , gt_()
  , anchor_pose_(boost::none) {}

bool Benchmark::initialize() {
    // load parameters
    nh_.param<std::string>("fixed_frame_id", fixed_frame_id_, "world");
    nh_.param<std::string>("child_frame_id", child_frame_id_, "robot");
    std::string file;
    nh_.param<std::string>("ground_truth", file, "");
    if (file.empty()) {
        ROS_ERROR_STREAM("unable to load groudn truth data: " << file);
        return false;
    }

    // load data
    gt_ = loadData(file);
    if (gt_.empty()) {
        ROS_ERROR("empty ground truth data");
        return false;
    }

    // subscribe topic
    odom_sub_ = nh_.subscribe(
        "odom", 1000, &Benchmark::inputEstimatedOdometryCallback, this);

    // publish topic
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("gt_odom", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("gt_path", 1);

    return true;
}

std::vector<Benchmark::Data> Benchmark::loadData(
    const std::string& name) const {

    ROS_INFO("loading ground truth data ... ");
    std::vector<Data> data;

    // open file
    FILE* file = std::fopen(name.c_str(), "r");
    if (file == NULL) {
        ROS_WARN("can't load ground truth, wrong path");
        return data;
    }

    char tmp[10000];
    if (fgets(tmp, 10000, file) == NULL)
    {
        ROS_WARN("can't load ground truth; no data available");
    }

    while (!feof(file)) {
        Data m;
        if (fscanf(file,
                   " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &m.t,
                   &m.x,
                   &m.y,
                   &m.z,
                   &m.qw,
                   &m.qx,
                   &m.qy,
                   &m.qz,
                   &m.vx,
                   &m.vy,
                   &m.vz,
                   &m.wx,
                   &m.wy,
                   &m.wz,
                   &m.ax,
                   &m.ay,
                   &m.az) != EOF) {
            m.t /= 1e9;
            data.emplace_back(m);
        }
    }
    fclose(file);
    data.pop_back();
    ROS_INFO("Loaded data size = %d", (int)data.size());
    return data;
}

void Benchmark::inputEstimatedOdometryCallback(
    const nav_msgs::OdometryConstPtr& odo_msg) {

    ROS_INFO("heard odo time = %f",  odo_msg->header.stamp.toSec());
    if (odo_msg->header.stamp.toSec() > gt_.back().t) {
        return;
    }

    // @todo currently find the upper bound of query time, a better way would be
    // interpolation
    auto iter =
        std::upper_bound(gt_.begin(),
                         gt_.end(),
                         odo_msg->header.stamp.toSec(),
                         [](const double t, const Data& m) { return t <= m.t; });
    if (iter == gt_.end()) {
        return;
    }
    ROS_INFO("time = %f", iter->t);

    // construct ground truth pose
    Eigen::Isometry3d gt_pose;
    {
        gt_pose.linear() =
            Eigen::Quaterniond(iter->qw, iter->qx, iter->qy, iter->qz)
                .toRotationMatrix();
        gt_pose.translation() = Eigen::Vector3d(iter->x, iter->y, iter->z);
    }

    // initial frame
    if (!anchor_pose_) {
        // construct estimated odometry
        Eigen::Isometry3d et_pose;
        {
            et_pose.linear() =
                Eigen::Quaterniond(odo_msg->pose.pose.orientation.w,
                                   odo_msg->pose.pose.orientation.x,
                                   odo_msg->pose.pose.orientation.y,
                                   odo_msg->pose.pose.orientation.z)
                    .toRotationMatrix();
            et_pose.translation() =
                Eigen::Vector3d(odo_msg->pose.pose.position.x,
                                odo_msg->pose.pose.position.y,
                                odo_msg->pose.pose.position.z);
        }

        // construct base pose
        Eigen::Isometry3d T = et_pose * gt_pose.inverse();
        anchor_pose_ = boost::make_optional<Eigen::Isometry3d>(T);
    }

    // transform ground truth pose
    gt_pose = *anchor_pose_ * gt_pose;

    // construct ground truth odometry
    nav_msgs::Odometry gt_odo;
    gt_odo.header.stamp = ros::Time(iter->t);
    gt_odo.header.frame_id = fixed_frame_id_;
    gt_odo.child_frame_id = child_frame_id_ + "_gt";
    tf::poseEigenToMsg(gt_pose, gt_odo.pose.pose);

    odom_pub_.publish(gt_odo);

    // construct ground truth path
    gt_path_.header = gt_odo.header;
    {
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = gt_odo.header;
        stamped_pose.pose = gt_odo.pose.pose;
        gt_path_.poses.push_back(stamped_pose);
    }

    path_pub_.publish(gt_path_);
}

}  // namespace msckf_vio