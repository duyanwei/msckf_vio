/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 *
 * @author duyanwei0702@gmail.com
 */

#include <msckf_vio/benchmark_nodelet.h>

namespace msckf_vio {
void BenchmarkNodelet::onInit() {
    benchmark_ptr_.reset(new Benchmark(getPrivateNodeHandle()));
    if (!benchmark_ptr_->initialize()) {
        ROS_ERROR("Cannot initialize benchmark ...");
        return;
    }
    return;
}

PLUGINLIB_EXPORT_CLASS(msckf_vio::BenchmarkNodelet, nodelet::Nodelet);

}  // namespace msckf_vio
