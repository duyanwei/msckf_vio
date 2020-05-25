/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 *
 * @author duyanwei0702@gmail.com
 */

#ifndef MSCKF_VIO_BENCHMARK_NODELET_H
#define MSCKF_VIO_BENCHMARK_NODELET_H

#include <msckf_vio/benchmark.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace msckf_vio {
class BenchmarkNodelet : public nodelet::Nodelet {
public:
    BenchmarkNodelet() { return; }
    ~BenchmarkNodelet() { return; }

private:
    virtual void onInit();
    BenchmarkPtr benchmark_ptr_;
};
}  // namespace msckf_vio

#endif
