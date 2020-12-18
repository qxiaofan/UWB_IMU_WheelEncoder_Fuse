#ifndef COMMON_INCLUDE_
#define COMMON_INCLUDE_

// for std
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// for opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// for eigen3
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Cholesky"


// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_dense.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/types/se3quat.h>
#include <g2o/types/types_six_dof_expmap.h>
#include <g2o/types/types_sba.h>


#define likuncout std::cout << "likun uwb: "
#define math_pi 3.141592653

struct anchor{
    std::string name;
    cv::Point3d pose;
};

struct uwb_observe{
    double timestamp;  // timestamp of uwb data
    std::vector<double> range_to_anchor;
    cv::Point3d point3d;  // uwb tag pose
    double quality;  // uwb tag quality
};

struct imu_observe{
    double timestamp; // timestamp of imu data
    cv::Point3d angle; // yaw pitch roll
    cv::Point3d acceleration; // ax, ay, az
    cv::Point3d angulay_velocity; // wx, wy, wz
};

struct wheel_observe{
    double timestamp; // timestamp of imu data
    cv::Point3d wheel; // x, y, theta (col:3, 4, 5)
};

struct vslam_observe{
    double timestamp; // timestamp of imu data
    cv::Point3d vslam; // x, y, theta (col:12, 13, 14)
};

#endif // !COMMON_INCLUDE_
