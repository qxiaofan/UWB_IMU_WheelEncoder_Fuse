#pragma once
#include "utils.h"
#include "common_head.h"
#include "Condensation.h"
#include "UKF.h"

void showUWBTrajectory(std::vector<uwb_observe> all_measure_data,std::string &picture_name,bool is_write);

void showVslamTrajectory(std::vector<vslam_observe> all_measure_data,const std::string &picture_name,bool is_write);

void showCalibrateTime(std::vector<Eigen::Vector2d> imu_yaw_normal,
                       std::vector<Eigen::Vector2d> vslam_yaw_normal,
                       double time_imu_vslam,
                       const std::string &picture_name,bool is_write);

void showCalibratePose(std::vector<uwb_observe> &uwb_observe,
                       std::vector<vslam_observe> &vslam_observe,
                       std::vector<std::pair<size_t,size_t>> matches,
                       Eigen::Matrix4d &trans_uwb_vslam,const std::string &picture_name,
                       bool is_write);

void showTrajectoryErrorRealTime(std::vector<uwb_observe> &uwb_observe,
                                 std::vector<vslam_observe> &vslam_observe,
                                 std::vector<std::pair<size_t,size_t>> matches,
                                 Eigen::Matrix4d &trans_uwb_vslam,const std::string &picture_name,
                                 bool is_write);

// accumulate the wheel data(x,y,theta)
void runAccumulateWheel(std::vector<wheel_observe> &wheel_observe);

// time calibration: imu = vslam + time_imu_vslam
void runAccumulateImu(std::vector<imu_observe> &imu_observe);

void runParticleFilter(std::vector<uwb_observe> &uwb_observe);

void runUKarmanFilter(std::vector<uwb_observe> &uwb_observe_origin, std::vector<uwb_observe> &uwb_observe_filter);

