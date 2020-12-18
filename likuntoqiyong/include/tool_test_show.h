#ifndef TOOL_TEST_SHOW_H
#define TOOL_TEST_SHOW_H


#include "common_include.h"


void ShowUWBTrajectory(std::vector<uwb_observe> all_measure_data, std::string &picture_name, bool is_write);


void ShowVslamTrajectory(std::vector<vslam_observe> all_measure_data, const std::string &picture_name, bool is_write);


void ShowCalibrateTime(std::vector<Eigen::Vector2d> imu_yaw_normal,
                       std::vector<Eigen::Vector2d> vslam_yaw_normal,
                       double time_imu_vslam,
                       const std::string &picture_name, bool is_write);

void ShowCalibratePose(std::vector<uwb_observe> &uwb_observe,
                         std::vector<vslam_observe> &vslam_observe,
                         std::vector<std::pair<size_t, size_t> > matches,
                         Eigen::Matrix4d &trans_uwb_vslam, const std::string &picture_name, bool is_write);

void ShowTrajectoryErrorRealTime(std::vector<uwb_observe> &uwb_observe,
                         std::vector<vslam_observe> &vslam_observe,
                         std::vector<std::pair<size_t, size_t> > matches,
                         Eigen::Matrix4d &trans_uwb_vslam, const std::string &picture_name, bool is_write);


#endif // !TOOL_TEST_SHOW_H
