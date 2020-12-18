#ifndef LOAD_DATA_H_
#define LOAD_DATA_H_

#include "common_include.h"

bool getSubstr(std::string string_in, char char1, char char2, std::string &sub_get, std::string &sub_other);

std::vector<double> split(std::string s, char token);

void LoadUWBMeasure(std::string file_path, std::vector<uwb_observe> &all_points);

void LoadIMUMeasure(std::string file_path, std::vector<imu_observe> &all_observation);

void LoadWheelMeasure(std::string file_path, std::vector<wheel_observe> &all_observation);

void LoadVslamMeasure(std::string file_path, std::vector<vslam_observe> &all_observation);


#endif // !LOAD_DATA_H_
