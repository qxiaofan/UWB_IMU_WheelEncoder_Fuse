#ifndef RUN_ACCUMULATE_IMU_H
#define RUN_ACCUMULATE_IMU_H


#include "common_include.h"

// time calibration: imu = vslam + time_imu_vslam
void RunAccumulateImu(std::vector<imu_observe> &imu_observe);




#endif // !RUN_ACCUMULATE_IMU_H
