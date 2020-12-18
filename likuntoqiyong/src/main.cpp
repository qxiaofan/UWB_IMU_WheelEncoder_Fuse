#include "common_include.h"
#include "load_data.h"

#include "run_ukf_filter.h"
#include "run_paticle_filter.h"

#include "run_accumulate_wheel.h"
#include "run_accumulate_imu.h"
#include "run_calibration.h"
#include "tool_test_show.h"
/*
 * gba and lba_homework_likun use g2o
 * */


int main( int argc, char** argv )
{

    std::string uwb_path = "/home/likun/UWBLoc/UWB-dataset/Data_all_20201208/data_UWB.log";

    // load uwb data from txt file
    std::vector<uwb_observe> all_uwb_data;
    LoadUWBMeasure(uwb_path, all_uwb_data);

    // load imu data from txt file
    std::string imu_path = "/home/likun/UWBLoc/UWB-dataset/Data_all_20201208/data_IMU.txt";
    std::vector<imu_observe> all_imu_data;
    LoadIMUMeasure(imu_path, all_imu_data);

    // load wheel data and vslam data from txt file
    std::string vslam_path = "/home/likun/UWBLoc/UWB-dataset/Data_all_20201208/wheel_encoder.txt";
    std::vector<wheel_observe> all_wheel_data;
    LoadWheelMeasure(vslam_path, all_wheel_data);
    std::vector<vslam_observe> all_vslam_data;
    LoadVslamMeasure(vslam_path, all_vslam_data);

    // imshow the vslam pose
    ShowVslamTrajectory(all_vslam_data, "vslam.png", true);

    // fiter the uwb data by karmanfilter
    std::vector<uwb_observe> all_uwb_fitered;
    RunUKarmanFilter(all_uwb_data, all_uwb_fitered);

    double time_imu_vslam = -1.0;
    double time_imu_uwb = 0.0;
    RunTimeCalibration(all_imu_data, all_wheel_data, time_imu_vslam);

    Eigen::Matrix4d pose_imu_vslam = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_uwb_vslam = Eigen::Matrix4d::Identity();
    double time_uwb_vslam = time_imu_vslam;
    RunPoseCalibration(all_uwb_fitered, all_vslam_data, time_uwb_vslam, pose_uwb_vslam);

    RunAccumulateWheel(all_wheel_data);
//    RunAccumulateImu(all_imu_data);


    return 0;
}

