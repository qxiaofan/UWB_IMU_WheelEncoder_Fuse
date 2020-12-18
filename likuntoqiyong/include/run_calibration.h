#ifndef RUN_CALIBRATION_H
#define RUN_CALIBRATION_H


#include "common_include.h"


// time calibration: imu = vslam + time_imu_vslam
void RunTimeCalibration(std::vector<imu_observe> &imu_observe, std::vector<wheel_observe> &wheel_observe, double &time_imu_vslam);

// pose calibration: uwb = pose_uwb_vslam * vslam
void RunPoseCalibration(std::vector<uwb_observe> &uwb_observe,
                        std::vector<vslam_observe> &vslam_observe,
                        double &time_uwb_vslam,
                        Eigen::Matrix4d &pose_uwb_vslam);

class EdgeVslamtoUwb: public g2o::BaseUnaryEdge< 3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVslamtoUwb();
    EdgeVslamtoUwb(Eigen::Vector3d &vslam);

    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read ( std::istream& in ){};
    virtual bool write ( std::ostream& out ) const{};

public:
    Eigen::Vector3d vslam_;   // pose in vslam coordinate
    double weight_ = 0.0;

};


#endif // !RUN_CALIBRATION_H

//    double time_imu_vslam = -1.0;
//    double time_imu_uwb = 0.0;
//    RunTimeCalibration(all_imu_data, all_vslam_data, time_imu_vslam);

//    Eigen::Matrix4d pose_imu_vslam = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d pose_uwb_vslam = Eigen::Matrix4d::Identity();
//    double time_uwb_vslam = time_imu_vslam;
//    RunPoseCalibration(all_uwb_data, all_vslam_data, time_uwb_vslam, pose_uwb_vslam);
