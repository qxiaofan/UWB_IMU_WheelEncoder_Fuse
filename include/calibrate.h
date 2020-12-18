#pragma once
#include "common_head.h"
#include "utils.h"
#include "CoreAlgorithm.h"

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
