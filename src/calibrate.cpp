
#include "calibrate.h"

// time calibration: imu = vslam + time_imu_vslam
size_t FindNearstImu(std::vector<Eigen::Vector2d> &imu_yaw_normal, size_t &start, double &est_time)
{
    for(size_t i = start; i < imu_yaw_normal.size()-1; i++)
    {
        if(imu_yaw_normal[i][0] <= est_time && imu_yaw_normal[i+1][0] >= est_time)
            return i;
    }

    cout << "error in find nearst imu timestamp for vslam" << std::endl;
    return imu_yaw_normal.size();
}

void RunTimeCalibration(std::vector<imu_observe> &imu_observe, std::vector<wheel_observe> &wheel_observe, double &time_imu_vslam)
{
    // unify the unit of imu yaw angle and vslam yaw angle
    std::vector<Eigen::Vector2d> imu_yaw_normal;
    std::vector<Eigen::Vector2d> vslam_yaw_normal;
    for(size_t i = 0; i < imu_observe.size(); i++)
    {
        Eigen::Vector2d imu_yawi;
        imu_yawi[0] = imu_observe[i].timestamp;
        imu_yawi[1] = imu_observe[i].angle.x * 2;
        imu_yaw_normal.push_back(imu_yawi);
    }
    for(size_t i = 0; i < wheel_observe.size(); i++)
    {
        Eigen::Vector2d vslam_yawi;
        vslam_yawi[0] = wheel_observe[i].timestamp;
        vslam_yawi[1] = -1 * wheel_observe[i].wheel.z * 180 / math_pi;
        vslam_yaw_normal.push_back(vslam_yawi);
    }

    // find window of vslam data (size 5)
    std::vector<Eigen::Vector2d> vslam_window;
    for(size_t i = 0; i < vslam_yaw_normal.size() - 20; i++)
    {
        if(std::fabs(vslam_yaw_normal[i][1]) > 12 && fabs(vslam_yaw_normal[i][1]) < 170)
            vslam_window.push_back(vslam_yaw_normal[i]);
        if(vslam_window.size() >= 20)
            break;
    }

    cout << "vslam_window.size() == "<<vslam_window.size() << std::endl;

    if(vslam_window.size() < 20)
        cout << "no rotation in vslam data, can't calibration timestamp" << std::endl;

    // sliding window of imu data till error < 0.5
    bool push = false;

    std::vector<std::pair<int, double>> imu_candidate;
    for(size_t i = 0; i < imu_yaw_normal.size(); i++)
    {
        double error_tmp = 0.0;
        size_t near = 0;
        for(size_t j = 0; j < vslam_window.size(); j++)
        {
            double est_time = imu_yaw_normal[i][0] + vslam_window[j][0] - vslam_window[0][0];
            near = FindNearstImu(imu_yaw_normal, near, est_time);
            error_tmp += fabs(imu_yaw_normal[near][1] - vslam_window[j][1]);
        }

        if(push)
        {
//          cout << "imu i: " << i << "; error_tmp: " << error_tmp << std::endl;
            imu_candidate.push_back(std::pair<int, double>(i, error_tmp));
        }
        if(push == false && error_tmp < 50)
            push = true;
        if(push == true && error_tmp > 50)
            break;
    }

    double error = DBL_MAX;
    int imu_final = -1;
    for(size_t i = 0; i < imu_candidate.size(); i++)
    {
        if(error > imu_candidate[i].second)
        {
            error = imu_candidate[i].second;
            imu_final = imu_candidate[i].first;
        }
    }

    if(imu_final == -1)
        cout << "error in time calibration, please check" << std::endl;

    time_imu_vslam = imu_yaw_normal[size_t(imu_final)][0] - vslam_window[0][0];

    cout << "time_imu_vslam: " << time_imu_vslam << std::endl;

    showCalibrateTime(imu_yaw_normal, vslam_yaw_normal, time_imu_vslam, "calibrate_time.png", true);

    return;
}


// pose calibration: uwb = pose_uwb_vslam * vslam
EdgeVslamtoUwb::EdgeVslamtoUwb()
{

}

EdgeVslamtoUwb::EdgeVslamtoUwb(  Eigen::Vector3d &vslam)
        : vslam_ ( vslam )
{

}

void EdgeVslamtoUwb::computeError()
{
    std::cout<<"get in computeError() ...."<<std::endl;
    const g2o::VertexSE3Expmap* v1  = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);
    Eigen::Vector3d to_uwb = v1->estimate().map(vslam_);
    _error = _measurement - to_uwb;
    std::cout<<"_error == "<<_error<<std::endl;
    return;
}

void EdgeVslamtoUwb::linearizeOplus()
{
    std::cout<<"get in linearizeOplus() ..."<<std::endl;
    if ( level() == 1 )
    {
        _jacobianOplusXi = Eigen::Matrix<double, 3, 6>::Zero();
        return;
    }

    const g2o::VertexSE3Expmap* v1  = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    Eigen::Vector3d to_uwb = v1->estimate().map(vslam_);

    Eigen::Matrix<double, 3, 6> jacobian_vslam_ksai;

    jacobian_vslam_ksai.block(0,3,3,3) = Eigen::Matrix3d::Identity();

    jacobian_vslam_ksai.block(0,0,3,3) = Eigen::Matrix3d::Zero();
    jacobian_vslam_ksai(1,0) = -1* to_uwb(2,0);
    jacobian_vslam_ksai(0,1) = to_uwb(2,0);
    jacobian_vslam_ksai(2,0) = to_uwb(1,0);
    jacobian_vslam_ksai(0,2) = -1 * to_uwb(1,0);
    jacobian_vslam_ksai(2,1) = -1 * to_uwb(0,0);
    jacobian_vslam_ksai(1,2) = to_uwb(0,0);

    _jacobianOplusXi = -1 * jacobian_vslam_ksai;

    std::cout<<"_jacobianOplusXi == "<<_jacobianOplusXi<<std::endl;

    return;
}

size_t FindNearstUwb(std::vector<uwb_observe> &uwb_observe, size_t &start, double &est_time)
{
    for(size_t i = start; i < uwb_observe.size()-1; i++)
    {
        if(uwb_observe[i].timestamp <= est_time && uwb_observe[i+1].timestamp >= est_time)
            return i;
    }

    cout << "error in find nearst imu timestamp for vslam" << std::endl;
    return uwb_observe.size();
}

void RunPoseCalibration(std::vector<uwb_observe> &uwb_observe,
                        std::vector<vslam_observe> &vslam_observe,
                        double &time_uwb_vslam,
                        Eigen::Matrix4d &pose_uwb_vslam)
{
    std::vector<std::pair<size_t, size_t>> matches;
    size_t near = 0;
    for(size_t i = 0; i < vslam_observe.size(); i++)
    {
        double est_time = time_uwb_vslam + vslam_observe[i].timestamp;
        if(est_time < uwb_observe[0].timestamp)
            continue;
        near = FindNearstUwb(uwb_observe, near, est_time);
        if(near < uwb_observe.size())
            matches.push_back(std::pair<size_t, size_t>(i, near));
    }
    std::cout << "size of vslam and uwb matches: " << matches.size() << std::endl;
    if(matches.size() < 20)
        return;

    // compute pose_uwb_vslam use g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 6> > BlockSolver_6_6;
    g2o::SparseOptimizer optimizer;
    BlockSolver_6_6::LinearSolverType* linearSolver = new  g2o::LinearSolverEigen<BlockSolver_6_6::PoseMatrixType> ();
    BlockSolver_6_6* block_solver = new BlockSolver_6_6( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );

    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( true );

    // add pose_uwb_vslam vertex
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setId(0);
    v->setEstimate( g2o::SE3Quat()); //set origin value
    optimizer.addVertex(v);

    // add each of matches as edge
    for ( size_t i=0; i < matches.size(); i++ )
    {
        size_t vslam_id = matches[i].first;
        Eigen::Vector3d vslam(vslam_observe[vslam_id].vslam.x,vslam_observe[vslam_id].vslam.y, 0);
        //std::cout<<"vslam == "<<vslam<<std::endl;

        size_t uwb_id = matches[i].second;
        cv::Point3d uwb_id_pose = uwb_observe[uwb_id].point3d;
        Eigen::Vector3d uwb(uwb_id_pose.x, uwb_id_pose.y, 0);
        //std::cout<<"uwb == "<<uwb<<std::endl;

        EdgeVslamtoUwb* edge = new EdgeVslamtoUwb(vslam);
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0)) );

        edge->setMeasurement( uwb );
        edge->setInformation( Eigen::Matrix<double, 3, 3>::Identity());
        edge->setRobustKernel( new g2o::RobustKernelHuber() );

        optimizer.addEdge(edge);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    v = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));

    std::cout<<"v == "<<v<<std::endl;

    g2o::SE3Quat est_result = v->estimate();
    pose_uwb_vslam = est_result.to_homogeneous_matrix();

    std::cout<<"pose_uwb_vslam = "<<pose_uwb_vslam<<std::endl;

    Eigen::Vector3d translate = pose_uwb_vslam.block(0,3,3,1);
    Eigen::Matrix3d rotation_matrix = pose_uwb_vslam.block(0,0,3,3);
    Eigen::Vector3d eula_angle = rotation_matrix.eulerAngles(2,1,0);
    eula_angle = eula_angle * 180 / math_pi;
    cout <<  "translate: " << translate.transpose() << "; eular angle: " << eula_angle.transpose() << std::endl << std::endl;

    // debug for pose calibration
    showCalibratePose(uwb_observe, vslam_observe, matches, pose_uwb_vslam, "calibrate_pose.png", true);
    showTrajectoryErrorRealTime(uwb_observe, vslam_observe, matches, pose_uwb_vslam, "real_time_trajectory_error.png", false);

    return;
}