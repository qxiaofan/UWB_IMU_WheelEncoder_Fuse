
#include"CoreAlgorithm.h"

void showUWBTrajectory(std::vector<uwb_observe> all_measure_data,std::string &picture_name,bool is_write)
{
    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Point2d point_uwb_last;
    for(size_t i = 0; i < all_measure_data.size(); i++)
    {
        Eigen::Vector3d measure_i;
        measure_i << all_measure_data[i].point3d.x,
                all_measure_data[i].point3d.y,
                all_measure_data[i].point3d.z;

        cv::Point2d point_uwb_current;
        point_uwb_current.x = 100 * measure_i(0,0) + 400;
        point_uwb_current.y = 100 * measure_i(1,0) + 400;

        if(i == 0) point_uwb_last = point_uwb_current;

        cv::line(image_show, point_uwb_last, point_uwb_current, cv::Scalar(0, 0, 0), 2);

        point_uwb_last = point_uwb_current;
    }

    if(is_write)
        cv::imwrite(picture_name, image_show);
    else {
        cv::imshow(picture_name, image_show);
        cv::waitKey(0);
    }

    return;
}

void showVslamTrajectory(std::vector<vslam_observe> all_measure_data,const std::string &picture_name,bool is_write)
{
    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Point2d point_uwb_last;
    for(size_t i = 0; i < all_measure_data.size(); i++)
    {
        Eigen::Vector3d measure_i;
        measure_i << all_measure_data[i].vslam.x,
                all_measure_data[i].vslam.y,
                all_measure_data[i].vslam.z;

        cv::Point2d point_uwb_current;
        point_uwb_current.x = 100 * measure_i(0,0) + 400;
        point_uwb_current.y = 100 * measure_i(1,0) + 400;

        if(i == 0) point_uwb_last = point_uwb_current;

        cv::line(image_show, point_uwb_last, point_uwb_current, cv::Scalar(0, 0, 0), 1);

        point_uwb_last = point_uwb_current;
    }

    if(is_write)
        cv::imwrite(picture_name, image_show);
    else {
        cv::imshow(picture_name, image_show);
        cv::waitKey(0);
    }

    return;
}

void showCalibrateTime(std::vector<Eigen::Vector2d> imu_yaw_normal,
                       std::vector<Eigen::Vector2d> vslam_yaw_normal,
                       double time_imu_vslam,
                       const std::string &picture_name,bool is_write)
{
    for(size_t i = 0; i < vslam_yaw_normal.size(); i++)
    {
        vslam_yaw_normal[i][0] += time_imu_vslam;
    }
    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Point point_imu_last(0, 400);
    for(size_t i = 0; i < imu_yaw_normal.size()/2; i++)
    {
        cv::Point point_center;
        point_center.x = int(5 * (imu_yaw_normal[i][0] - vslam_yaw_normal[0][0]));
        point_center.y = int(imu_yaw_normal[i][1] + 400);
        cv::line(image_show, point_imu_last, point_center, cv::Scalar(0, 255, 0), 1);
        point_imu_last = point_center;
    }
    cv::Point point_vslam_last(0, 400);
    for(size_t i = 0; i < vslam_yaw_normal.size()/2; i++)
    {
        cv::Point point_center;
        point_center.x = int(5 * (vslam_yaw_normal[i][0] - vslam_yaw_normal[0][0]));
        point_center.y = int(vslam_yaw_normal[i][1] + 400);
        cv::line(image_show, point_vslam_last, point_center, cv::Scalar(255, 0, 0), 1);
        point_vslam_last = point_center;
    }

    if(is_write)
        cv::imwrite(picture_name, image_show);
    else {
        cv::imshow(picture_name, image_show);
        cv::waitKey(0);
    }

    return;
}

void showCalibratePose(std::vector<uwb_observe> &uwb_observe,
                       std::vector<vslam_observe> &vslam_observe,
                       std::vector<std::pair<size_t,size_t>> matches,
                       Eigen::Matrix4d &trans_uwb_vslam,const std::string &picture_name,
                       bool is_write)
{
    Eigen::Matrix4d transe_vslam_uwb = trans_uwb_vslam.inverse();
    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat image_show_quality(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Point2d point_uwb_last(400, 400);
    cv::Point2d point_vslam_last(400, 400);

    for(size_t i = 0; i < matches.size(); i++)
    {
        cv::Point2d point_vslam_current;
        point_vslam_current.x = 100 * vslam_observe[matches[i].first].vslam.x + 400;
        point_vslam_current.y = 100 * vslam_observe[matches[i].first].vslam.y + 400;
        if(i == 0) point_vslam_last = point_vslam_current;
        cv::line(image_show, point_vslam_last, point_vslam_current, cv::Scalar(255, 0, 0), 2);
        point_vslam_last = point_vslam_current;

        Eigen::Vector3d uwb_origin;
        uwb_origin(0,0) = uwb_observe[matches[i].second].point3d.x;
        uwb_origin(1,0) = uwb_observe[matches[i].second].point3d.y;
        uwb_origin(2,0) = 0;
        Eigen::Vector3d uwb_transe = transe_vslam_uwb.block(0,0,3,3) * uwb_origin + transe_vslam_uwb.block(0,3,3,1);
        cv::Point2d point_uwb_current;
        point_uwb_current.x = 100 * uwb_transe(0,0) + 400;
        point_uwb_current.y = 100 * uwb_transe(1,0) + 400;
        if(i == 0) point_uwb_last = point_uwb_current;
        cv::line(image_show, point_uwb_last, point_uwb_current, cv::Scalar(0, 255, 0), 2);

//        size_t pixel_value = size_t((uwb_observe[matches[i].second].quality - 40) * 6);
//        likuncout << "pixel_value: " << pixel_value << std::endl;
//        cv::circle(image_show_quality, point_uwb_current, 1.0, cv::Scalar(pixel_value, pixel_value, pixel_value), 1);

        point_uwb_last = point_uwb_current;
    }

    if(is_write)
        cv::imwrite(picture_name, image_show);
    else {
//        cv::imshow("quality_distribe.png", image_show_quality);
        cv::imshow(picture_name, image_show);
        cv::waitKey(0);
    }

    return;
}

void showTrajectoryErrorRealTime(std::vector<uwb_observe> &uwb_observe,
                                 std::vector<vslam_observe> &vslam_observe,
                                 std::vector<std::pair<size_t,size_t>> matches,
                                 Eigen::Matrix4d &trans_uwb_vslam,const std::string &picture_name,
                                 bool is_write)
{
    Eigen::Matrix4d transe_vslam_uwb = trans_uwb_vslam.inverse();
    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Point2d point_uwb_last(400, 400);
    cv::Point2d point_vslam_last(400, 400);
    for(size_t i = 0; i < matches.size(); i++)
    {
        cv::Point2d point_vslam_current;
        point_vslam_current.x = 100 * vslam_observe[matches[i].first].vslam.x + 400;
        point_vslam_current.y = 100 * vslam_observe[matches[i].first].vslam.y + 400;

        if(i == 0) point_vslam_last = point_vslam_current;
        cv::line(image_show, point_vslam_last, point_vslam_current, cv::Scalar(255, 0, 0), 2);
        point_vslam_last = point_vslam_current;

        Eigen::Vector3d uwb_origin;
        uwb_origin(0,0) = uwb_observe[matches[i].second].point3d.x;
        uwb_origin(1,0) = uwb_observe[matches[i].second].point3d.y;
        uwb_origin(2,0) = 0;
        Eigen::Vector3d uwb_transe = transe_vslam_uwb.block(0,0,3,3) * uwb_origin + transe_vslam_uwb.block(0,3,3,1);
        cv::Point2d point_uwb_current;
        point_uwb_current.x = 100 * uwb_transe(0,0) + 400;
        point_uwb_current.y = 100 * uwb_transe(1,0) + 400;

        if(i == 0) point_uwb_last = point_uwb_current;
        cv::line(image_show, point_uwb_last, point_uwb_current, cv::Scalar(0, 255, 0), 2);
        point_uwb_last = point_uwb_current;

        Eigen::Vector3d Error_vec(uwb_transe(0,0) - vslam_observe[matches[i].first].vslam.x,
                                  uwb_transe(1,0) - vslam_observe[matches[i].first].vslam.y,
                                  0);
        double error = sqrt(Error_vec.transpose() * Error_vec);
        int quality_curr = uwb_observe[matches[i].second].quality;
        cout << "uwb quality: " << quality_curr << "; error: " << error << std::endl;
        cv::putText(image_show, std::to_string(quality_curr), cv::Point(500,40), CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,0), 1);
        cv::putText(image_show, std::to_string(error), cv::Point(600,40), CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,0), 1);

        cv::imshow(picture_name, image_show);
        cv::waitKey(150);

        image_show(cv::Rect(400,0,400,40)).setTo(cv::Scalar(255, 255, 255));
    }

    cv::waitKey(0);

    return;

}

// accumulate the wheel data(x,y,theta)
void RunAccumulateWheel(std::vector<wheel_observe> &wheel_observe)
{
    // debug for time calibration
//    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
//    cv::Point point_wheel_last(400, 400);
//    for(size_t i = 0; i < vslam_observe.size(); i++)
//    {
//        cv::Point point_center;
//        point_center.x = 100 * vslam_observe[i].wheel.x + 400;
//        point_center.y = 100 * vslam_observe[i].wheel.y + 400;
//        cv::line(image_show, point_wheel_last, point_center, cv::Scalar(0, 255, 0), 1);
//        point_wheel_last = point_center;
//    }
//    cv::imwrite("wheel_encoder.png", image_show);
//    cv::waitKey(0);
}

// time calibration: imu = vslam + time_imu_vslam
void RunAccumulateImu(std::vector<imu_observe> &imu_observe)
{
//    // debug for time calibration
//    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
//    cv::Point point_imu_last(0, 400);
//    for(size_t i = 0; i < imu_yaw_normal.size()/2; i++)
//    {
//        cv::Point point_center;
//        point_center.x = 5 * (imu_yaw_normal[i][0] - vslam_yaw_normal[0][0]);
//        point_center.y = imu_yaw_normal[i][1] + 400;
//        cv::line(image_show, point_imu_last, point_center, cv::Scalar(0, 255, 0), 1);
//        point_imu_last = point_center;
//    }
//    cv::Point point_vslam_last(0, 400);
//    for(size_t i = 0; i < vslam_yaw_normal.size()/2; i++)
//    {
//        cv::Point point_center;
//        point_center.x = 5 * (vslam_yaw_normal[i][0] - vslam_yaw_normal[0][0]);
//        point_center.y = vslam_yaw_normal[i][1] + 400;
//        cv::line(image_show, point_vslam_last, point_center, cv::Scalar(255, 0, 0), 1);
//        point_vslam_last = point_center;
//    }
//    cv::imshow("image_show", image_show);
//    cv::waitKey(0);

}

void runParticleFilter(std::vector<uwb_observe> &uwb_observe)
{

    //UWB particle filter
    //三维
    std::vector<cv::Point3f> vcurrent_pos, vparticle_pos;

    int DP = 3;
    int nParticles = 200;
    float xRange = 800.0f;
    float flocking = 0.9f;
    float minRange[] = {0.0f,0.0f,0.0f};
    float maxRange[] = {xRange,xRange,xRange};
    cv::Mat_<float> LB(1,DP,minRange);
    cv::Mat_<float> UB(1,DP,maxRange);
    cv::Mat_<float> measurement(1,DP);
    cv::Mat_<float> dyna(cv::Mat_<float>::eye(3,3));

    ConDensation condens(DP,nParticles);

    cv::Mat img((int)xRange,(int)xRange,CV_8UC3);
    cv::namedWindow("UWB particle filter");

    condens.initSampleSet(LB,UB,dyna);

    for(size_t i = 0; i < uwb_observe.size(); ++i)
    {
        current_uwb cur_uwb;
        cv::waitKey(30);

        cur_uwb.x = uwb_observe[i].point3d.x * 100;
        cur_uwb.y = uwb_observe[i].point3d.y * 100;
        cur_uwb.z = uwb_observe[i].point3d.z * 100;

        measurement(0) = float(cur_uwb.x);
        measurement(1) = float(cur_uwb.y);
        measurement(2) = float(cur_uwb.z);

        cv::Point3f measPt(cur_uwb.x,cur_uwb.y,cur_uwb.z);
        vcurrent_pos.push_back(measPt);

        //Clear screen
        img = cv::Scalar::all(100);

        //Update and get prediction:
        const cv::Mat_<float> &pred = condens.correct(measurement);
        cv::Point3f statePt(pred(0),pred(1),pred(2));
        vparticle_pos.push_back(statePt);

        std::cout<<"measPt: "<<measPt.x <<" "<<measPt.y<<" "<<measPt.z<<" "
                 <<"statePt: "<<statePt.x <<" "<<statePt.y<<" "<<statePt.z<<std::endl;

        for(int s = 0; s < condens.sampleCount();s++)
        {
            cv::Point2f partPt(condens.sample(s,0), condens.sample(s,1));
            drawCross(img,partPt,cv::Scalar(255,90,(int)(s*255.0)/(float)condens.sampleCount()),2);
        }


        for(size_t i = 0; i < vcurrent_pos.size() - 1; i++)
        {
            cv::line(img,cv::Point2f(vcurrent_pos[i].x,vcurrent_pos[i].y),cv::Point2f(vcurrent_pos[i + 1].x,vcurrent_pos[i + 1].y),cv::Scalar(255,0,0),1);
        }

        for(size_t i = 0; i < vparticle_pos.size() - 1; i++)
        {
            cv::line(img,cv::Point2f(vparticle_pos[i].x,vparticle_pos[i].y),cv::Point2f(vparticle_pos[i + 1].x,vparticle_pos[i + 1].y),cv::Scalar(0,255,0),1);
        }

        drawCross(img,cv::Point2f(statePt.x,statePt.y),cv::Scalar(255,255,255),5);
        drawCross(img,cv::Point2f(measPt.x,measPt.y),cv::Scalar(0,0,255),5);

        cv::imwrite("particle_filter.png",img);

        cv::Mat imgshow = cv::Mat::zeros(640,640,CV_8UC3);
        cv::resize(img,imgshow,cv::Size(500,500));
        cv::imshow("UWB particle filter",imgshow);
    }

    return;
}

class UKF2DPoint : public UKF {
public:
    Matrix state_function (Matrix s)
    {
        Matrix state(4,1);
        state(0,0) = s(0,0)+s(2,0);	// x position in 2D point
        state(1,0) = s(1,0)+s(3,0);	// y position in 2D point
        state(2,0) = s(2,0);	// velocity in x
        state(3,0) = s(3,0);	// velocity in y
        return state;
    }

    Matrix measurement_function (Matrix m)
    {
        Matrix measurement(2,1);
        measurement(0,0) = m(0,0);	// measured x position in 2D point
        measurement(1,0) = m(1,0);	// measured y position in 2D point
        return measurement;
    }
};

void runUKarmanFilter(std::vector<uwb_observe> &uwb_observe_origin, std::vector<uwb_observe> &uwb_observe_filter)
{
    float xRange = 800.0f;
    cv::Mat img((int)xRange,(int)xRange,CV_8UC3);

    cv::namedWindow("ukf");

    unsigned int n = 4;
    unsigned int m = 2;

    UKF2DPoint tracked_point;
    tracked_point.n = n;
    tracked_point.m = m;

    Matrix I4(4,4); //4x4 Identity Matrix
    I4(0,0) = 1;
    I4(1,1) = 1;
    I4(2,2) = 1;
    I4(3,3) = 1;
    Matrix I2(m,m); //2x2 Identity Matrix
    I2(0,0) = 1;
    I2(1,1) = 1;

    Matrix s(n,1); //initial state
    s(0,0) = 1;
    s(1,0) = 1;
    s(2,0) = 0;
    s(3,0) = 0;

    Matrix  x = s; //initial state
    const double q=0.1;	//std of process. "smoothness". lower the value, smoother the curve
    const double r=0.1;	//std of measurement. "tracking". lower the value, faster the track
    tracked_point.P = I4;	// state covriance
    tracked_point.Q = (q*q) * I4;	// covariance of process	(size must be nxn)
    tracked_point.R = (r*r) * I2;	// covariance of measurement (size must be mxm)

    unsigned int N = uwb_observe_origin.size();
    Matrix xV(n,N);	//estmate        // allocate memory to show outputs
    Matrix sV(n,N);	//actual
    Matrix zV(m,N);

    std::vector<double> deltData;

    std::vector<cv::Point2f> vcurrent_pos,vukf_pos;

    for(unsigned int k = 0; k < uwb_observe_origin.size();k++)
    {
        cv::Point2f measPt = cv::Point2f(uwb_observe_origin[k].point3d.x*100,
                                         uwb_observe_origin[k].point3d.y*100 );
        s(0,0) = uwb_observe_origin[k].point3d.x*100;	// measured x position in 2D point
        s(1,0) = uwb_observe_origin[k].point3d.y*100;	// measured y position in 2D point

        vcurrent_pos.push_back(cv::Point2f(s(0,0),s(1,0)));

        Matrix z = tracked_point.measurement_function(s); //make measurements

        for (unsigned int i=0; i<n; i++)
        {
            sV(i,k) = s(i,0);	// save actual state
        }
        for (unsigned int i=0; i<m; i++)
        {
            zV(i,k) = z(i,0);	// save measurement
        }

        tracked_point.ukf(x,z);

        cv::Point2f statePt= cv::Point2f(x(0,0),x(1,0));

        vukf_pos.push_back(statePt);

        for (unsigned int i=0; i<n; i++)
        {
            xV(i,k) = x(i,0);	// save estimate
        }

        //Clear screen
        img = cv::Scalar::all(100);

        for(size_t i = 0; i < vcurrent_pos.size() - 1; i++)
        {
            cv::line(img,vcurrent_pos[i],vcurrent_pos[i + 1],cv::Scalar(255,0,0),1);
        }

        for(size_t i = 0; i < vukf_pos.size() - 1; i++)
        {
            cv::line(img,vukf_pos[i],vukf_pos[i + 1],cv::Scalar(0,255,0),1);
        }

        s = tracked_point.state_function(s); // update process with artificial increment
        drawCross(img,statePt,cv::Scalar(255,255,255),5);
        drawCross(img,measPt,cv::Scalar(0,0,255),5);

        std::cout<<"vcurrent_pos.size() == "<<vcurrent_pos.size()<<std::endl;
        std::cout<<"vukf_pos.size() == "<<vukf_pos.size()<<std::endl;
        std::cout<<"statePt == " <<statePt<<std::endl;
        std::cout<<"measPt == "<<measPt<<std::endl;
        double dis = sqrt(pow((statePt.x - measPt.x),2)+pow((statePt.y - measPt.y),2));
        std::cout<<"dis == "<<dis<<std::endl;
        deltData.push_back(dis);

        cv::imwrite("../ukf.png",img);
        cv::Mat imgshow = cv::Mat::zeros(640,640,CV_8UC3);
        cv::resize(img,imgshow,cv::Size(500,500));
        cv::imshow("ukf",img);
        cv::waitKey(30);
    }

    cout << "\nDifference between actual states and tracked measurements:\n";
    for (unsigned int k=0; k<N; k++)
    {
        cout << k << ": " << abs( sV(0,k) - xV(0,k) ) << "  (" << sV(0,k) << '-' << xV(0,k) << ")\n";
    }
    cv::Mat meanValue,stdValue;
    cv::meanStdDev(deltData,meanValue,stdValue);

    std::cout<<"meanValue == "<<meanValue<<std::endl;
    std::cout<<"stdValue == "<<stdValue<<std::endl;

    return;
}
