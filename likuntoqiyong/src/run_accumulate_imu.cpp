#include "common_include.h"
#include "run_accumulate_imu.h"


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

    return;
}

