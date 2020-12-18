#include "common_include.h"
#include "run_accumulate_wheel.h"


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

    return;
}

