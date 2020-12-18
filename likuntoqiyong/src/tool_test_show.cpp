#include "common_include.h"
#include "tool_test_show.h"


void ShowUWBTrajectory(std::vector<uwb_observe> all_measure_data, std::string &picture_name, bool is_write)
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


void ShowVslamTrajectory(std::vector<vslam_observe> all_measure_data, const std::string &picture_name, bool is_write)
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



void ShowCalibrateTime(std::vector<Eigen::Vector2d> imu_yaw_normal,
                       std::vector<Eigen::Vector2d> vslam_yaw_normal,
                       double time_imu_vslam,
                       const std::string &picture_name, bool is_write)
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


void ShowCalibratePose(std::vector<uwb_observe> &uwb_observe,
                         std::vector<vslam_observe> &vslam_observe,
                         std::vector<std::pair<size_t, size_t>> matches,
                         Eigen::Matrix4d &trans_uwb_vslam,
                         const std::string &picture_name,  bool is_write)
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


void ShowTrajectoryErrorRealTime(std::vector<uwb_observe> &uwb_observe,
                         std::vector<vslam_observe> &vslam_observe,
                         std::vector<std::pair<size_t, size_t>> matches,
                         Eigen::Matrix4d &trans_uwb_vslam,
                         const std::string &picture_name,  bool is_write)
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
        likuncout << "uwb quality: " << quality_curr << "; error: " << error << std::endl;
        cv::putText(image_show, std::to_string(quality_curr), cv::Point(500,40), CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,0), 1);
        cv::putText(image_show, std::to_string(error), cv::Point(600,40), CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,0), 1);

        cv::imshow(picture_name, image_show);
        cv::waitKey(150);

        image_show(cv::Rect(400,0,400,40)).setTo(cv::Scalar(255, 255, 255));
    }

    cv::waitKey(0);

    return;
}
