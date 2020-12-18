//
// Created by yong on 2020/11/30.
//

#ifndef TEST_UWB_UTILS_H
#define TEST_UWB_UTILS_H

#include "common_head.h"

struct anchor{
    std::string name;
    cv::Point3d pose;
};

struct uwb_observe
{
    double timestamp; // timestamp of uwb data
    std::vector<double> range_to_anchor;
    cv::Point3d point3d; // uwb tag pose
    double quality; // uwb tag quality
};

struct imu_observe
{
    double timestamp; // timestamp of imu data
    cv::Point3d angle; // yaw pitch roll
    cv::Point3d acceleration; // ax,ay,az
    cv::Point3d angulay_velocity;// wx,wy,wz
};

struct wheel_observe
{
    double timestamp; // timestamp of imu data
    cv::Point3d wheel; // x, y,theta(col:3,4,5)
};

struct vslam_observe
{
    double timestamp; // timestamp of imu data
    cv::Point3d vslam; // x,y,theta(col:12,13,14)
};

//for UWB串口自动识别
string get_driver(const string& tty);

void register_comport( list<string>& comList, list<string>& comList8250, const string& dir);

void probe_serial8250_comports(list<string>& comList, list<string> comList8250);

list<string> getComList();

//for particle filter
struct current_uwb{
    float x, y, z;
};

void drawCross(cv::Mat & img, cv::Point center, cv::Scalar color, int d);



#endif //TEST_UWB_UTILS_H
