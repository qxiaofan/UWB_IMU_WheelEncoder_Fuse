#include "common_include.h"
#include "run_ukf_filter.h"

/*
 * karman_filter
 * */
class UKF2DPoint : public UKF {
public:
    Eigen::MatrixXd state_function (Eigen::MatrixXd s)
    {
        Eigen::MatrixXd state(4,1);
        state(0,0) = s(0,0)+s(2,0);	// x position in 2D point
        state(1,0) = s(1,0)+s(3,0);	// y position in 2D point
        state(2,0) = s(2,0);	// velocity in x
        state(3,0) = s(3,0);	// velocity in y
        return state;
    }

    Eigen::MatrixXd measurement_function (Eigen::MatrixXd m)
    {
        Eigen::MatrixXd measurement(2,1);
        measurement(0,0) = m(0,0);	// measured x position in 2D point
        measurement(1,0) = m(1,0);	// measured y position in 2D point
        return measurement;
    }
};

void RunUKarmanFilter(std::vector<uwb_observe> &uwb_observe_origin, std::vector<uwb_observe> &uwb_observe_filter)
{
    uwb_observe_filter = uwb_observe_origin;

    // fitering uwb data using UKF
    unsigned int n = 4;
    unsigned int m = 2;

    UKF2DPoint tracked_point;
    tracked_point.n = n;
    tracked_point.m = m;

    Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity(); //4x4 Identity Matrix
    Eigen::Matrix2d I2 = Eigen::Matrix2d::Identity(); //4x4 Identity Matrix

    Eigen::MatrixXd s(n,1); //initial state
    s(0,0) = uwb_observe_origin[0].point3d.x;
    s(1,0) = uwb_observe_origin[0].point3d.y;
    s(2,0) = 0;
    s(3,0) = 0;
    Eigen::MatrixXd  x = s; //initial state

    Eigen::MatrixXd measure(m,1);
    measure(0,0) =  uwb_observe_origin[0].point3d.x;	// measured x position in 2D point
    measure(0,0) = uwb_observe_origin[0].point3d.y;	// measured y position in 2D point


    const double q=0.1;  //std of process. "smoothness". lower the value, smoother the curve
    const double r=1.0;  //std of measurement. "tracking". lower the value, faster the track
    tracked_point.P = I4;	// state covriance
    tracked_point.Q = (q*q) * I4;	// covariance of process	(size must be nxn)
    tracked_point.R = (r*r) * I2;	// covariance of measurement (size must be mxm)

    std::vector<cv::Point2f> all_origin_state, all_filter_state;
    for(unsigned int k = 0; k < uwb_observe_origin.size();k++)
    {
        cv::Point2f measPt = cv::Point2f(uwb_observe_origin[k].point3d.x, uwb_observe_origin[k].point3d.y );
        measure(0,0) = uwb_observe_origin[k].point3d.x;	// measured x position in 2D point
        measure(1,0) = uwb_observe_origin[k].point3d.y;	// measured y position in 2D point
        Eigen::MatrixXd z = tracked_point.measurement_function(measure); //make measurements

        tracked_point.ukf(x,z);

        all_origin_state.push_back(100 * measPt);
        cv::Point2f state_filter = cv::Point2f(x(0,0),x(1,0));
        all_filter_state.push_back(100 * state_filter);

        uwb_observe_filter[k].point3d.x = x(0,0);
        uwb_observe_filter[k].point3d.y = x(1,0);
    }

    likuncout << "Filter uwb data by UKF\n" << std::endl;

    cv::Mat image_show(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    for(size_t i = 0; i < all_origin_state.size()-1; i++)
        cv::line(image_show, all_origin_state[i], all_origin_state[i+1], cv::Scalar(0, 0, 0), 1);
    for(size_t i = 0; i < all_filter_state.size()-1; i++)
        cv::line(image_show, all_filter_state[i], all_filter_state[i+1], cv::Scalar(0, 0, 255), 1);

    cv::imwrite("image_show_ukf.png", image_show);
    cv::waitKey(0);

    return;
}

