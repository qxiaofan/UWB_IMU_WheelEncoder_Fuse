#ifndef KARMAN_FILTER_H
#define KARMAN_FILTER_H


#include "common_include.h"
#include "load_data.h"


class UKF {
public:
    void ukf(Eigen::MatrixXd &x, const Eigen::MatrixXd z);
    virtual Eigen::MatrixXd state_function (Eigen::MatrixXd x) = 0;
    virtual Eigen::MatrixXd measurement_function (Eigen::MatrixXd x) = 0;

    int n;      //number of state
    int m;      //number of measurement
    Eigen::MatrixXd Q;	//noise covariance of process	(size must be nxn)
    Eigen::MatrixXd R;	//noise covariance of measurement (size must be mxm)
    Eigen::MatrixXd P;	//state covariance
};


#endif // !KARMAN_FILTER_H
