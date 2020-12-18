//for std
#include "fstream"
#include "sstream"
#include "string"
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <list>

//for opencv
#include<opencv2/opencv.hpp>

//for eigen3
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Cholesky"

//for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_dense.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/types/se3quat.h>
#include <g2o/types/types_six_dof_expmap.h>
#include <g2o/types/types_sba.h>

#define yongcout std::cout << "yong: "
#define math_pi 3.141592653

using namespace std;