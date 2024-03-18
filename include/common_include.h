#pragma once
#ifndef GL_SLAM_COMMON_INCLUDE_H
#define GL_SLAM_COMMON_INCLUDE_H

// std
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <json/json.h>
#include <chrono>
#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>


// define the commonly included file to avoid a long include list
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SE3d SOPHUS_DOUBLE_SE3;
typedef Sophus::SO3d SOPHUS_DOUBLE_SO3;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/eigen.hpp"
typedef cv::Mat                                                 CV_MATRIX;
typedef cv::KeyPoint                                            CV_KEKYPOINT;
// glog
#include <glog/logging.h>


// typedefs for eigen
// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>   EIGEN_DOUBLE_MATXX;
typedef Eigen::Matrix<double, 10, 10>                           EIGEN_DOUBLE_MAT1010;
typedef Eigen::Matrix<double, 13, 13>                           EIGEN_DOUBLE_MAT1313;
typedef Eigen::Matrix<double, 8, 10>                            EIGEN_DOUBLE_MAT810;
typedef Eigen::Matrix<double, 8, 3>                             EIGEN_DOUBLE_MAT83;
typedef Eigen::Matrix<double, 6, 6>                             EIGEN_DOUBLE_MAT66;
typedef Eigen::Matrix<double, 5, 3>                             EIGEN_DOUBLE_MAT53;
typedef Eigen::Matrix<double, 4, 3>                             EIGEN_DOUBLE_MAT43;
typedef Eigen::Matrix<double, 4, 2>                             EIGEN_DOUBLE_MAT42;
typedef Eigen::Matrix<double, 3, 3>                             EIGEN_DOUBLE_MAT33;
typedef Eigen::Matrix<double, 2, 2>                             EIGEN_DOUBLE_MAT22;
typedef Eigen::Matrix<double, 8, 8>                             EIGEN_DOUBLE_MAT88;
typedef Eigen::Matrix<double, 7, 7>                             EIGEN_DOUBLE_MAT77;
typedef Eigen::Matrix<double, 4, 9>                             EIGEN_DOUBLE_MAT49;
typedef Eigen::Matrix<double, 8, 9>                             EIGEN_DOUBLE_MAT89;
typedef Eigen::Matrix<double, 9, 4>                             EIGEN_DOUBLE_MAT94;
typedef Eigen::Matrix<double, 9, 8>                             EIGEN_DOUBLE_MAT98;
typedef Eigen::Matrix<double, 8, 1>                             EIGEN_DOUBLE_MAT81;
typedef Eigen::Matrix<double, 1, 8>                             EIGEN_DOUBLE_MAT18;
typedef Eigen::Matrix<double, 9, 1>                             EIGEN_DOUBLE_MAT91;
typedef Eigen::Matrix<double, 1, 9>                             EIGEN_DOUBLE_MAT19;
typedef Eigen::Matrix<double, 8, 4>                             EIGEN_DOUBLE_MAT84;
typedef Eigen::Matrix<double, 4, 8>                             EIGEN_DOUBLE_MAT48;
typedef Eigen::Matrix<double, 4, 4>                             EIGEN_DOUBLE_MAT44;
typedef Eigen::Matrix<double, 3, 4>                             EIGEN_DOUBLE_MAT34;
typedef Eigen::Matrix<double, 14, 14>                           EIGEN_DOUBLE_MAT1414;

// float matricies
typedef Eigen::Matrix<float, 3, 3>                              EIGEN_FLOAT_MAT33;
typedef Eigen::Matrix<float, 10, 3>                             EIGEN_FLOAT_MAT103;
typedef Eigen::Matrix<float, 2, 2>                              EIGEN_FLOAT_MAT22;
typedef Eigen::Matrix<float, 3, 1>                              EIGEN_FLOAT_VEC3;
typedef Eigen::Matrix<float, 2, 1>                              EIGEN_FLOAT_VEC2;
typedef Eigen::Matrix<float, 6, 1>                              EIGEN_FLOAT_VEC6;
typedef Eigen::Matrix<float, 1, 8>                              EIGEN_FLOAT_MAT18;
typedef Eigen::Matrix<float, 6, 6>                              EIGEN_FLOAT_MAT66;
typedef Eigen::Matrix<float, 8, 8>                              EIGEN_FLOAT_MAT88;
typedef Eigen::Matrix<float, 8, 4>                              EIGEN_FLOAT_MAT84;
typedef Eigen::Matrix<float, 6, 6>                              EIGEN_FLOAT_MAT66;
typedef Eigen::Matrix<float, 4, 4>                              EIGEN_FLOAT_MAT44;
typedef Eigen::Matrix<float, 12, 12>                            EIGEN_FLOAT_MAT1212;
typedef Eigen::Matrix<float, 13, 13>                            EIGEN_FLOAT_MAT1313;
typedef Eigen::Matrix<float, 10, 10>                            EIGEN_FLOAT_MAT1010;
typedef Eigen::Matrix<float, 9, 9>                              EIGEN_FLOAT_MAT99;
typedef Eigen::Matrix<float, 4, 2>                              EIGEN_FLOAT_MAT42;
typedef Eigen::Matrix<float, 6, 2>                              EIGEN_FLOAT_MAT62;
typedef Eigen::Matrix<float, 1, 2>                              EIGEN_FLOAT_MAT12;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>    EIGEN_FLOAT_MATXX;
typedef Eigen::Matrix<float, 14, 14>                            EIGEN_FLOAT_MAT1414;

// double vectors
typedef Eigen::Matrix<double, 14, 1>                            EIGEN_DOUBLE_VEC14;
typedef Eigen::Matrix<double, 13, 1>                            EIGEN_DOUBLE_VEC13;
typedef Eigen::Matrix<double, 10, 1>                            EIGEN_DOUBLE_VEC10;
typedef Eigen::Matrix<double, 9, 1>                             EIGEN_DOUBLE_VEC9;
typedef Eigen::Matrix<double, 8, 1>                             EIGEN_DOUBLE_VEC8;
typedef Eigen::Matrix<double, 7, 1>                             EIGEN_DOUBLE_VEC7;
typedef Eigen::Matrix<double, 6, 1>                             EIGEN_DOUBLE_VEC6;
typedef Eigen::Matrix<double, 5, 1>                             EIGEN_DOUBLE_VEC5;
typedef Eigen::Matrix<double, 4, 1>                             EIGEN_DOUBLE_VEC4;
typedef Eigen::Matrix<double, 3, 1>                             EIGEN_DOUBLE_VEC3;
typedef Eigen::Matrix<double, 2, 1>                             EIGEN_DOUBLE_VEC2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1>                EIGEN_DOUBLE_VECX;

// float vectors
typedef Eigen::Matrix<float, 12, 1>                             EIGEN_FLOAT_VEC12;
typedef Eigen::Matrix<float, 8, 1>                              EIGEN_FLOAT_VEC8;
typedef Eigen::Matrix<float, 10, 1>                             EIGEN_FLOAT_VEC10;
typedef Eigen::Matrix<float, 4, 1>                              EIGEN_FLOAT_VEC4;
typedef Eigen::Matrix<float, 12, 1>                             EIGEN_FLOAT_VEC12;
typedef Eigen::Matrix<float, 13, 1>                             EIGEN_FLOAT_VEC13;
typedef Eigen::Matrix<float, 9, 1>                              EIGEN_FLOAT_VEC9;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1>                 EIGEN_FLOAT_VECX;
typedef Eigen::Matrix<float, 14, 1>                             EIGEN_FLOAT_VEC14;


#endif  // MYSLAM_COMMON_INCLUDE_H
