/*
 * @Description: 通用头文件和宏定义
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-09 19:13:10
 */

#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

// define the commonly included file to avoid a long include list

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

// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
typedef Eigen::Matrix<double, 10, 10> Matrix1010;
typedef Eigen::Matrix<double, 13, 13> Matrix1313;
typedef Eigen::Matrix<double, 8, 10> Matrix810;
typedef Eigen::Matrix<double, 8, 3> Matrix83;
typedef Eigen::Matrix<double, 6, 6> Matrix66;
typedef Eigen::Matrix<double, 5, 3> Matrix53;
typedef Eigen::Matrix<double, 4, 3> Matrix43;
typedef Eigen::Matrix<double, 4, 2> Matrix42;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 2, 2> Matrix22;
typedef Eigen::Matrix<double, 8, 8> Matrix88;
typedef Eigen::Matrix<double, 7, 7> Matrix77;
typedef Eigen::Matrix<double, 4, 9> Matrix49;
typedef Eigen::Matrix<double, 8, 9> Matrix89;
typedef Eigen::Matrix<double, 9, 4> Matrix94;
typedef Eigen::Matrix<double, 9, 8> Matrix98;
typedef Eigen::Matrix<double, 8, 1> Matrix81;
typedef Eigen::Matrix<double, 1, 8> Matrix18;
typedef Eigen::Matrix<double, 9, 1> Matrix91;
typedef Eigen::Matrix<double, 1, 9> Matrix19;
typedef Eigen::Matrix<double, 8, 4> Matrix84;
typedef Eigen::Matrix<double, 4, 8> Matrix48;
typedef Eigen::Matrix<double, 4, 4> Matrix44;
typedef Eigen::Matrix<double, 3, 4> Matrix34;
typedef Eigen::Matrix<double, 14, 14> Matrix1414;

// float matricies
typedef Eigen::Matrix<float, 3, 3> Matrix33f;
typedef Eigen::Matrix<float, 10, 3> Matrix103f;
typedef Eigen::Matrix<float, 2, 2> Matrix22f;
typedef Eigen::Matrix<float, 1, 8> Matrix18f;
typedef Eigen::Matrix<float, 6, 6> Matrix66f;
typedef Eigen::Matrix<float, 8, 8> Matrix88f;
typedef Eigen::Matrix<float, 8, 4> Matrix84f;
typedef Eigen::Matrix<float, 6, 6> Matrix66f;
typedef Eigen::Matrix<float, 4, 4> Matrix44f;
typedef Eigen::Matrix<float, 12, 12> Matrix1212f;
typedef Eigen::Matrix<float, 13, 13> Matrix1313f;
typedef Eigen::Matrix<float, 10, 10> Matrix1010f;
typedef Eigen::Matrix<float, 9, 9> Matrix99f;
typedef Eigen::Matrix<float, 4, 2> Matrix42f;
typedef Eigen::Matrix<float, 6, 2> Matrix62f;
typedef Eigen::Matrix<float, 1, 2> Matrix12f;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXXf;
typedef Eigen::Matrix<float, 14, 14> Matrix1414f;

// double vectors
typedef Eigen::Matrix<double, 14, 1> Vector14d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

// float vectors
typedef Eigen::Matrix<float, 12, 1> Vector12f;
typedef Eigen::Matrix<float, 8, 1> Vector8f;
typedef Eigen::Matrix<float, 10, 1> Vector10f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;
typedef Eigen::Matrix<float, 12, 1> Vector12f;
typedef Eigen::Matrix<float, 13, 1> Vector13f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXf;
typedef Eigen::Matrix<float, 14, 1> Vector14f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 2, 1> Vector2f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

// for cv
#include <opencv2/core/core.hpp>

// glog
#include <glog/logging.h>

#endif //SLAM_COMMON_H
