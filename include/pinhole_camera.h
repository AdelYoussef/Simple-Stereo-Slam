#pragma once

#ifndef GL_SLAM_PINHOLE_CAMERA_H
#define GL_SLAM_PINHOLE_CAMERA_H

#include "../include/common_include.h"

namespace GL_SLAM
{
    void test_pinhole();

    class PinHole_Camera
    {
        public:
        //for aligned memory allocation.
        //It ensures that memory allocated for Eigen types is aligned to the necessary boundaries for efficient operations.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // create smart pointer to Camera Class
        // smart pointers provides automatic memory management, including reference counting, and helps prevent memory leaks.
        typedef std::shared_ptr<PinHole_Camera> Camera_Ptr;


        // camera intrinsics
        double Cam_fx_ = 0;
        double Cam_fy_ = 0;
        double Cam_cx_ = 0;
        double Cam_cy_ = 0;
        double Cam_baseline_ = 0;

        SOPHUS_DOUBLE_SE3 Cam_Pose_;
        SOPHUS_DOUBLE_SE3 Cam_Inv_Pose_;

        PinHole_Camera() {}

        PinHole_Camera(double fx , double fy , double cx , double cy , double baseline , const SOPHUS_DOUBLE_SE3 &Pose):
        Cam_fx_(fx)  , Cam_fy_(fy)  , Cam_cx_(cx)  , Cam_cy_(cy)  , Cam_baseline_(baseline)
        {
            Cam_Inv_Pose_ = Cam_Pose_.inverse();
        }

        SOPHUS_DOUBLE_SE3 Get_Pose() const {
            return Cam_Pose_;
        }

        EIGEN_DOUBLE_MAT33 Get_Camera_Matrix() const {
            EIGEN_DOUBLE_MAT33 Camera_Matrix_;
            Camera_Matrix_ << Cam_fx_ , 0 , Cam_cx_ , 0 , Cam_fy_ , Cam_cy_ , 0 , 0 , 1;
            return Camera_Matrix_;
        }

        EIGEN_DOUBLE_VEC3 World_To_Camera(const EIGEN_DOUBLE_VEC3 &P_W , const SOPHUS_DOUBLE_SE3 &T_C_W);

        EIGEN_DOUBLE_VEC3 Camera_To_World(const EIGEN_DOUBLE_VEC3 &P_C , const SOPHUS_DOUBLE_SE3 &T_C_W);

        EIGEN_DOUBLE_VEC2 Camera_To_Pixel(const EIGEN_DOUBLE_VEC3 &P_C );

        EIGEN_DOUBLE_VEC3 Pixel_To_Camera(const EIGEN_DOUBLE_VEC2 &P_P , double depth = 1);

        EIGEN_DOUBLE_VEC3 Pixel_To_World(const EIGEN_DOUBLE_VEC2 &P_P , const SOPHUS_DOUBLE_SE3 &T_C_W , double depth = 1);

        EIGEN_DOUBLE_VEC2 World_To_Pixel(const EIGEN_DOUBLE_VEC3 &P_W , const SOPHUS_DOUBLE_SE3 &T_C_W);

    };

}

#endif