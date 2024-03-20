#include "../include/pinhole_camera.h"

namespace GL_SLAM 
{
    void test_pinhole()
    {
        std::cout<<"pinhole camera is working "<<std::endl;
    }


    // PinHole_Camera::PinHole_Camera() {}

    EIGEN_DOUBLE_VEC3 PinHole_Camera::World_To_Camera(const EIGEN_DOUBLE_VEC3 &P_W , const SOPHUS_DOUBLE_SE3 &T_C_W)
    {
        return Cam_Pose_ * T_C_W * P_W;
    }

    EIGEN_DOUBLE_VEC3 PinHole_Camera::Camera_To_World(const EIGEN_DOUBLE_VEC3 &P_C , const SOPHUS_DOUBLE_SE3 &T_C_W) 
    {
        return T_C_W.inverse() * Cam_Inv_Pose_ * P_C;
    }

    EIGEN_DOUBLE_VEC2 PinHole_Camera:: Camera_To_Pixel(const EIGEN_DOUBLE_VEC3 &P_C ) 
    {
        return EIGEN_DOUBLE_VEC2(Cam_fx_ * P_C(0, 0) / P_C(2, 0) + Cam_cx_,Cam_fy_ * P_C(1, 0) / P_C(2, 0) + Cam_cy_);
    }

    EIGEN_DOUBLE_VEC3 PinHole_Camera::Pixel_To_Camera(const EIGEN_DOUBLE_VEC2 &P_P, double depth) 
    {
        return EIGEN_DOUBLE_VEC3((P_P(0, 0) - Cam_cx_) * depth / Cam_fx_,(P_P(1, 0) - Cam_cy_) * depth / Cam_fy_,depth);
    }

    EIGEN_DOUBLE_VEC2 PinHole_Camera::World_To_Pixel(const EIGEN_DOUBLE_VEC3 &P_W , const SOPHUS_DOUBLE_SE3 &T_C_W) 
    {
        return Camera_To_Pixel(World_To_Camera(P_W, T_C_W));
    }

    EIGEN_DOUBLE_VEC3 PinHole_Camera::Pixel_To_World(const EIGEN_DOUBLE_VEC2 &P_P , const SOPHUS_DOUBLE_SE3 &T_C_W , double depth ) 
    {
        return Camera_To_World(Pixel_To_Camera(P_P, depth), T_C_W);
    }

}
