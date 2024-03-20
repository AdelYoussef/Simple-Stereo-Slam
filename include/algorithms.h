
#ifndef GL_SLAM_ALGORITHMS_H
#define GL_SLAM_ALGORITHMS_H

#include "../include/common_include.h"

namespace GL_SLAM
{
    void test_algorithms()
    {
        std::cout<<" algorithms is working "<<std::endl;
    }

    inline bool Triangulation(const std::vector<SOPHUS_DOUBLE_SE3> &Poses, const std::vector<EIGEN_DOUBLE_VEC3> Points , EIGEN_DOUBLE_VEC3 &PT_World)
    {
        EIGEN_DOUBLE_MATXX A(2 * Poses.size() , 4);
        EIGEN_DOUBLE_VECX b(2 * Poses.size());
        b.setZero();

        for(size_t index ; index<Poses.size() ; ++index)
        {
            EIGEN_DOUBLE_MAT34 Mat = Poses[index].matrix3x4();

            A.block<1,4>(2 * index , 0) = Points[index][0] * Mat.row(2) - Mat.row(0);
            A.block<1,4>(2 * index + 1  , 0) = Points[index][1] * Mat.row(2) - Mat.row(1);
        }

        auto SVD = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        PT_World = (SVD.matrixV().col(3) / SVD.matrixV()(3 , 3)).head<3>();

        if(SVD.singularValues()[3] / SVD.singularValues()[2] < 1e-2) {
        return true;
    }
    return false;
    
    }
}
#endif