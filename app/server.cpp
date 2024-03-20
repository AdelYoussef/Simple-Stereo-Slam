

#include"../include/frame.h"
#include"../include/feature.h"
#include"../include/map.h"
#include"../include/mappoint.h"
#include"../include/frontend.h"
#include"../include/pinhole_camera.h"
#include"../include/algorithms.h"

int main() {


    std::cout<<"building GL SLAM  YA GAAAAAAMED BGD WALLAHY"<<std::endl;
    GL_SLAM::frame_test();
    GL_SLAM::feature_test();
    GL_SLAM::map_test();
    GL_SLAM::mappoint_test();
    GL_SLAM::frontend_test();
    GL_SLAM::test_pinhole();
    GL_SLAM::test_algorithms();
    
    return 0;
}
