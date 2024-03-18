

#include"../include/frame.h"
#include"../include/feature.h"
//#include"../include/map.h"
#include"../include/mappoint.h"

int main() {


    std::cout<<"building GL SLAM  YA GAAAAAAMED"<<std::endl;
    GL_SLAM::frame_test();
    GL_SLAM::feature_test();
    //GL_SLAM::map_test();
    GL_SLAM::mappoint_test();
    
    return 0;
}
