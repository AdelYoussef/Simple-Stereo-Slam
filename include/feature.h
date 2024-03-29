
#pragma once



#ifndef GL_SLAM_FEATURE_H
#define GL_SLAM_FEATURE_H

#include "../include/common_include.h"


/*
-- The feature’s main information is its 2D position and several flags describing
whether it is an abnormal point.
-- We can access the host frame and its corresponding map point through a feature object.
-- However, the real ownership of frame and map point objects belongs to the map.
-- In order to avoid the circular reference generated by shared_ptr, the weak_ptr is used
 */



namespace GL_SLAM
{
    void feature_test();

    struct Frame;
    struct MapPoint;

    struct Feature
    {

        //for aligned memory allocation.
        //It ensures that memory allocated for Eigen types is aligned to the necessary boundaries for efficient operations.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // create smart pointer to Feature struct
        // smart pointers provides automatic memory management, including reference counting, and helps prevent memory leaks.
        typedef std::shared_ptr<Feature> Feature_Ptr;

        /*
        --create as smart pointer to frame struct

        --weak pointers doesnt contribute to the reference count of the managed object

        --this is often used in scenarios where you want to observe or access the Frame object
          without preventing it from being deleted when all strong references are gone.

        --The primary application of weak_ptr is to prevent circular references. 
        --When an object wants to reference another object without owning it,it can use weak_ptr. 
         This ensures that no circular references are created and objects can be safely freed when they are no longer needed.
         
         --the frame that takes this feature
        */
        std::weak_ptr<Frame> Holding_Frame_Ptr;

        //the assigned map point to this feature
        std::weak_ptr<MapPoint> Map_Point_Ptr ;

        //2d position of feature pixel
        CV_KEKYPOINT KeyPoint_Pixel_Position;


        bool Is_Outlier = false;

        //default constructor for the Feature struct
        //It initializes an instance of the struct without any parameters
        Feature(){}

        //initializer constructor for the Feature struct
        Feature(std::shared_ptr<Frame> frame_ptr , const CV_KEKYPOINT & keypoint_pixel_position)
            :Holding_Frame_Ptr(frame_ptr), KeyPoint_Pixel_Position(keypoint_pixel_position) {}
    };
}


#endif
