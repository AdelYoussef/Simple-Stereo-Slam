
#pragma once


#ifndef GL_SLAM_FRAME_H
#define GL_SLAM_FRAME_H


#include "../include/common_include.h"
#include "../include/pinhole_camera.h"


/*
-- We define the frame struct to contain id, pose, image, and features in the images.
-- Among them, the pose will be set or accessed by the front and backends simultaneously.
-- we define the pose set and get functions and lock the data
-- a frame can be constructed by a static function, and we can automatically set its id in the static create function.
*/



//forward decleraion for the feature struct to used a shared pointer


namespace GL_SLAM

{
    struct MapPoint;
    struct Feature;
    void frame_test();

    struct Frame  // Each frame is assigned a unique ID, and keyframes are assigned keyframe IDs.

    {
        //for aligned memory allocation.
        //It ensures that memory allocated for Eigen types is aligned to the necessary boundaries for efficient operations.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // create smart pointer to Frame struct
        // smart pointers provides automatic memory management, including reference counting, and helps prevent memory leaks.
        typedef std::shared_ptr<Frame> Frame_Ptr;

        unsigned long Frame_Id = 0; // unique id of this frames
        unsigned long Keyframe_Id = 0 ; // unique id of keyframe
        bool Is_Keyframe = false; // whether this frame is a keyframe
        double Time_Stamp;  // timestamp of an image sequence -- for record.
        SOPHUS_DOUBLE_SE3 Camera_Pose; // pose defined as Tcw --> world to camera
        std::mutex Pose_Mutex;  // mutex used for datalock -- to prevent change while in multi-threading

        CV_MATRIX Left_Image; // left image in the stereo system
        CV_MATRIX Right_Image; // right image in the stereo system

        // create smart pointer to Feature struct for extracted features in the left image
        std::vector<std::shared_ptr<Feature>> Feature_Ptr_Left;

        // create smart pointer to Feature struct for extracted features in the right image -- nullptr if not used
        std::vector<std::shared_ptr<Feature>> Feature_Ptr_Right;


        //default constructor for the Frame struct
        //It initializes an instance of the struct without any parameters
        Frame() {}

        //initializer constructor for the Frame struct
        Frame(long id, double time_stamp,  const SOPHUS_DOUBLE_SE3 &pose, const CV_MATRIX &left_img, const CV_MATRIX &right_img);

        // get pose using thread save process
        SOPHUS_DOUBLE_SE3 Get_Pose()
        {
            std::unique_lock<std::mutex> lck(Pose_Mutex);
            return Camera_Pose;
        }

        // set pose using a save
        void Set_Pose(const SOPHUS_DOUBLE_SE3 &pose)
        {
            std::unique_lock<std::mutex> lck(Pose_Mutex);
            Camera_Pose = pose;
        }

        // set keyframes and assign ids
        void Set_Keyframe();

        //Factory build mode, assign id
        static std::shared_ptr<Frame> Create_Frame();


    };
}
#endif
