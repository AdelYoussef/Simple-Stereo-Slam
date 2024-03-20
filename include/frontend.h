
#pragma once

#ifndef GL_SLAM_FRONTEND_H
#define GL_SLAM_FRONTEND_H

#include "../include/common_include.h"
#include "../include/frame.h"
#include "../include/map.h"

namespace GL_SLAM
{
    class BackEnd;
    class Viewer;

    enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

    void frontend_test();


    class FrontEnd
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<FrontEnd> FrontEnd_Ptr;

        FrontEnd();

        bool Add_Frame(Frame::Frame_Ptr Frame_Ptr);

        void Set_Map(Map::Map_Ptr Map_Ptr)
        {
            Map_ = Map_Ptr;
        }

        void Set_BackEnd(std::shared_ptr<BackEnd> BackEnd_Ptr)
        {
            BackEnd_ = BackEnd_Ptr;
        }

        void Set_Viewer(std::shared_ptr<Viewer> Viewer_Ptr)
        {
            Viewer_ = Viewer_Ptr;
        }

        FrontendStatus Get_Status() const { return Status_; }

        void Set_Cameras (PinHole_Camera::Camera_Ptr Left, PinHole_Camera::Camera_Ptr Right)
        {
            Left_Camera_ = Left;
            Right_Camera_ = Right;
        }

        private:

        // tracking in noraml mode
        // returns true if success
        bool Track();

        // resets when lost
        // returns true if success
        bool Reset();

        // track using last keyframe
        // returns number of tracked points
        int Track_Last_Frame();

        // estimates current keyframe pose
        // returns number of inliers
        int Estimat_Current_Pose();

        // set the current frame as keyframe and insert it into backend
        // returns true if success
        bool Insert_Key_frame();

        // Try init the frontend with stereo images saved in current_frame_
        //return true if success
        bool Stereo_Init();

        // Detect features in left image in current_frame_
        // keypoints will be saved in current_frame_
        int Detect_Features();

        //Find the corresponding features in right image of current_frame_
        //return num of features found
        int Find_Features_In_Right();

        // Build the initial map with single image
        //return true if succeed
        bool Build_Init_Map();

        //Triangulate the 2D points in current frame
        //return num of triangulated points
        int Triangulate_New_Points();

        //Set the features in keyframe as new observation of the map points
        void Set_Observations_For_KeyFrame();


        FrontendStatus Status_ = FrontendStatus::INITING; // inital state of frontend

        Frame::Frame_Ptr Current_Frame_ = nullptr;  // current frame pointer
        Frame::Frame_Ptr Last_Frame_ = nullptr;     // Previous frame
        PinHole_Camera::Camera_Ptr Left_Camera_ = nullptr;   // left camera
        PinHole_Camera::Camera_Ptr Right_Camera_ = nullptr;   // right camera

        Map::Map_Ptr Map_ = nullptr;
        std::shared_ptr<BackEnd> BackEnd_ = nullptr;
        std::shared_ptr<Viewer> Viewer_ = nullptr;

        //The relative motion between the current frame and the previous frame (Last Frame),
        // used to estimate the initial pose value of the current frame
        SOPHUS_DOUBLE_SE3 Relative_Motion_;  

        int Tracking_Inliers_ = 0;  // inliers, used for testing new keyframes

        // params
        int Num_Features_ = 200;
        int Num_Features_Init_ = 100;
        int Num_Features_Tracking_ = 50;
        int Num_Features_Tracking_Bad_ = 20;
        int Num_Features_Needed_For_KeyFrame_ = 80;

        // utilities
        cv::Ptr<cv::GFTTDetector> GFTT_;  // feature detector in opencv

    };
}

#endif