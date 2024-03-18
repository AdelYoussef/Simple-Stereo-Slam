

#pragma once


#ifndef GL_SLAM_MAP_H
#define GL_SLAM_MAP_H

#include "../include/common_include.h"
#include "../include/frame.h"
#include "../include/mappoint.h"

namespace GL_SLAM
{
    void map_test();

    // Interaction with the map: The front end calls InsertKeyframe and InsertMapPoint to insert new frames and map points,
    //and the back end maintains the map structure, determines outlier/elimination,

    class Map
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            typedef std::shared_ptr<Map> Map_Ptr;
            // creating a map where the stored values are smart pointers from MapPoint represendting landmarks
            //and the unique key for each elemnts is unsigned long
            typedef std::unordered_map<unsigned long, MapPoint::MapPoint_Ptr> LandmarksType;
            // creating a map where the stored values are smart pointers from Frame represendting keyframes
            //and the unique key for each elemnts is unsigned long
            typedef std::unordered_map<unsigned long, Frame::Frame_Ptr> KeyframesType;

            void Insert_Key_Frame(Frame::Frame_Ptr key_frame);

            void Insert_Map_Point(MapPoint::MapPoint_Ptr);

            LandmarksType Get_All_Map_Points()
            {
                std::unique_lock<std::mutex> lck(Map_Mutex);
                return Map_Landmarks;
            }


            KeyframesType Get_All_Key_Frames()
            {
                std::unique_lock<std::mutex> lck(Map_Mutex);
                return Map_Keyframes;
            }


            LandmarksType Get_Active_Map_Points()
            {
                std::unique_lock<std::mutex> lck(Map_Mutex);
                return Map_Active_landmarks;
            }


            KeyframesType Get_Active_Key_Frames()
            {
                std::unique_lock<std::mutex> lck(Map_Mutex);
                return Map_Active_keyframes;
            }


            void Clean_Map();

        private:

            void Remove_Old_Key_Frames();

            std::mutex Map_Mutex;

            LandmarksType Map_Landmarks; // all landmakrks
            LandmarksType Map_Active_landmarks; // active landmarks only
            KeyframesType Map_Keyframes; // all keyframes
            KeyframesType Map_Active_keyframes; // active keyframes only

            Frame::Frame_Ptr Current_Frame_Ptr = nullptr;

            int Num_Active_Keyframes = 7;





    };




}


#endif
