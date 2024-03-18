
#include "../include/map.h"
#include "../include/feature.h"

namespace GL_SLAM
{

    void map_test()
        {
        std::cout<<"map is working "<<std::endl;
        }


    void Map::Insert_Key_Frame(Frame::Frame_Ptr frame_ptr)
    {
        Current_Frame_Ptr = frame_ptr;

        // if the keyframe id is not present in the keyframes map
        if(map_keyframes.find(frame_ptr->keyframe_id) == map_keyframes.end())
        {
            //then the keyframe pointer is added with its id as a map key in the keyframe map
            map_keyframes.insert(make_pair(frame_ptr->keyframe_id, frame_ptr));
            //the keyframe pointer is added with its id as a map key in the active keyframe map
            map_active_keyframes.insert(make_pair(frame_ptr->keyframe_id, frame_ptr));
        }

        else
        {
            //if the keyframe is already present in the map then we just update the entry
            map_keyframes[frame_ptr->keyframe_id] = frame_ptr;
            map_active_keyframes[frame_ptr->keyframe_id] = frame_ptr;
        }

        if(map_active_keyframes.size() > num_active_keyframes)
        {
            Remove_Old_Key_Frames();
        }
    }


    void Map::Insert_Map_Point(MapPoint::MapPoint_Ptr mappoint_ptr)
    {

        // if the landmark id is not present in the landmarks map
        if(map_landmarks.find(mappoint_ptr->map_point_id) == map_landmarks.end())
        {
            //then the mappoint pointer is added with its id as a map key in the mappoint map
            map_landmarks.insert(make_pair(mappoint_ptr->map_point_id, mappoint_ptr));
            //the mappoint pointer is added with its id as a map key in the active mappoint map
            map_active_landmarks.insert(make_pair(mappoint_ptr->map_point_id, mappoint_ptr));
        }

        else
        {
            //if the mappoint is already present in the map then we just update the entry
            map_landmarks[mappoint_ptr->map_point_id] = mappoint_ptr;
            map_active_landmarks[mappoint_ptr->map_point_id] = mappoint_ptr;
        }
    }


    void Map::Remove_Old_Key_Frames()
    {
        if(Current_Frame_Ptr == nullptr) // if the currnet frame is null then don't complete the loop
            return;

        double max_distance = 0;
        double min_distance = 9999;
        double max_keyframe_id = 0;
        double min_keyframe_id = 0;
        const double min_distance_threshold = 0.2;
        auto Twc = Current_Frame_Ptr->Get_Pose().inverse(); // get camera pose in the world
        for(auto& kf: map_active_keyframes)
        {
            if(kf.second == Current_Frame_Ptr)
                continue;

            auto distance  = (kf.second->Get_Pose() * Twc).log().norm();
            if(distance > max_distance)
            {
                max_distance = distance;
                max_keyframe_id = kf.first;
            }

            if(distance < min_distance)
            {
                min_distance = distance;
                min_keyframe_id = kf.first;
            }

            Frame::Frame_Ptr frame_to_remove = nullptr;

            // first remove the most recent "nearest"frames -- according to distance from the current frame
            if(min_distance < min_distance_threshold)
            {
                frame_to_remove = map_keyframes.at(min_keyframe_id);

            }

            // remove the farthest key frame from the current keyframe
            else
            {
                frame_to_remove = map_keyframes.at(max_keyframe_id);
            }
            map_active_keyframes.erase(frame_to_remove->keyframe_id);



        }

    }

}
